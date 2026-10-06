"""An RViz interactive marker that the arm follows, using move_group's IK.

While the marker is dragged (and after it is released, until the arm arrives or stops making
progress), each cycle:

1. steps the commanded gripper pose toward the marker, limited to max_linear_speed and
   max_angular_speed;
2. solves IK for that pose with the pose group ('arm'); if there is no solution (with 5 joints,
   most orientations are unreachable), solves for the position alone with the position group
   ('arm_teleop', local search from the current joints);
3. if the step is not feasible, tries it again without its x, y or z part, so the gripper slides
   along the floor or the edge of the workspace instead of stopping;
4. rejects a solution that moves any joint more than max_joint_step (a jump to another IK
   branch), and streams the accepted joint positions to arm_controller.

IK is collision-aware (avoid_collisions), so with lekiwi_moveit_config the gripper stops at the
base and the ground. Nothing is published while the marker is idle, so planned motions from
move_group are never overridden. The marker follows the gripper while idle.
"""

import math
import threading

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from interactive_markers import InteractiveMarkerServer, MenuHandler
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    InteractiveMarkerFeedback, Marker)

MARKER_NAME = 'arm_target'
ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


# --- small pose helpers ---------------------------------------------------------------------

def position_distance(a, b):
    return math.dist((a.position.x, a.position.y, a.position.z),
                     (b.position.x, b.position.y, b.position.z))


def quaternion_angle(a, b):
    dot = abs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z)
    return 2.0 * math.acos(min(1.0, dot))


def slerp(a, b, t):
    """Spherical interpolation from quaternion a to b, t in [0, 1]."""
    dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z
    if dot < 0.0:
        b = Quaternion(w=-b.w, x=-b.x, y=-b.y, z=-b.z)
        dot = -dot
    if dot > 0.9995:
        q = [a.w + t * (b.w - a.w), a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)]
    else:
        theta = math.acos(dot)
        sa, sb = math.sin((1.0 - t) * theta), math.sin(t * theta)
        q = [(sa * c1 + sb * c2) / math.sin(theta)
             for c1, c2 in ((a.w, b.w), (a.x, b.x), (a.y, b.y), (a.z, b.z))]
    n = math.sqrt(sum(c * c for c in q))
    return Quaternion(w=q[0] / n, x=q[1] / n, y=q[2] / n, z=q[3] / n)


class ArmMarker(Node):

    def __init__(self):
        super().__init__('arm_marker')
        self.frame_id = self.declare_parameter('frame_id', 'so101_base_link').value
        self.ee_frame = self.declare_parameter('ee_frame', 'gripper_frame_link').value
        self.pose_group = self.declare_parameter('pose_group', 'arm').value
        self.position_group = self.declare_parameter('position_group', 'arm_teleop').value
        self.rate = self.declare_parameter('rate', 20.0).value
        self.max_linear_speed = self.declare_parameter('max_linear_speed', 0.08).value      # m/s
        self.max_angular_speed = self.declare_parameter('max_angular_speed', 0.8).value     # rad/s
        self.max_joint_step = self.declare_parameter('max_joint_step', 0.15).value          # rad/cycle
        self.ik_timeout = self.declare_parameter('ik_timeout', 0.02).value                  # s
        # the controller reaches each streamed point this long after receiving it
        self.command_lead = self.declare_parameter('command_lead', 0.1).value               # s
        # after release: stop when this close to the marker...
        self.position_tolerance = self.declare_parameter('position_tolerance', 0.003).value
        self.orientation_tolerance = self.declare_parameter('orientation_tolerance', 0.05).value
        # ...or when the distance to the marker has not shrunk for this long
        self.settle_timeout = self.declare_parameter('settle_timeout', 0.5).value
        self.marker_scale = self.declare_parameter('marker_scale', 0.12).value
        # a drag with no feedback for this long counts as released (in case MOUSE_UP is lost)
        self.drag_timeout = self.declare_parameter('drag_timeout', 1.0).value
        trajectory_topic = self.declare_parameter(
            'trajectory_topic', '/arm_controller/joint_trajectory').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        services = ReentrantCallbackGroup()
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=services)
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk', callback_group=services)
        self.trajectory_pub = self.create_publisher(JointTrajectory, trajectory_topic, 10)
        self.joint_positions = {}
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10,
                                 callback_group=services)

        # The default feedback queue holds one message, so a MOUSE_DOWN followed quickly by a
        # POSE_UPDATE loses the MOUSE_DOWN.
        self.server = InteractiveMarkerServer(self, 'arm_marker',
                                              feedback_sub_qos=QoSProfile(depth=100))
        self.menu = MenuHandler()
        self.menu.insert('Reset to gripper', callback=self.on_reset)

        self.lock = threading.Lock()
        self.marker_created = False
        self.tracking = False
        self.dragging = False
        self.target = None          # Pose in frame_id
        self.q_cmd = None           # commanded arm joint positions
        self.ee_cmd = None          # gripper pose at q_cmd
        self.best_error = math.inf
        self.last_progress = None
        self.last_feedback = None
        self.last_snap = None

        self.create_timer(1.0 / self.rate, self.on_timer,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f'waiting for /compute_ik and {self.frame_id} -> {self.ee_frame}')

    # --- ROS helpers -------------------------------------------------------------------------

    def on_joint_states(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position

    def ee_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.frame_id, self.ee_frame, Time())
        except TransformException:
            return None
        p = Pose()
        p.position.x = t.transform.translation.x
        p.position.y = t.transform.translation.y
        p.position.z = t.transform.translation.z
        p.orientation = t.transform.rotation
        return p

    def seed_state(self, q):
        state = GetPositionIK.Request().ik_request.robot_state
        state.is_diff = True   # every other joint (gripper, wheels) from the current state
        state.joint_state.name = list(ARM_JOINTS)
        state.joint_state.position = list(q)
        return state

    def solve_ik(self, group, pose, seed):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = group
        r.ik_link_name = self.ee_frame
        r.pose_stamped = PoseStamped(pose=pose)
        r.pose_stamped.header.frame_id = self.frame_id
        r.robot_state = self.seed_state(seed)
        r.avoid_collisions = True
        r.timeout = Duration(seconds=self.ik_timeout).to_msg()
        res = self.ik_client.call(req)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        js = res.solution.joint_state
        try:
            return [js.position[js.name.index(j)] for j in ARM_JOINTS]
        except ValueError:
            return None

    def forward_kinematics(self, q):
        req = GetPositionFK.Request()
        req.header.frame_id = self.frame_id
        req.fk_link_names = [self.ee_frame]
        req.robot_state = self.seed_state(q)
        res = self.fk_client.call(req)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        return res.pose_stamped[0].pose

    def publish(self, q):
        msg = JointTrajectory(joint_names=list(ARM_JOINTS))
        lead = Duration(seconds=self.command_lead).to_msg()
        msg.points = [JointTrajectoryPoint(
            positions=list(q), time_from_start=DurationMsg(sec=lead.sec, nanosec=lead.nanosec))]
        self.trajectory_pub.publish(msg)

    # --- marker ------------------------------------------------------------------------------

    def create_marker(self, pose):
        im = InteractiveMarker()
        im.header.frame_id = self.frame_id
        im.name = MARKER_NAME
        im.description = 'Arm target'
        im.scale = self.marker_scale
        im.pose = pose

        # a small sphere at the target: drag it freely in 3D
        sphere = Marker(type=Marker.SPHERE)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = self.marker_scale * 0.25
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.1, 0.6, 1.0, 0.8
        free = InteractiveMarkerControl(name='move_3d', always_visible=True,
                                        interaction_mode=InteractiveMarkerControl.MOVE_3D)
        free.markers.append(sphere)
        im.controls.append(free)

        # move and rotate along / about each axis
        for name, (w, x, y, z) in (('x', (1.0, 1.0, 0.0, 0.0)), ('y', (1.0, 0.0, 0.0, 1.0)),
                                   ('z', (1.0, 0.0, 1.0, 0.0))):
            for mode, prefix in ((InteractiveMarkerControl.MOVE_AXIS, 'move_'),
                                 (InteractiveMarkerControl.ROTATE_AXIS, 'rotate_')):
                c = InteractiveMarkerControl(name=prefix + name, interaction_mode=mode)
                c.orientation.w, c.orientation.x, c.orientation.y, c.orientation.z = w, x, y, z
                im.controls.append(c)

        im.controls.append(InteractiveMarkerControl(
            name='menu', interaction_mode=InteractiveMarkerControl.MENU))

        self.server.insert(im, feedback_callback=self.on_feedback)
        self.menu.apply(self.server, MARKER_NAME)
        self.server.applyChanges()
        self.marker_created = True
        self.last_snap = pose

    def snap_to_gripper(self, pose=None):
        pose = pose or self.ee_pose()
        if pose is None:
            return
        self.server.setPose(MARKER_NAME, pose)
        self.server.applyChanges()
        self.last_snap = pose

    def on_feedback(self, fb):
        with self.lock:
            now = self.get_clock().now()
            if fb.event_type in (InteractiveMarkerFeedback.MOUSE_DOWN,
                                 InteractiveMarkerFeedback.POSE_UPDATE):
                # a POSE_UPDATE also starts a drag, in case its MOUSE_DOWN was lost
                self.dragging = True
                self.target = fb.pose
                self.last_feedback = now
                if not self.tracking:
                    self.tracking = True
                    self.q_cmd = None   # start from the measured joints
                    self.get_logger().info('following the marker')
            elif fb.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                self.dragging = False
                if self.tracking:
                    self.target = fb.pose
                self.released(now)

    def on_reset(self, fb):
        with self.lock:
            if self.tracking:
                self.stop_tracking('reset from the menu')
            else:
                self.snap_to_gripper()

    def released(self, now):
        self.best_error = math.inf
        self.last_progress = now

    def error_to(self, target):
        # orientation counts little: with 5 joints it is often unreachable
        return (position_distance(self.ee_cmd, target)
                + 0.02 * quaternion_angle(self.ee_cmd.orientation, target.orientation))

    def stop_tracking(self, reason):
        self.tracking = False
        self.target = None
        self.get_logger().info(f'stopped: {reason}')

    # --- control -----------------------------------------------------------------------------

    def next_step(self, target):
        """Return (q, pose) for the next commanded state, or None if no step is feasible."""
        cur = self.ee_cmd
        d = [target.position.x - cur.position.x, target.position.y - cur.position.y,
             target.position.z - cur.position.z]
        dist = math.sqrt(sum(c * c for c in d))
        max_step = self.max_linear_speed / self.rate
        if dist > max_step:
            d = [c * max_step / dist for c in d]
        angle = quaternion_angle(cur.orientation, target.orientation)
        max_turn = self.max_angular_speed / self.rate
        orientation = slerp(cur.orientation, target.orientation,
                            1.0 if angle <= max_turn else max_turn / angle)

        def pose_with(delta, q):
            p = Pose(orientation=q)
            p.position.x = cur.position.x + delta[0]
            p.position.y = cur.position.y + delta[1]
            p.position.z = cur.position.z + delta[2]
            return p

        def accept(q):
            return q is not None and max(abs(a - b) for a, b in zip(q, self.q_cmd)) <= self.max_joint_step

        # 1. the whole step, exact pose
        q = self.solve_ik(self.pose_group, pose_with(d, orientation), self.q_cmd)
        if accept(q):
            return q
        # 2. position only: the whole step, then the step without its x, y or z part
        candidates = [d] + [[0.0 if i == axis else c for i, c in enumerate(d)]
                            for axis in range(3) if abs(d[axis]) > 1e-6]
        for delta in candidates:
            if max(abs(c) for c in delta) < 1e-6:
                continue
            q = self.solve_ik(self.position_group, pose_with(delta, cur.orientation), self.q_cmd)
            if accept(q):
                return q
        return None

    def on_timer(self):
        try:
            self.control_cycle()
        except Exception as e:  # noqa: BLE001 -- an executor would otherwise drop it silently
            self.get_logger().error(f'control cycle failed: {e!r}', throttle_duration_sec=1.0)

    def control_cycle(self):
        ee = self.ee_pose()
        if ee is None or not self.ik_client.service_is_ready():
            return
        if not self.marker_created:
            self.create_marker(ee)
            self.get_logger().info('marker ready')
            return

        with self.lock:
            if (self.dragging and self.last_feedback is not None and self.get_clock().now()
                    - self.last_feedback > Duration(seconds=self.drag_timeout)):
                self.dragging = False   # no MOUSE_UP: treat as released
                self.released(self.get_clock().now())
            tracking, dragging, target = self.tracking, self.dragging, self.target
        if not tracking:
            # follow the gripper (e.g. during planned motions) so the next drag starts from it
            if (position_distance(ee, self.last_snap) > 0.001
                    or quaternion_angle(ee.orientation, self.last_snap.orientation) > 0.01):
                self.snap_to_gripper(ee)
            return
        if target is None:
            return

        if self.q_cmd is None:
            if not all(j in self.joint_positions for j in ARM_JOINTS):
                return
            self.q_cmd = [self.joint_positions[j] for j in ARM_JOINTS]
            self.ee_cmd = self.forward_kinematics(self.q_cmd)
            if self.ee_cmd is None:
                self.q_cmd = None
                return

        reached = (position_distance(self.ee_cmd, target) < self.position_tolerance
                   and quaternion_angle(self.ee_cmd.orientation, target.orientation)
                   < self.orientation_tolerance)
        q = None if reached else self.next_step(target)
        now = self.get_clock().now()
        if q is not None:
            pose = self.forward_kinematics(q)
            if pose is not None:
                self.q_cmd, self.ee_cmd = q, pose
                self.publish(q)

        if dragging:
            return
        with self.lock:
            if not self.tracking:
                return
            error = self.error_to(target)
            if self.last_progress is None or error < self.best_error - 0.0005:
                self.best_error = min(self.best_error, error)
                self.last_progress = now
            if reached:
                self.stop_tracking('reached the marker')
            elif now - self.last_progress > Duration(seconds=self.settle_timeout):
                p_err = position_distance(self.ee_cmd, target)
                a_err = quaternion_angle(self.ee_cmd.orientation, target.orientation)
                self.stop_tracking(
                    f'as close as possible: {p_err * 1000:.0f} mm and {math.degrees(a_err):.0f} deg '
                    f'from the marker (out of reach, unreachable orientation, or blocked)')


def main():
    rclpy.init()
    node = ArmMarker()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
