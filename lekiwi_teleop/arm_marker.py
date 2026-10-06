"""An RViz interactive marker that the arm follows, using move_group's IK.

Modes (the marker's right-click menu, or the 'mode' parameter):

free    The fingertips follow the marker's position, and its orientation as far as 5 joints
        allow.
claw    Gripper pointing straight down. The marker moves in x/y/z; turning it about the vertical
        axis turns the gripper (wrist_roll, with shoulder_pan).
planar  shoulder_pan held at 0 and wrist_roll at planar_roll (-90 deg): the arm stays in the
        vertical x/z plane in front of it. The marker moves in x and z; turning it about y sets
        the gripper's pitch. Sideways motion is left to the omnidirectional base.

Velocity input (a joystick, lekiwi_teleop joy_teleop): ~/twist_cmd (TwistStamped, in frame_id;
linear.x, linear.z and angular.y) drives the planar target, and switches to planar first. The
target is the commanded fingertip pose moved ahead by the velocity times twist_lead, so it cannot
run away from an arm that is blocked. ~/goal (PoseStamped) is the current target, or the
fingertips when idle.

Entering claw or planar first moves the arm into the mode with a planned, collision-checked
motion (move_group). Then, each cycle while the marker is dragged (and after it is released,
until the arm arrives or stops getting closer):

1. Step the commanded fingertip pose toward the marker's target (the marker pose projected onto
   the mode): a fraction (tracking_gain / rate) of the remaining distance, so the arm eases into
   each marker update instead of jumping to it and stopping, limited to max_linear_speed and
   max_angular_speed.
2. Solve IK. free: the full pose with pose_group ('arm'), else the position alone with
   position_group ('arm_teleop', local). claw and planar: the full pose with teleop_pose_group
   ('arm_teleop_pose', local); only the mode's orientations are requested, and those are
   reachable.
3. If the step is blocked (the floor, the base, the edge of the reach), try it again with the
   orientation unchanged, then without its x, y or z part: the gripper slides along the obstacle.
4. Reject a solution that moves any joint more than max_joint_step (a jump to another IK branch),
   or that breaks the mode's locks.

Each cycle plans 'horizon' such steps ahead (6, 0.3 s) and sends them to arm_controller as one
trajectory that ends at rest. The next cycle replaces it, starting from where it is then, so the
arm only follows the first step or two; but a late cycle (Python cannot keep a steady 20 Hz) no
longer stops the arm, and if the node stops, the arm comes to rest at the end of the plan.

IK is collision-aware (avoid_collisions). Nothing is published while idle, so planned motions
from move_group are never overridden; the marker follows the gripper while idle.

The control loop runs on its own thread, at a fixed rate, not as an executor timer: rclpy's
executor starts timers late while it is busy with other callbacks.
"""

import math
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TwistStamped
from interactive_markers import InteractiveMarkerServer, MenuHandler
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import (ExternalShutdownException, MultiThreadedExecutor,
                             SingleThreadedExecutor)
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    InteractiveMarkerFeedback, Marker)

MARKER_NAME = 'arm_target'
ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
PAN, ROLL = 0, 4
MODES = ('free', 'claw', 'planar')


# --- rotations ------------------------------------------------------------------------------

def quat_to_mat(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(m):
    t = np.trace(m)
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        q = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    return Quaternion(w=float(q[0]), x=float(q[1]), y=float(q[2]), z=float(q[3]))


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


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


def make_pose(position, orientation):
    p = Pose(orientation=orientation)
    p.position.x, p.position.y, p.position.z = (float(v) for v in position)
    return p


def pose_to_mat(p):
    m = np.eye(4)
    m[:3, :3] = quat_to_mat(p.orientation)
    m[:3, 3] = (p.position.x, p.position.y, p.position.z)
    return m


def mat_to_pose(m):
    return make_pose(m[:3, 3], mat_to_quat(m[:3, :3]))


def axis_orientation(axis):
    """Orientation of an interactive marker control whose x axis points along 'x', 'y' or 'z'."""
    r = math.sqrt(0.5)
    return {'x': Quaternion(w=1.0), 'y': Quaternion(w=r, z=r), 'z': Quaternion(w=r, y=-r)}[axis]


class ArmMarker(Node):

    def __init__(self):
        super().__init__('arm_marker')
        initial_mode = self.declare_parameter('mode', 'free').value
        if initial_mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, not '{initial_mode}'")
        self.frame_id = self.declare_parameter('frame_id', 'so101_base_link').value
        # the point that follows the marker, and the groups' IK tip it is fixed to
        self.tip_frame = self.declare_parameter('tip_frame', 'gripper_tip_link').value
        self.ik_link = self.declare_parameter('ik_link', 'gripper_frame_link').value
        self.pose_group = self.declare_parameter('pose_group', 'arm').value
        self.position_group = self.declare_parameter('position_group', 'arm_teleop').value
        self.teleop_pose_group = self.declare_parameter('teleop_pose_group', 'arm_teleop_pose').value
        self.rate = self.declare_parameter('rate', 20.0).value
        self.max_linear_speed = self.declare_parameter('max_linear_speed', 0.08).value      # m/s
        self.max_angular_speed = self.declare_parameter('max_angular_speed', 0.8).value     # rad/s
        # each cycle, close this fraction per second of the remaining distance to the target
        # (a first-order lag with time constant 1 / tracking_gain), within the speed limits
        self.tracking_gain = self.declare_parameter('tracking_gain', 6.0).value             # 1/s
        self.max_joint_step = self.declare_parameter('max_joint_step', 0.15).value          # rad/cycle
        # planar: wrist_roll is held here (shoulder_pan at 0), within lock_tolerance
        self.planar_roll = self.declare_parameter('planar_roll', -math.pi / 2).value       # rad
        self.lock_tolerance = self.declare_parameter('lock_tolerance', 0.02).value          # rad
        self.ik_timeout = self.declare_parameter('ik_timeout', 0.02).value                  # s
        # steps planned ahead each cycle (horizon / rate seconds)
        self.horizon = self.declare_parameter('horizon', 6).value
        # after release: stop when this close to the target...
        self.position_tolerance = self.declare_parameter('position_tolerance', 0.003).value
        self.orientation_tolerance = self.declare_parameter('orientation_tolerance', 0.05).value
        # ...or when the distance to it has not shrunk for this long
        self.settle_timeout = self.declare_parameter('settle_timeout', 0.5).value
        # a drag with no feedback for this long counts as released (in case MOUSE_UP is lost)
        self.drag_timeout = self.declare_parameter('drag_timeout', 1.0).value
        # Entering claw: the gripper can point straight down only fairly low in front of the arm
        # (wrist_flex reaches +-95 deg), so the fingertips go down from where they are, in
        # claw_search_step steps, else to claw_ready (frame_id).
        self.claw_search_step = self.declare_parameter('claw_search_step', 0.02).value
        self.claw_ready = self.declare_parameter('claw_ready', [0.20, 0.0, 0.04]).value
        # velocity scaling of the planned motion that enters a mode
        self.transition_scaling = self.declare_parameter('transition_scaling', 0.3).value
        self.marker_scale = self.declare_parameter('marker_scale', 0.12).value
        trajectory_topic = self.declare_parameter(
            'trajectory_topic', '/arm_controller/joint_trajectory').value
        # velocity input: ignored when older than twist_timeout; the target is the commanded
        # pose moved ahead by velocity * twist_lead
        self.twist_timeout = self.declare_parameter('twist_timeout', 0.25).value            # s
        self.twist_lead = self.declare_parameter('twist_lead', 0.15).value                  # s
        # nothing moves until this controller is active (robot.launch.py starts everything at once)
        self.controller = self.declare_parameter('controller', 'arm_controller').value
        controller_manager = self.declare_parameter('controller_manager', '/controller_manager').value

        services = ReentrantCallbackGroup()
        # IK and FK are called from the control thread on a node of their own, spun only by that
        # thread, so a busy main executor cannot delay the responses
        # use_global_arguments=False: the launch's __node:=arm_marker remap would rename it too
        self.kinematics_node = rclpy.create_node('arm_marker_kinematics',
                                                 use_global_arguments=False)
        self.kinematics_executor = SingleThreadedExecutor()
        self.kinematics_executor.add_node(self.kinematics_node)
        self.ik_client = self.kinematics_node.create_client(GetPositionIK, '/compute_ik')
        self.fk_client = self.kinematics_node.create_client(GetPositionFK, '/compute_fk')
        self.move_client = ActionClient(self, MoveGroup, '/move_action', callback_group=services)
        self.list_controllers = self.create_client(
            ListControllers, controller_manager + '/list_controllers', callback_group=services)
        self.controller_active = False
        self.controller_query = None
        self.trajectory_pub = self.create_publisher(JointTrajectory, trajectory_topic, 10)
        self.joint_positions = {}
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10,
                                 callback_group=services)
        self.create_subscription(TwistStamped, '~/twist_cmd', self.on_twist, 10,
                                 callback_group=services)
        self.goal_pub = self.create_publisher(PoseStamped, '~/goal', 10)

        # The default feedback queue holds one message, so a MOUSE_DOWN followed quickly by a
        # POSE_UPDATE loses the MOUSE_DOWN.
        self.server = InteractiveMarkerServer(self, 'arm_marker',
                                              feedback_sub_qos=QoSProfile(depth=100))
        self.menu = MenuHandler()
        self.menu.insert('Reset to gripper', callback=self.on_reset)
        mode_menu = self.menu.insert('Mode')
        self.mode_entries = {m: self.menu.insert(m, parent=mode_menu, callback=self.on_mode_menu)
                             for m in MODES}

        self.lock = threading.Lock()
        self.mode = 'free'
        self.requested_mode = initial_mode if initial_mode != 'free' else None
        self.calibrated = False
        self.marker_created = False
        self.rebuild_marker = False
        self.transitioning = False
        self.tracking = False
        self.dragging = False
        self.target = None          # marker Pose in frame_id
        self.q_cmd = None           # commanded arm joint positions
        self.ee_cmd = None          # fingertip pose at q_cmd
        self.plan = []              # last sent trajectory: (time.monotonic(), q, fingertip pose)
        self.best_error = math.inf
        self.last_progress = None
        self.last_feedback = None
        self.last_snap = None
        self.snap_requested = False # put the marker back on the gripper (control thread)
        self.twist = None           # latest ~/twist_cmd, and its time.monotonic()
        self.twist_time = None
        self.joystick_active = False

        self.loop = threading.Thread(target=self.run_loop, daemon=True)
        self.get_logger().info('waiting for move_group and the joint states')

    # --- ROS helpers -------------------------------------------------------------------------

    def on_twist(self, msg):
        with self.lock:
            self.twist = msg
            self.twist_time = time.monotonic()

    def publish_goal(self, pose):
        msg = PoseStamped(pose=pose)
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        self.goal_pub.publish(msg)

    def on_joint_states(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position

    def measured_q(self):
        if not all(j in self.joint_positions for j in ARM_JOINTS):
            return None
        return [self.joint_positions[j] for j in ARM_JOINTS]

    def tip_pose(self):
        """The measured fingertip pose (forward kinematics of /joint_states)."""
        q = self.measured_q()
        if q is None or not self.fk_client.service_is_ready():
            return None
        return self.forward_kinematics(q)

    def call(self, client, request, timeout=1.0):
        """A service call from the control thread, spinning only the kinematics node."""
        future = client.call_async(request)
        self.kinematics_executor.spin_until_future_complete(future, timeout_sec=timeout)
        if not future.done():
            client.remove_pending_request(future)
            return None
        return future.result()

    def robot_state(self, q):
        state = GetPositionIK.Request().ik_request.robot_state
        state.is_diff = True   # every other joint (gripper, wheels) from the current state
        state.joint_state.name = list(ARM_JOINTS)
        state.joint_state.position = [float(v) for v in q]
        return state

    def solve_ik(self, group, tip_pose, seed, timeout=None):
        """IK for a fingertip pose; the groups' tip is ik_link, at a fixed offset from it."""
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = group
        r.ik_link_name = self.ik_link
        r.pose_stamped = PoseStamped(pose=mat_to_pose(pose_to_mat(tip_pose) @ self.tip_to_ik))
        r.pose_stamped.header.frame_id = self.frame_id
        r.robot_state = self.robot_state(seed)
        r.avoid_collisions = True
        r.timeout = Duration(seconds=timeout or self.ik_timeout).to_msg()
        res = self.call(self.ik_client, req)
        self.last_ik_error = None if res is None else res.error_code.val
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        js = res.solution.joint_state
        try:
            return [js.position[js.name.index(j)] for j in ARM_JOINTS]
        except ValueError:
            return None

    def forward_kinematics(self, q, links=None):
        req = GetPositionFK.Request()
        req.header.frame_id = self.frame_id
        req.fk_link_names = links or [self.tip_frame]
        req.robot_state = self.robot_state(q)
        res = self.call(self.fk_client, req)
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        poses = [p.pose for p in res.pose_stamped]
        return poses if links else poses[0]

    def calibrate(self):
        """Mode geometry from the robot model, with all arm joints at 0."""
        poses = self.forward_kinematics([0.0] * 5, [self.tip_frame, self.ik_link])
        if poses is None:
            return False
        tip, ik = poses
        self.tip_to_ik = np.linalg.inv(pose_to_mat(tip)) @ pose_to_mat(ik)
        # claw: pointing straight down with yaw 0 is the zero pose turned 90 deg about y (the axis
        # of shoulder_lift, elbow_flex and wrist_flex)
        self.r_down = rot_y(math.pi / 2) @ quat_to_mat(tip.orientation)
        # planar: with shoulder_pan = 0 and wrist_roll = planar_roll the fingertips move in this
        # vertical plane (they are off the roll axis, so it depends on the roll), and the gripper
        # pitches about y from r_planar
        planar = self.forward_kinematics([0.0, 0.0, 0.0, 0.0, self.planar_roll])
        if planar is None:
            return False
        self.r_planar = quat_to_mat(planar.orientation)
        self.plane_y = planar.position.y
        return True

    def send_plan(self, now, points):
        """Send (time.monotonic(), q, pose) points to arm_controller as one trajectory, ending at
        rest. Velocities from the neighbouring points, so the controller's splines pass through
        the points instead of stopping at each."""
        msg = JointTrajectory(joint_names=list(ARM_JOINTS))
        n = len(points)
        for i, (t, q, _) in enumerate(points):
            if i == n - 1:
                v = [0.0] * len(q)
            else:
                ta, qa = (points[i - 1][0], points[i - 1][1]) if i > 0 else (t, q)
                tb, qb = points[i + 1][0], points[i + 1][1]
                v = [(b - a) / (tb - ta) for a, b in zip(qa, qb)]
            d = Duration(seconds=t - now).to_msg()
            msg.points.append(JointTrajectoryPoint(
                positions=[float(x) for x in q], velocities=[float(x) for x in v],
                time_from_start=DurationMsg(sec=d.sec, nanosec=d.nanosec)))
        self.trajectory_pub.publish(msg)
        self.plan = points

    # --- modes -------------------------------------------------------------------------------

    def mode_target(self, marker):
        """The fingertip pose that the marker asks for in the current mode."""
        if self.mode == 'claw':
            m = quat_to_mat(marker.orientation)
            yaw = math.atan2(m[1, 0], m[0, 0])
            return make_pose((marker.position.x, marker.position.y, marker.position.z),
                             mat_to_quat(rot_z(yaw) @ self.r_down))
        if self.mode == 'planar':
            m = quat_to_mat(marker.orientation)
            pitch = math.atan2(m[0, 2], m[0, 0])
            return make_pose((marker.position.x, self.plane_y, marker.position.z),
                             mat_to_quat(rot_y(pitch) @ self.r_planar))
        return marker

    def marker_pose_for(self, tip):
        """The marker pose that shows a fingertip pose in the current mode."""
        r = quat_to_mat(tip.orientation)
        position = (tip.position.x, tip.position.y, tip.position.z)
        if self.mode == 'claw':
            m = r @ self.r_down.T
            return make_pose(position, mat_to_quat(rot_z(math.atan2(m[1, 0], m[0, 0]))))
        if self.mode == 'planar':
            m = r @ self.r_planar.T
            return make_pose((tip.position.x, self.plane_y, tip.position.z),
                             mat_to_quat(rot_y(math.atan2(m[0, 2], m[0, 0]))))
        return tip

    def mode_entry_goal(self, mode, q):
        """Joint positions that put the arm into a mode, starting from the joints q."""
        if mode == 'planar':
            return [0.0, q[1], q[2], q[3], self.planar_roll]
        # claw: pointing down at the fingertips' x/y, as high as possible from their current
        # height down to claw_ready's; else at claw_ready
        tip = self.forward_kinematics(q)
        if tip is None:
            return None
        down = mat_to_quat(rot_z(-q[PAN]) @ self.r_down)   # shoulder_pan turns about -z
        candidates = []
        z = tip.position.z
        while z >= self.claw_ready[2]:
            candidates.append((tip.position.x, tip.position.y, z))
            z -= self.claw_search_step
        candidates.append(tuple(self.claw_ready))
        for position in candidates:
            goal = self.solve_ik(self.pose_group, make_pose(position, down), q, timeout=0.1)
            if goal is not None:
                return goal
        return None

    def accept(self, q, q_from):
        if q is None or max(abs(a - b) for a, b in zip(q, q_from)) > self.max_joint_step:
            return False
        if self.mode == 'planar':
            return (abs(q[PAN]) <= self.lock_tolerance
                    and abs(q[ROLL] - self.planar_roll) <= self.lock_tolerance)
        return True

    # --- marker ------------------------------------------------------------------------------

    def create_marker(self, pose):
        im = InteractiveMarker()
        im.header.frame_id = self.frame_id
        im.name = MARKER_NAME
        im.description = f'Arm target ({self.mode})'
        im.scale = self.marker_scale
        im.pose = pose

        # a small sphere at the target
        sphere = Marker(type=Marker.SPHERE)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = self.marker_scale * 0.25
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.1, 0.6, 1.0, 0.8
        if self.mode == 'planar':
            # drag it within the x/z plane only
            free = InteractiveMarkerControl(name='move_xz', always_visible=True,
                                            orientation=axis_orientation('y'),
                                            orientation_mode=InteractiveMarkerControl.FIXED,
                                            interaction_mode=InteractiveMarkerControl.MOVE_PLANE)
        else:
            free = InteractiveMarkerControl(name='move_3d', always_visible=True,
                                            interaction_mode=InteractiveMarkerControl.MOVE_3D)
        free.markers.append(sphere)
        im.controls.append(free)

        moves = {'free': 'xyz', 'claw': 'xyz', 'planar': 'xz'}[self.mode]
        rotations = {'free': 'xyz', 'claw': 'z', 'planar': 'y'}[self.mode]
        # free: the handles turn with the marker; claw and planar: they stay aligned with the base
        orientation_mode = (InteractiveMarkerControl.INHERIT if self.mode == 'free'
                            else InteractiveMarkerControl.FIXED)
        for axis in moves:
            im.controls.append(InteractiveMarkerControl(
                name='move_' + axis, orientation=axis_orientation(axis),
                orientation_mode=orientation_mode,
                interaction_mode=InteractiveMarkerControl.MOVE_AXIS))
        for axis in rotations:
            im.controls.append(InteractiveMarkerControl(
                name='rotate_' + axis, orientation=axis_orientation(axis),
                orientation_mode=orientation_mode,
                interaction_mode=InteractiveMarkerControl.ROTATE_AXIS))
        im.controls.append(InteractiveMarkerControl(
            name='menu', interaction_mode=InteractiveMarkerControl.MENU))

        for m, entry in self.mode_entries.items():
            self.menu.setCheckState(entry, MenuHandler.CHECKED if m == self.mode
                                    else MenuHandler.UNCHECKED)
        self.server.erase(MARKER_NAME)
        self.server.insert(im, feedback_callback=self.on_feedback)
        self.menu.apply(self.server, MARKER_NAME)
        self.server.applyChanges()
        self.marker_created = True
        self.last_snap = pose

    def snap_to_gripper(self, tip=None):
        tip = tip or self.tip_pose()
        if tip is None:
            return
        pose = self.marker_pose_for(tip)
        self.server.setPose(MARKER_NAME, pose)
        self.server.applyChanges()
        self.last_snap = pose

    def on_feedback(self, fb):
        with self.lock:
            if self.transitioning:
                return
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
                    self.plan = []
                    self.get_logger().info(f'following the marker ({self.mode})')
            elif fb.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                self.dragging = False
                if self.tracking:
                    self.target = fb.pose
                self.released(now)

    def on_reset(self, fb):
        with self.lock:
            if self.tracking:
                self.stop_tracking('reset from the menu')
            self.snap_requested = True

    def on_mode_menu(self, fb):
        for m, entry in self.mode_entries.items():
            if entry == fb.menu_entry_id:
                with self.lock:
                    self.requested_mode = m
                    if self.tracking:
                        self.stop_tracking(f'switching to {m}')

    def released(self, now):
        self.best_error = math.inf
        self.last_progress = now

    def error_to(self, target):
        # orientation counts little: in free mode it is often unreachable
        return (position_distance(self.ee_cmd, target)
                + 0.02 * quaternion_angle(self.ee_cmd.orientation, target.orientation))

    def stop_tracking(self, reason):
        # the last plan ends at rest
        self.tracking = False
        self.target = None
        self.snap_requested = True
        self.get_logger().info(f'stopped: {reason}')

    def check_controller(self):
        """Poll the controller manager until the arm controller is active."""
        if self.controller_query is not None and not self.controller_query.done():
            return
        if self.controller_query is not None:
            res = self.controller_query.result()
            self.controller_query = None
            if res is not None and any(c.name == self.controller and c.state == 'active'
                                       for c in res.controller):
                self.controller_active = True
                self.get_logger().info(f'{self.controller} is active')
                return
        if self.list_controllers.service_is_ready():
            self.controller_query = self.list_controllers.call_async(ListControllers.Request())

    # --- mode transitions --------------------------------------------------------------------

    def start_transition(self, mode):
        q = self.measured_q()
        if q is None:
            return
        with self.lock:
            self.requested_mode = None
        if mode == 'free':
            self.mode = mode
            self.rebuild_marker = True
            self.get_logger().info('mode: free')
            return
        goal_q = self.mode_entry_goal(mode, q)
        if goal_q is None:
            self.get_logger().warn(f'cannot enter {mode}: no collision-free pose for it found')
            return
        if max(abs(a - b) for a, b in zip(goal_q, q)) < 0.01:
            # already there (a plan to the current state is empty, and the controller rejects it)
            self.mode = mode
            self.rebuild_marker = True
            self.get_logger().info(f'mode: {mode}')
            return
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = self.pose_group
        r.num_planning_attempts = 3
        r.allowed_planning_time = 5.0
        r.max_velocity_scaling_factor = self.transition_scaling
        r.max_acceleration_scaling_factor = self.transition_scaling
        r.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=float(p), tolerance_above=0.01,
                            tolerance_below=0.01, weight=1.0) for j, p in zip(ARM_JOINTS, goal_q)])]
        self.transitioning = True
        self.get_logger().info(f'entering {mode}: moving the arm into it')
        self.move_client.send_goal_async(goal).add_done_callback(
            lambda f: self.on_transition_accepted(f, mode))

    def on_transition_accepted(self, future, mode):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn(f'cannot enter {mode}: move_group rejected the motion')
            self.transitioning = False
            return
        handle.get_result_async().add_done_callback(lambda f: self.on_transition_done(f, mode))

    def on_transition_done(self, future, mode):
        code = future.result().result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.mode = mode
            self.get_logger().info(f'mode: {mode}')
        else:
            self.get_logger().warn(f'cannot enter {mode}: the move failed (MoveIt error {code})')
        self.rebuild_marker = True
        self.transitioning = False

    # --- control -----------------------------------------------------------------------------

    def next_step(self, target, q_from, cur):
        """(joint positions, fingertip pose) one step from (q_from, cur) toward target, or None
        if no step is feasible. The pose is None when it is not known exactly (a position-only
        solution)."""
        fraction = min(1.0, self.tracking_gain / self.rate)
        d = [fraction * (target.position.x - cur.position.x),
             fraction * (target.position.y - cur.position.y),
             fraction * (target.position.z - cur.position.z)]
        dist = math.sqrt(sum(c * c for c in d))
        max_step = self.max_linear_speed / self.rate
        if dist > max_step:
            d = [c * max_step / dist for c in d]
        angle = fraction * quaternion_angle(cur.orientation, target.orientation)
        max_turn = self.max_angular_speed / self.rate
        orientation = slerp(cur.orientation, target.orientation,
                            fraction * (1.0 if angle <= max_turn else max_turn / angle))

        def pose_with(delta, q):
            return make_pose((cur.position.x + delta[0], cur.position.y + delta[1],
                              cur.position.z + delta[2]), q)

        dropped = [[0.0 if i == axis else c for i, c in enumerate(d)]
                   for axis in range(3) if abs(d[axis]) > 1e-6]
        if self.mode == 'free':
            # the exact pose, else the position alone
            attempts = [(self.pose_group, d, orientation)]
            attempts += [(self.position_group, delta, cur.orientation) for delta in [d] + dropped]
        else:
            # the mode's orientations are reachable: always the full pose
            attempts = [(self.teleop_pose_group, d, orientation)]
            if angle > 1e-6:
                attempts.append((self.teleop_pose_group, d, cur.orientation))
            attempts += [(self.teleop_pose_group, delta, orientation) for delta in dropped]
        failures = []
        for group, delta, q_target in attempts:
            if max(abs(c) for c in delta) < 1e-6 and q_target is cur.orientation:
                continue
            pose = pose_with(delta, q_target)
            q = self.solve_ik(group, pose, q_from)
            if self.accept(q, q_from):
                # a full-pose solution is the requested pose (within the IK thresholds)
                return q, (None if group == self.position_group else pose)
            failures.append(f'IK error {self.last_ik_error}' if q is None
                            else f'rejected {[round(v - c, 3) for v, c in zip(q, q_from)]}')
        self.get_logger().debug(f'no feasible step ({self.mode}): {failures}')
        return None

    def run_loop(self):
        period = 1.0 / self.rate
        deadline = time.monotonic()
        while rclpy.ok():
            try:
                self.control_cycle()
            except Exception as e:  # noqa: BLE001 -- keep the loop alive, and say why
                self.get_logger().error(f'control cycle failed: {e!r}', throttle_duration_sec=1.0)
            deadline += period
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                deadline = time.monotonic()   # overran: start the next cycle now

    def control_cycle(self):
        tip = self.tip_pose()
        if tip is None or not self.ik_client.service_is_ready():
            return
        if not self.calibrated:
            self.calibrated = self.calibrate()
            return
        if not self.controller_active:
            self.check_controller()
            return
        if not self.marker_created or self.rebuild_marker:
            first = not self.marker_created
            self.rebuild_marker = False
            self.create_marker(self.marker_pose_for(tip))
            if first:
                self.get_logger().info('marker ready')
        if self.transitioning:
            return
        with self.lock:
            requested, tracking = self.requested_mode, self.tracking
        if requested is not None and not tracking:
            if requested == 'free' or self.move_client.server_is_ready():
                self.start_transition(requested)
            return

        # velocity input: drives the planar target while it is fresh and non-zero
        with self.lock:
            twist, twist_time = self.twist, self.twist_time
        twist_fresh = (twist is not None and time.monotonic() - twist_time < self.twist_timeout
                       and any(abs(v) > 1e-6 for v in (twist.twist.linear.x, twist.twist.linear.z,
                                                       twist.twist.angular.y)))
        if twist_fresh and self.mode != 'planar':
            with self.lock:
                if self.requested_mode != 'planar':
                    self.requested_mode = 'planar'
                    if self.tracking:
                        self.stop_tracking('velocity input: switching to planar')
                    self.get_logger().info('velocity input: switching to planar mode')
            return
        with self.lock:
            if twist_fresh:
                if not self.tracking:
                    self.tracking = True
                    self.q_cmd = None
                    self.plan = []
                    self.get_logger().info('following the velocity input (planar)')
                self.dragging = True
                self.last_feedback = self.get_clock().now()
                self.joystick_active = True
                if self.target is None:
                    self.target = self.marker_pose_for(tip)   # replaced once the anchor is known
            elif self.joystick_active:
                self.joystick_active = False
                self.dragging = False
                self.released(self.get_clock().now())

        with self.lock:
            if (self.dragging and self.last_feedback is not None and self.get_clock().now()
                    - self.last_feedback > Duration(seconds=self.drag_timeout)):
                self.dragging = False   # no MOUSE_UP: treat as released
                self.released(self.get_clock().now())
            tracking, dragging, marker = self.tracking, self.dragging, self.target
        if not tracking:
            # follow the gripper (e.g. during planned motions) so the next drag starts from it
            pose = self.marker_pose_for(tip)
            if (self.snap_requested or position_distance(pose, self.last_snap) > 0.001
                    or quaternion_angle(pose.orientation, self.last_snap.orientation) > 0.01):
                self.snap_requested = False
                self.snap_to_gripper(tip)
            self.publish_goal(tip)
            return
        if marker is None:
            return
        target = self.mode_target(marker)

        if self.q_cmd is None:
            self.q_cmd = self.measured_q()
            if self.q_cmd is None:
                return
            self.ee_cmd = self.forward_kinematics(self.q_cmd)
            if self.ee_cmd is None:
                self.q_cmd = None
                return
        # Continue from the running plan: its first point not yet reached (the controller is on
        # its way there), or its end if it has finished.
        clock = time.monotonic()
        anchor_time = clock
        if self.plan:
            upcoming = [p for p in self.plan if p[0] > clock + 0.005]
            anchor_time, self.q_cmd, self.ee_cmd = upcoming[0] if upcoming else self.plan[-1]
            anchor_time = max(anchor_time, clock)

        if self.joystick_active and twist is not None:
            # the commanded pose, moved ahead by the velocity
            v = twist.twist
            m = quat_to_mat(self.ee_cmd.orientation) @ self.r_planar.T
            pitch = math.atan2(m[0, 2], m[0, 0]) + v.angular.y * self.twist_lead
            marker = make_pose((self.ee_cmd.position.x + v.linear.x * self.twist_lead, self.plane_y,
                                self.ee_cmd.position.z + v.linear.z * self.twist_lead),
                               mat_to_quat(rot_y(pitch)))
            with self.lock:
                self.target = marker
            target = self.mode_target(marker)
            self.server.setPose(MARKER_NAME, marker)
            self.server.applyChanges()
        self.publish_goal(target)

        p_err = position_distance(self.ee_cmd, target)
        a_err = quaternion_angle(self.ee_cmd.orientation, target.orientation)
        reached = p_err < self.position_tolerance and a_err < self.orientation_tolerance
        # while dragging, keep easing toward the marker however close; after release, stop
        # within the tolerances
        idle = p_err < 0.0002 and a_err < 0.002
        now = self.get_clock().now()
        if not (idle or (reached and not dragging)):
            # plan the horizon from the anchor; send it unless no step is feasible (then the
            # running plan finishes, at rest)
            points = [(anchor_time, self.q_cmd, self.ee_cmd)]
            period = 1.0 / self.rate
            for _ in range(self.horizon):
                step = self.next_step(target, points[-1][1], points[-1][2])
                if step is None:
                    break
                q, pose = step
                if pose is None:
                    pose = self.forward_kinematics(q)
                    if pose is None:
                        break
                points.append((points[-1][0] + period, q, pose))
            if len(points) > 1:
                if anchor_time - clock < 0.005:
                    points = points[1:]   # already at the anchor
                self.send_plan(clock, points)
                # what the next cycle starts from, if it is on time
                self.q_cmd, self.ee_cmd = points[0][1], points[0][2]

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
                    f'from the target (out of reach, unreachable orientation, or blocked)')


def main():
    rclpy.init()
    node = ArmMarker()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node.loop.start()
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.kinematics_node.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
