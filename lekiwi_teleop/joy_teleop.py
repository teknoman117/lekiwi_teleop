"""Joystick teleoperation of the LeKiwi: the omnidirectional base and the arm (planar mode).

Base (as teleop_twist_joy was set up): axis_linear_x / axis_linear_y / axis_angular_yaw, scaled
by scale_linear_x / scale_linear_y / scale_angular_yaw, to cmd_vel_topic (TwistStamped).

Arm, through arm_marker's velocity input (~/twist_cmd; arm_marker switches to planar mode):
  axis_arm_z  +1: up, -1: down                         (arm_linear_speed)
  axis_arm_x  -1: forward (+x), +1: back               (arm_linear_speed)
  button_pitch_away / button_pitch_toward: turn the gripper's tip away from / toward the robot
                                                       (arm_pitch_speed)
The directions are the robot's, not the gripper's.

Gripper: button_gripper_open / button_gripper_close send gripper_open / gripper_close (rad) to
gripper_action (control_msgs/ParallelGripperCommand), once per press.

Orbit: button_orbit_left / button_orbit_right move the base sideways around the arm's goal point
(arm_marker ~/goal), turning so that it keeps facing it. A rigid body turning at w about a point
p = (px, py) in its own frame moves its origin at v = w x (-p) = (w * py, -w * px), so the base is
sent linear (w * py, -w * px) and angular w, with w = orbit_speed / |p| (orbit_speed is the
speed of the base along the circle). With the goal ahead (px > 0), moving left (+y) needs w < 0.
The orbit adds to the sticks.
"""

import math
import time

import rclpy
from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformException, TransformListener


class JoyTeleop(Node):

    def __init__(self):
        super().__init__('joy_teleop')
        p = self.declare_parameter
        # base, as teleop_twist_joy was set up
        self.axis_linear_x = p('axis_linear_x', 1).value
        self.axis_linear_y = p('axis_linear_y', 0).value
        self.axis_angular_yaw = p('axis_angular_yaw', 3).value
        self.scale_linear_x = p('scale_linear_x', -0.25).value
        self.scale_linear_y = p('scale_linear_y', -0.25).value
        self.scale_angular_yaw = p('scale_angular_yaw', -1.0).value
        # arm
        self.axis_arm_x = p('axis_arm_x', 6).value
        self.axis_arm_z = p('axis_arm_z', 7).value
        self.button_pitch_away = p('button_pitch_away', 2).value
        self.button_pitch_toward = p('button_pitch_toward', 0).value
        self.arm_linear_speed = p('arm_linear_speed', 0.04).value          # m/s
        self.arm_pitch_speed = p('arm_pitch_speed', 0.5).value             # rad/s
        # gripper
        self.button_gripper_open = p('button_gripper_open', 6).value
        self.button_gripper_close = p('button_gripper_close', 7).value
        self.gripper_open = p('gripper_open', 1.2).value                   # rad
        self.gripper_close = p('gripper_close', -0.1).value                # rad (lower limit -0.1745)
        gripper_action = p('gripper_action', '/gripper_controller/gripper_cmd').value
        # orbit around the arm's goal
        self.button_orbit_left = p('button_orbit_left', 4).value
        self.button_orbit_right = p('button_orbit_right', 5).value
        self.orbit_speed = p('orbit_speed', 0.1).value                     # m/s along the circle
        self.min_orbit_radius = p('min_orbit_radius', 0.1).value           # m
        self.base_frame = p('base_frame', 'base_footprint').value
        self.rate = p('rate', 20.0).value                                  # Hz
        self.joy_timeout = p('joy_timeout', 0.5).value                     # s: stop when older
        cmd_vel_topic = p('cmd_vel_topic', '/omni_wheel_drive_controller/cmd_vel').value
        arm_twist_topic = p('arm_twist_topic', '/arm_marker/twist_cmd').value
        arm_goal_topic = p('arm_goal_topic', '/arm_marker/goal').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        self.arm_pub = self.create_publisher(TwistStamped, arm_twist_topic, 10)
        self.gripper_client = ActionClient(self, ParallelGripperCommand, gripper_action)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_subscription(PoseStamped, arm_goal_topic, self.on_goal, 10)

        self.joy = None
        self.joy_time = None
        self.goal = None
        self.base_moving = False
        self.arm_moving = False
        self.create_timer(1.0 / self.rate, self.on_timer)

    def on_joy(self, msg):
        previous = self.joy
        self.joy = msg
        self.joy_time = time.monotonic()
        # the gripper buttons act once per press
        for button, position, label in ((self.button_gripper_open, self.gripper_open, 'open'),
                                        (self.button_gripper_close, self.gripper_close, 'close')):
            was = previous is not None and 0 <= button < len(previous.buttons) and \
                previous.buttons[button] != 0
            if self.button(button) and not was:
                self.send_gripper(position, label)

    def send_gripper(self, position, label):
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn('gripper action server not available', throttle_duration_sec=2.0)
            return
        goal = ParallelGripperCommand.Goal()
        goal.command.name = ['gripper']
        goal.command.position = [float(position)]
        self.gripper_client.send_goal_async(goal)
        self.get_logger().info(f'gripper: {label}')

    def on_goal(self, msg):
        self.goal = msg

    def axis(self, index):
        axes = self.joy.axes
        return axes[index] if 0 <= index < len(axes) else 0.0

    def button(self, index):
        buttons = self.joy.buttons
        return 0 <= index < len(buttons) and buttons[index] != 0

    def goal_in_base(self):
        """The arm's goal point (x, y) in base_frame, or None."""
        if self.goal is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.goal.header.frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(f'no transform to the arm goal: {e}', throttle_duration_sec=2.0)
            return None
        q, tr = t.transform.rotation, t.transform.translation
        g = self.goal.pose.position
        # rotate by q, then translate
        x, y, z = g.x, g.y, g.z
        tx = 2.0 * (q.y * z - q.z * y)
        ty = 2.0 * (q.z * x - q.x * z)
        tz = 2.0 * (q.x * y - q.y * x)
        rx = x + q.w * tx + (q.y * tz - q.z * ty)
        ry = y + q.w * ty + (q.z * tx - q.x * tz)
        return rx + tr.x, ry + tr.y

    def stamped(self, frame):
        msg = TwistStamped()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def on_timer(self):
        fresh = self.joy is not None and time.monotonic() - self.joy_time < self.joy_timeout

        # base: sticks, plus the orbit
        vx = vy = wz = 0.0
        if fresh:
            vx = self.scale_linear_x * self.axis(self.axis_linear_x)
            vy = self.scale_linear_y * self.axis(self.axis_linear_y)
            wz = self.scale_angular_yaw * self.axis(self.axis_angular_yaw)
            direction = (1.0 if self.button(self.button_orbit_left) else 0.0) - \
                (1.0 if self.button(self.button_orbit_right) else 0.0)   # +1: left
            if direction != 0.0:
                p = self.goal_in_base()
                if p is not None:
                    radius = max(math.hypot(*p), self.min_orbit_radius)
                    w = -direction * self.orbit_speed / radius
                    vx += w * p[1]
                    vy += -w * p[0]
                    wz += w
        base_moving = any(abs(v) > 1e-6 for v in (vx, vy, wz))
        if base_moving or self.base_moving:   # one zero command after motion, then silence
            msg = self.stamped(self.base_frame)
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
            self.cmd_vel_pub.publish(msg)
        self.base_moving = base_moving

        # arm: planar velocity of the goal (the robot's directions)
        ax = az = wy = 0.0
        if fresh:
            ax = -self.arm_linear_speed * self.axis(self.axis_arm_x)
            az = self.arm_linear_speed * self.axis(self.axis_arm_z)
            # turning the tip toward the robot is +y (pointing down -> pointing back)
            wy = self.arm_pitch_speed * ((1.0 if self.button(self.button_pitch_toward) else 0.0)
                                         - (1.0 if self.button(self.button_pitch_away) else 0.0))
        arm_moving = any(abs(v) > 1e-6 for v in (ax, az, wy))
        if arm_moving or self.arm_moving:
            msg = self.stamped('so101_base_link')
            msg.twist.linear.x, msg.twist.linear.z, msg.twist.angular.y = ax, az, wy
            self.arm_pub.publish(msg)
        self.arm_moving = arm_moving


def main():
    rclpy.init()
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
