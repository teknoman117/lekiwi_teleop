from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # The joystick (joy_node) and joy_teleop together: the base, and the arm through arm_marker
    # (planar mode). With the robot / workstation split, robot.launch.py runs joy_teleop and
    # workstation.launch.py (or robot.launch.py joy:=true) runs joy_node instead.
    declare_device = DeclareLaunchArgument(
        'device_id', default_value='0', description='Joystick device number (joy_node)')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': ParameterValue(LaunchConfiguration('device_id'), value_type=int),
            'deadzone': 0.166,
        }],
    )

    joy_teleop = Node(
        package='lekiwi_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        output='screen',
    )

    return LaunchDescription([declare_device, joy_node, joy_teleop])
