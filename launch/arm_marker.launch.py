from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # The RViz marker that the arm follows. Run it next to move_group (lekiwi_moveit_config or
    # so101_moveit_config), and add an InteractiveMarkers display on /arm_marker in RViz (both
    # moveit.rviz configs have one). The mode can also be changed from the marker's menu.
    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='free',
        choices=['free', 'claw', 'planar'],
        description='free: follow position (and orientation where reachable); claw: gripper '
                    'pointing down, x/y/z and yaw; planar: shoulder_pan and wrist_roll at 0, '
                    'x/z and pitch'
    )

    arm_marker = Node(
        package='lekiwi_teleop',
        executable='arm_marker',
        name='arm_marker',
        output='screen',
        parameters=[{'mode': LaunchConfiguration('mode')}],
    )

    return LaunchDescription([declare_mode, arm_marker])
