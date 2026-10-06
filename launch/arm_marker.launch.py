from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # The RViz marker that the arm follows. Run it next to move_group (lekiwi_moveit_config or
    # so101_moveit_config), and add an InteractiveMarkers display on /arm_marker in RViz (both
    # moveit.rviz configs have one).
    arm_marker = Node(
        package='lekiwi_teleop',
        executable='arm_marker',
        name='arm_marker',
        output='screen',
    )

    return LaunchDescription([arm_marker])
