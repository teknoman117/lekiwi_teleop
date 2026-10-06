# lekiwi_teleop

Teleoperation of the SO-101 arm, on the LeKiwi or on its own.

## arm_marker

An RViz interactive marker that the arm follows. It uses move_group's IK (`/compute_ik`), so it
runs next to `lekiwi_moveit_config` or `so101_moveit_config` and is collision-checked against
the same model (with `lekiwi_moveit_config`: the LeKiwi base and the ground).

```bash
# with move_group running (e.g. ros2 launch lekiwi_moveit_config demo.launch.py)
ros2 launch lekiwi_teleop arm_marker.launch.py
```

In RViz, the `moveit.rviz` configs of both MoveIt packages have an InteractiveMarkers display on
`/arm_marker` ("Arm marker"). Drag the blue sphere, or the arrows and rings. Right-click:
"Reset to gripper".

### How it follows

Each cycle (`rate`, 20 Hz) while the marker is dragged, and after it is released until the arm
arrives or stops getting closer:

1. Step the commanded gripper pose toward the marker, at most `max_linear_speed` (0.08 m/s) and
   `max_angular_speed` (0.8 rad/s).
2. Solve IK for the full pose with `pose_group` (`arm`). The arm has 5 joints, so most
   orientations are unreachable; then solve for the position alone with `position_group`
   (`arm_teleop`: position-only, local search from the current joints).
3. If the step is blocked (the floor, the base, the edge of the reach), try it again without its
   x, y or z part: the gripper slides along the obstacle instead of stopping.
4. Reject a solution that moves any joint more than `max_joint_step` (0.15 rad) in one cycle (a
   jump to another IK branch), then stream the joint positions to
   `/arm_controller/joint_trajectory`, to be reached in `command_lead` (0.1 s).

After release it stops when the gripper is within `position_tolerance` / `orientation_tolerance`
of the marker, or when the distance has not shrunk for `settle_timeout` (0.5 s), and logs how far
from the marker it ended. While idle it publishes nothing, so planned motions from move_group are
never overridden, and the marker follows the gripper.

A message on the controller's topic replaces any running trajectory: do not drag the marker
while a planned motion executes.

### Parameters

`frame_id` (`so101_base_link`), `ee_frame` (`gripper_frame_link`), `pose_group`,
`position_group`, `rate`, `max_linear_speed`, `max_angular_speed`, `max_joint_step`,
`ik_timeout` (0.02 s), `command_lead`, `position_tolerance` (0.003 m), `orientation_tolerance`
(0.05 rad), `settle_timeout`, `drag_timeout` (1.0 s: a drag with no feedback for this long counts
as released), `marker_scale`, `trajectory_topic`.
