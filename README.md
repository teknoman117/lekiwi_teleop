# lekiwi_teleop

Teleoperation of the SO-101 arm, on the LeKiwi or on its own.

## arm_marker

An RViz interactive marker that the arm follows. It uses move_group's IK (`/compute_ik`), so it
runs next to `lekiwi_moveit_config` or `so101_moveit_config` and is collision-checked against
the same model (with `lekiwi_moveit_config`: the LeKiwi base and the ground). The point that
follows the marker is `gripper_tip_link`, at the ends of the jaws.

```bash
# with move_group running (e.g. ros2 launch lekiwi_moveit_config demo.launch.py)
ros2 launch lekiwi_teleop arm_marker.launch.py mode:=claw
# or, on the robot: ros2 launch lekiwi_bringup robot.launch.py marker_mode:=claw
```

In RViz, the `moveit.rviz` configs of both MoveIt packages have an InteractiveMarkers display on
`/arm_marker` ("Arm marker"). Right-click the marker for "Reset to gripper" and "Mode".

### Modes

| Mode | Marker | Arm |
|---|---|---|
| `free` | moves in x/y/z, turns about every axis | fingertips follow the position; the orientation as far as 5 joints allow |
| `claw` | moves in x/y/z, turns about the vertical | gripper pointing straight down at the marker; turning the marker turns the gripper (wrist_roll, with shoulder_pan) |
| `planar` | moves in x/z, turns about y | `shoulder_pan` held at 0 and `wrist_roll` at `planar_roll` (-90 deg): the fingertips stay in the vertical plane in front of the arm, at the marker's x/z, pitched like the marker. Sideways motion is for the base |

Switching to `claw` or `planar` first moves the arm into the mode with a planned,
collision-checked motion (move_group, at `transition_scaling` speed). For `claw`, the fingertips
point down at their current x/y, as high as possible, else at `claw_ready`.

The gripper can point straight down only fairly low in front of the arm, because `wrist_flex`
turns at most 95 degrees: measured, with the fingertips at y = 0 in `so101_base_link`, up to
z = 6 cm at x = 15 cm, 8 cm at 20 cm, 6 cm at 25 cm, 2 cm at 30 cm, and nowhere at 35 cm. In
claw mode the marker can go anywhere, and the arm stops at that envelope.

### How it follows

Each cycle (`rate`, 20 Hz) while the marker is dragged, and after it is released until the arm
arrives or stops getting closer:

1. Step the commanded fingertip pose toward the mode's target (the marker projected onto the
   mode): `tracking_gain / rate` (6/s / 20 Hz = 30 %) of the remaining distance, so the arm eases
   into each marker update, at most `max_linear_speed` (0.08 m/s) and `max_angular_speed`
   (0.8 rad/s).
2. Solve IK. `free`: the full pose with `pose_group` (`arm`), else the position alone with
   `position_group` (`arm_teleop`, local search from the current joints). `claw` and `planar`:
   the full pose with `teleop_pose_group` (`arm_teleop_pose`, local); those modes only request
   orientations the arm can reach.
3. If the step is blocked (the floor, the base, the edge of the reach), try it again with the
   orientation unchanged, then without its x, y or z part: the gripper slides along the obstacle.
4. Reject a solution that moves any joint more than `max_joint_step` (0.15 rad) in one cycle (a
   jump to another IK branch) or, in `planar`, that moves `shoulder_pan` or `wrist_roll` more
   than `lock_tolerance` from 0 and `planar_roll`.

Each cycle plans `horizon` (6) such steps ahead, 0.3 s at 20 Hz, all IK-solved and
collision-checked, and sends them to `/arm_controller/joint_trajectory` as one trajectory that
ends at rest, with velocities through the points. The next cycle replaces it, starting from the
point the controller is then heading to. So the arm only follows the first step or two of each
plan, but a late cycle does not stop it: Python cannot hold a steady 20 Hz (stalls of 100-200 ms
were measured), and with one point per cycle the arm stopped and started. If the node stops, the
arm comes to rest at the end of the last plan. `arm_controller` has
`interpolate_from_desired_state: true` (`so101_description/config/so101_controllers.yaml`), so
each trajectory continues from the last commanded state, not the measured one, which lags on
real servos.

The loop runs on its own thread at a fixed rate, and calls IK and FK on a node of its own: as an
executor timer, rclpy started it late while busy with other callbacks.

After release it stops when the fingertips are within `position_tolerance` /
`orientation_tolerance` of the target, or when the distance has not shrunk for
`settle_timeout` (0.5 s), and logs how far from the target it ended. While idle it publishes
nothing, so planned motions from move_group are never overridden, and the marker follows the
gripper.

A message on the controller's topic replaces any running trajectory: do not drag the marker
while a planned motion executes.

### Parameters

`mode` (`free`), `frame_id` (`so101_base_link`), `tip_frame` (`gripper_tip_link`), `ik_link`
(`gripper_frame_link`, the IK groups' tip), `pose_group`, `position_group`, `teleop_pose_group`,
`rate`, `max_linear_speed`, `max_angular_speed`, `tracking_gain` (6/s), `max_joint_step`,
`planar_roll` (-pi/2),
`lock_tolerance` (0.02 rad),
`ik_timeout` (0.02 s), `horizon` (6 steps), `position_tolerance` (0.003 m), `orientation_tolerance`
(0.05 rad), `settle_timeout`, `drag_timeout` (1.0 s: a drag with no feedback for this long counts
as released), `claw_search_step` (0.02 m), `claw_ready` (`[0.20, 0.0, 0.04]`),
`transition_scaling` (0.3), `twist_timeout` (0.25 s), `twist_lead` (0.15 s), `marker_scale`,
`trajectory_topic`, `controller` (`arm_controller`:
nothing moves until it is active) and `controller_manager`.

With `--log-level arm_marker:=debug`, every cycle without a feasible step logs why (the IK error
codes, or the joint jump that was rejected).

## joy_teleop

A joystick for the base and the arm together. Run it where the joystick is plugged in, on the
same `ROS_DOMAIN_ID` as the robot (which runs `robot.launch.py`, with `arm_marker`):

```bash
ros2 launch lekiwi_teleop joy_teleop.launch.py        # joy_node + joy_teleop
```

| Input | Does |
|---|---|
| axis 1 / axis 0 | base forward/back, left/right (`scale_linear_x` / `scale_linear_y`, -0.25 m/s) |
| axis 3 | base turn (`scale_angular_yaw`, -1.0 rad/s) |
| axis 7 | arm goal up (+1) / down (-1) (`arm_linear_speed`, 0.04 m/s) |
| axis 6 | arm goal forward (-1) / back (+1) |
| button 0 / button 2 | gripper tip turns toward / away from the robot (`arm_pitch_speed`, 0.5 rad/s) |
| button 4 / button 5 | base moves left / right around the arm's goal, facing it (`orbit_speed`, 0.1 m/s) |
| button 6 / button 7 | gripper open (`gripper_open`, 1.2 rad) / close (`gripper_close`, -0.1 rad), once per press |

The base mapping is the `teleop_twist_joy` setup it replaces (no enable button); commands go to
`/omni_wheel_drive_controller/cmd_vel` (TwistStamped), one zero command when the sticks are
released, then nothing. The arm directions are the robot's, not the gripper's.

The arm is driven through `arm_marker`'s velocity input (`/arm_marker/twist_cmd`), so it gets
the same IK, collision checking, clamping and smooth streaming as the marker. Joystick input
switches `arm_marker` to planar mode (`shoulder_pan` 0, `wrist_roll` -90 deg) first, with a
planned move. The RViz marker shows the joystick's goal.

**Orbit.** To circle the goal point p = (px, py) (from `/arm_marker/goal`, in `base_footprint`)
while facing it, the base turns at w and moves its centre at v = w x (-p) = (w py, -w px), the
velocity of a rigid body turning about p. joy_teleop sends linear (w py, -w px) and angular w,
with w = `orbit_speed` / |p| (w < 0 to go left with the goal ahead). The orbit adds to the
sticks. Checked in mock: integrating 2 s of the commands (0.19 m sideways, 26 deg), the goal
moved 1.1 mm.

All axis and button numbers are parameters (`axis_linear_x`, `axis_linear_y`,
`axis_angular_yaw`, `axis_arm_x`, `axis_arm_z`, `button_pitch_away`, `button_pitch_toward`,
`button_orbit_left`, `button_orbit_right`, `button_gripper_open`, `button_gripper_close`), as are the speeds, `min_orbit_radius` (0.1 m),
`rate` (20 Hz) and `joy_timeout` (0.5 s: no commands from older joystick messages).
