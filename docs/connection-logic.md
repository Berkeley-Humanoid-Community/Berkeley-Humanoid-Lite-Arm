# BHL Arm Lowlevel Connection Logic

This document describes the current runtime behavior of [bhl_arm.py](/home/tk/Desktop/Berkeley-Humanoid-Arm-Workspace/lerobot_robot_bhl_arm/lerobot_robot_bhl_arm/bhl_arm.py) and the intended usage pattern in [connect_bhl_arm.py](/home/tk/Desktop/Berkeley-Humanoid-Arm-Workspace/lerobot_robot_bhl_arm/examples/connect_bhl_arm.py).

## Summary

The Berkeley Humanoid Lite arm uses a Robstride MIT-frame style interface where actuator feedback is returned only after a command frame is written.

Because of that:

- `connect()` performs an initial passive refresh of measured joint state through `prepare_for_control(...)`.
- `get_observation()` does not talk to the low-level buses for joint state.
- Joint state is updated only inside `_write_joint_targets()`.
- Fresh joint state still depends on a write cycle, but `connect()` performs that first cycle for you.

## Connection Sequence

`BHLArm.connect()` currently performs these steps:

1. Resolve calibration from the LeRobot calibration file if available.
2. Call `_connect_buses(calibration)` to open each configured CAN channel and enable the configured actuators.
3. If calibration is missing and `calibrate=True`, run `calibrate()`, wait briefly, then reconnect using the saved calibration.
4. Connect any configured cameras.
5. Mark the robot as connected.
6. Call `prepare_for_control(...)` with the configured `stiffness`/`damping`.
7. Inside `prepare_for_control(...)`, set all gains to zero and send one command to refresh measured state.
8. Capture the current measured pose and hold that pose.
9. If `soft_start_enabled=True`, linearly apply the configured gains over `soft_start_duration`; otherwise apply the configured gains immediately and hold the current pose once.

## Read/Write Behavior

### `_write_joint_targets()`

This is the only path that both writes commands and updates cached joint telemetry.

For each actuator it:

1. Clips target positions to configured joint limits.
2. Sends `write_operation_frame(...)` with the target position and current `kP`/`kD`.
3. Immediately calls `read_operation_frame(...)`.
4. Stores the returned position, velocity, torque, and temperature into the robot's cached arrays.

This means the internal measured state is only as fresh as the most recent command write.

### `_read_joint_states()`

This helper still exists, but it is intended only for situations where a low-level read is known to be valid immediately after a write. It should not be used as a standalone polling mechanism before the first command.

### `get_observation()`

`get_observation()` only packages the currently cached joint state plus any camera frames.

It does:

- read cached `*.pos`, `*.vel`, `*.torque`, `*.temp`
- read camera frames if cameras are configured

It does **not** refresh joint data from the CAN bus. If you need fresh actuator telemetry, some write path such as `send_action(...)`, `hold_current_position()`, or `hold_joint_positions(...)` must run first.

## Why the Example Sends an Action Before Each Observation

In [connect_bhl_arm.py](/home/tk/Desktop/Berkeley-Humanoid-Arm-Workspace/lerobot_robot_bhl_arm/examples/connect_bhl_arm.py):

1. The script connects the robot.
2. It sets `kP = 0` and `kD = 0` for every joint with `configure_stiffness_and_damping(0.0, 0.0)`.
3. Before each observation, it calls `send_action({"positions": [0.0] * robot.num_joints})`.
4. It then calls `get_observation()`.

The reason for this pattern is:

- a command must be written to receive fresh actuator feedback
- zero gains minimize the chance that the arm actively drives toward the commanded target
- the subsequent `get_observation()` reads the cached response produced by that write

This is still a valid passive-inspection pattern. It is no longer required just to populate the initial measured state after `connect()`, but it is still required if the script wants each printed observation to reflect a fresh hardware response.

## Calibration Flow

`calibrate()` also follows the same write-then-observe rule:

- it sets all gains to zero
- repeatedly sends zero targets through `_write_joint_targets()`
- uses the returned measured positions to collect min/max ranges
- computes homing offsets and saves them to the LeRobot calibration file path

So calibration does not rely on a separate background state polling path either.

## Operational Implications

When writing new scripts against `BHLArm`, assume:

- `connect()` establishes communication, refreshes measured state once, and initializes target positions to the current pose.
- `send_action()` is the mechanism that refreshes joint telemetry.
- `get_observation()` is a packaging step, not a bus polling step.
- The robot accepts absolute position actions (`positions`, `*.pos`) and incremental position actions (`deltas`, `*.delta`).

## Recommended Usage Pattern

For scripts that want to inspect the arm without intentionally moving it:

1. `robot.connect(...)`
2. `robot.configure_stiffness_and_damping(0.0, 0.0)`
3. `robot.hold_current_position()`
4. `robot.get_observation()`

Repeat steps 3 and 4 if you want each observation to be backed by a fresh write/read cycle while still minimizing active motion.

## Reusable Startup/Shutdown Helpers

`BHLArm` now exposes a few reusable safety-oriented helpers for controllers:

- `prepare_for_control(stiffness, damping, ramp_duration=..., frequency=...)`
- `enter_damping_mode(stiffness=0.0, damping=10.0)`
- `hold_current_position()`

These helpers keep the controller-specific state machine outside the robot class, while moving the repeated low-level mechanics into the robot adapter:

- `prepare_for_control(...)` refreshes measured state at zero gains, captures the current pose, and then applies or ramps gains while holding that pose.
- `enter_damping_mode(...)` applies the requested damping-style gains and immediately holds the current pose.
- `hold_current_position()` is intended for repeated calls in a shutdown/damping loop because joint telemetry is refreshed only after a write.
