# Berkeley Humanoid Lite Arm (BHL v1.5)

[![Python](https://img.shields.io/badge/python-3.11-blue.svg)](https://docs.python.org/3/whatsnew/3.11.html)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](https://opensource.org/license/mit)

<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/84104020-76b0-4cef-8ed8-7ea395ef2de8" alt="NAI-Pose-1" width="400px"></td>
    <td><img src="https://github.com/user-attachments/assets/4e47b86b-4fef-4f79-a4da-fdef86502ec9" alt="NAI-Pose-2" width="400px"></td>
 </tr>
</table>

We are building a new bimanual arm platform for the Berkeley Humanoid Lite project!

This version improves upon the original [BHL](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite) with a more robust hardware stack and refined mechanical design.

Key improvements include:

- **Actuator upgrade:** Transition from custom M6C12 / 5010 actuators to more reliable and accessible **Robstride actuators**.
- **Refined mechanical design:** Redesigned the CAD geometry with smoother contours resembling human arms for a more aesthetically pleasing form.
- **LeRobot integration:** Provides [LeRobot](https://github.com/huggingface/lerobot)-compatible robot and teleoperator plugins for real-robot control, calibration, and teleoperation workflows.

While making these upgrades, our goal remains the same: building open-source, accessible, and customizable humanoid robots for researchers, makers, and the community.

*Note: this repository is under active development. API-breaking changes are likely.*


## CAD Source

[Onshape project](https://cad.onshape.com/documents/a91acc2c2b9072230d69b3af/w/13818f4f8c3432771bd4146f/e/b03e950d8d77cb51d8732685?renderMode=0&uiState=69b248ebb7254faf3c62ff10)

![](https://github.com/user-attachments/assets/4cb33d44-de2a-4fcc-bb4b-6156c7f03db3)

*Note: we are still refining the actuator selection and the design of the head, so that part is not open-sourced yet.*


## Setup

Install dependencies:

```bash
uv sync
```

This package provides third-party LeRobot plugins. The robot and teleoperator
types become available after Python imports `lerobot_robot_bhl_arm` and
`lerobot_teleoperator_bhl_arm`.

The example scripts in this repository import those packages for you.

After those imports, the robot config is available in LeRobot as:

```text
berkeley_humanoid_lite_arm
```

And the different teleoperator configs are available as:

```text
bhl_arm_keyboard
bhl_arm_keyboard_ik
bhl_arm_steamvr_ik
bhl_arm_spacemouse_ik
```


## CAN Bring-Up

Bring up the default CAN interfaces:

```bash
source ./examples/canup.sh
```

The default mapping in this package is:

- `left arm -> can0`
- `right arm -> can1`


## Real-Robot Connection Check

To connect and print measured joint positions:

```bash
uv run ./examples/connect_bhl_arm.py --count 3 --period 0.5
```

For the full observation dictionary:

```bash
uv run ./examples/connect_bhl_arm.py --count 1 --full-observation
```


## Calibration (Real Robot)

Run:

```bash
uv run ./examples/connect_bhl_arm.py --calibrate --count 1
```

This will connect to the robot, run calibration if no saved LeRobot calibration exists yet, and write the calibration file under LeRobot's calibration directory:

```text
~/.cache/huggingface/lerobot/calibration/
```

The exact filename is resolved by LeRobot from the robot configuration, typically incorporating the robot `id` when one is set. If you want to inspect the resolved path directly in Python, print `robot.calibration_fpath` from a `BHLArm` instance.

During calibration, the robot temporarily sets `kP=0` and `kD=0` so the joints stay backdrivable while you manually move each joint through its full range of motion and try to reach both the lower and upper limits.

The program continuously records the measured joint positions returned by the actuators, tracks the minimum and maximum value reached for each joint, and compares those sampled extrema against the default joint limits in `BHLArmConfig`. From that difference, it computes a per-joint homing offset so the actuator-reported positions line up with the expected robot joint ranges.

For best results, make sure each joint reaches both ends of its intended range before stopping the calibration.

After calibration, stop the program by pressing `Ctrl + C`. The offset will be automatically calculated and stored.


## Teleoperation

### Manual Keyboard Teleop

This repository currently includes a keyboard teleop sanity check for:

- `left_wrist_yaw`
- `right_wrist_yaw`

Default keys:

- `q` / `a`: increase / decrease `left_wrist_yaw`
- `w` / `s`: increase / decrease `right_wrist_yaw`
- `x`: request exit

Run:

```bash
uv run ./examples/teleop_bhl_arm_keyboard.py \
  --stiffness 8 \
  --damping 1 \
  --soft-start-duration 2 \
  --step-deg 2
```

Behavior:

- On startup, the script ramps `kP/kD` while holding the current pose.
- Press `x` once to enter damping mode.
- Press `x` again, or `Ctrl+C` again, to disconnect and fully exit.

### LeRobot CLI Teleop

You can invoke the same teleoperator through the official LeRobot CLI by
preloading this repository's plugin packages:

```bash
uv run python -c 'import lerobot_robot_bhl_arm, lerobot_teleoperator_bhl_arm; from lerobot.scripts.lerobot_teleoperate import main; main()' \
  --robot.type=berkeley_humanoid_lite_arm \
  --robot.stiffness=8 \
  --robot.damping=1 \
  --robot.soft_start_enabled=true \
  --robot.soft_start_duration=2.0 \
  --robot.soft_start_show_progress=true \
  --teleop.type=bhl_arm_keyboard \
  --teleop.control_frequency=20 \
  --teleop.position_step=0.034906585
```

Notes:

- `--robot.soft_start_enabled=true` is recommended so the robot syncs to its current pose before keyboard deltas are applied while smoothly applying the configured gains.
- The keyboard teleoperator uses per-joint `*.delta` commands.
- `BHLArm` accepts both absolute commands (`positions`, `*.pos`) and incremental commands (`deltas`, `*.delta`).
- The preloaded CLI path above uses the standard LeRobot teleoperate lifecycle and does not implement the manual script's two-stage damping-mode exit loop.

### Keyboard IK Teleop

This repository also includes a second keyboard teleoperator that moves an end-effector target in Cartesian space and uses MuJoCo + Mink IK to solve joint positions for the real robot.

Default keys:

- Left arm:
- `w` / `s`: `+x / -x` forward/backward translation
- `a` / `d`: `+y / -y` left/right translation
- `r` / `f`: `+z / -z` up/down translation
- `q` / `e`: `+x` roll / `-x` roll
- `z` / `x`: `+z` yaw / `-z` yaw
- Right arm:
- `i` / `k`: `+x / -x` forward/backward translation
- `j` / `l`: `+y / -y` left/right translation
- `y` / `h`: `+z / -z` up/down translation
- `u` / `o`: `+x` roll / `-x` roll
- `n` / `m`: `+z` yaw / `-z` yaw
- `c`: reset both targets to the current measured end-effector poses
- `p`: request exit

Run the manual script with the MuJoCo viewer and frame markers:

```bash
uv run ./examples/teleop_bhl_arm_keyboard_ik.py \
  --stiffness 8 \
  --damping 1 \
  --soft-start-duration 2 \
  --ee-step 0.01 \
  --viewer
```

The viewer shows both the current end-effector frame and the commanded target frame for each arm. By default, the teleoperator looks for the BHL Arm MJCF at:

```text
./data/bhl_arm/mjcf/bhl_arm.xml
```

relative to this repository's parent workspace. If your MJCF lives elsewhere, pass `--xml-path /abs/path/to/bhl_arm.xml`.


### SteamVR IK Teleop

This repository also includes a SteamVR teleoperator that listens for controller pose deltas over UDP, rebases those deltas to the robot's current end-effector pose, and solves dual-arm IK with MuJoCo + Mink.

The expected controller payload per side is:

- `relative_location`: `[x, y, z]`
- `relative_orientation`: `[qw, qx, qy, qz]`

Run the manual real-robot teleop:

```bash
uv run ./examples/teleop_bhl_arm_steamvr_ik.py \
  --stiffness 8 \
  --damping 1 \
  --soft-start-duration 2 \
  --frequency 30 \
  --steamvr-host 0.0.0.0 \
  --steamvr-port 11005 \
  --viewer
```

This listens for SteamVR packets on UDP `0.0.0.0:11005` by default. The viewer shows current and target end-effector frames for both arms.


### Dual SpaceMouse IK Teleop

This repository also includes a dual-SpaceMouse teleoperator that uses one 3Dconnexion SpaceMouse per arm, integrates each device's 6-DoF deflection into end-effector pose targets, and solves dual-arm IK with MuJoCo + Mink.

On Linux, the matching `hidraw` devices must be readable by your user. A typical udev rule for the SpaceMouse Compact is:

```text
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="256f", ATTRS{idProduct}=="c635", MODE="0660", GROUP="plugdev", TAG+="uaccess"
```

Reload rules and replug the devices after adding the rule:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

To verify raw device input before starting teleop:

```bash
uv run ./examples/test_spacemouse.py --device /dev/hidraw1
uv run ./examples/test_spacemouse.py --device /dev/hidraw2
```

Run the manual real-robot teleop:

```bash
uv run ./examples/teleop_bhl_arm_spacemouse_ik.py \
  --stiffness 8 \
  --damping 1 \
  --soft-start-duration 2 \
  --frequency 30 \
  --left-device /dev/hidraw1 \
  --right-device /dev/hidraw2 \
  --translation-speed 0.12 \
  --rotation-speed 1.2 \
  --viewer
```

Behavior:

- The first SpaceMouse controls the left end effector.
- The second SpaceMouse controls the right end effector.
- The left button on either device resets that arm target to the current measured end-effector pose.
- `left_translation_signs`, `right_translation_signs`, `left_rotation_signs`, and `right_rotation_signs` in `BHLArmSpaceMouseIKTeleopConfig` can be adjusted if your physical device orientation does not match the robot frame convention.


## Debugging CAN Discovery

To scan the default CAN buses and list detected actuator IDs:

```bash
uv run ./examples/discover_bhl_arm_actuators.py
```

To limit the scanned ID range:

```bash
uv run ./examples/discover_bhl_arm_actuators.py --start-id 1 --end-id 50
```

## Safety Helpers

Reusable controller-facing safety helpers live on `BHLArm`:

- `prepare_for_control(...)`
- `enter_damping_mode(...)`
- `hold_current_position()`
- `hold_joint_positions(...)`

These helpers are intended for future controllers that need the same startup/shutdown mechanics without duplicating robot-side logic.


## Community

Consider joining our [Discord channel](https://discord.gg/yH8XZDMrwA).


## Acknowledgement

We would like to thank [DeepCybo](https://deepcybo.top/) and [Zhongguancun Academy (ZGCA)](https://www.bza.edu.cn/en/) for the collaboration and support.
