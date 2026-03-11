from __future__ import annotations

import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Any

import mujoco
import mujoco.viewer
import numpy as np
from lerobot.processor import RobotAction
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from lerobot_robot_bhl_arm.bhl_arm import BHLArm

from .config_bhl_arm_keyboard_ik import BHLArmKeyboardIKTeleopConfig
from .motion_retargeting import (
    BHLMotionRetargeting,
    IKSolveInfo,
    create_frame_visualization_xml,
    set_mocap_marker,
)
from .steamvr import quat_mul


def _quat_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    axis = rotvec / angle
    half_angle = 0.5 * angle
    sin_half = np.sin(half_angle)
    return np.array(
        [np.cos(half_angle), axis[0] * sin_half, axis[1] * sin_half, axis[2] * sin_half],
        dtype=np.float64,
    )


@dataclass(frozen=True)
class IKKeypressResult:
    should_quit: bool = False
    target_reset: bool = False
    body_name: str | None = None
    position_delta: np.ndarray | None = None
    rotation_delta: np.ndarray | None = None


class BHLArmKeyboardIKTeleop(Teleoperator):
    """Keyboard teleop that moves an end-effector target and solves IK to joint positions."""

    config_class = BHLArmKeyboardIKTeleopConfig
    name = "bhl_arm_keyboard_ik"

    def __init__(self, config: BHLArmKeyboardIKTeleopConfig | None = None):
        config = config or BHLArmKeyboardIKTeleopConfig()
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._raw_terminal: _RawTerminal | None = None
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._retargeting: BHLMotionRetargeting | None = None
        self._viewer = None
        self._target_poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._current_joint_positions = np.asarray(config.initial_joint_positions, dtype=np.float32)

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{joint_name}.pos": float for joint_name in self.config.controlled_joints}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def print_controls(self) -> None:
        print("Keyboard IK teleop controls:")
        for line in self.config.describe_controls():
            print(f"  {line}")

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        if not sys.stdin.isatty():
            raise RuntimeError("Keyboard IK teleop requires a TTY stdin.")

        xml_path = self.config.robot_xml()
        if not xml_path.exists():
            raise FileNotFoundError(
                "MuJoCo XML not found for IK teleop. "
                f"Set `robot_xml_path` to a valid BHL MJCF file. Tried: {xml_path}"
            )

        model_path = (
            create_frame_visualization_xml(xml_path, list(self.config.body_names))
            if self.config.launch_viewer
            else str(xml_path)
        )
        self._model = mujoco.MjModel.from_xml_path(model_path)
        self._data = mujoco.MjData(self._model)
        self._retargeting = BHLMotionRetargeting(
            self._model,
            self._data,
            list(self.config.joint_names),
            solver=self.config.ik.solver,
            damping=self.config.ik.damping,
            step_size=self.config.ik.step_size,
            max_joint_delta=self.config.ik.max_joint_delta,
            position_weight=self.config.ik.position_weight,
            orientation_weight=self.config.ik.orientation_weight,
        )
        self._sync_sim_joint_positions(self._current_joint_positions, reset_targets=True)

        if self.config.launch_viewer:
            self._viewer = mujoco.viewer.launch_passive(self._model, self._data)

        self._raw_terminal = _RawTerminal(sys.stdin.fileno())
        self._raw_terminal.__enter__()
        self._is_connected = True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        result = self._read_keypress(timeout=self._keypress_timeout())
        if result.should_quit:
            raise KeyboardInterrupt

        if result.target_reset:
            self._reset_targets_to_current_poses()
            self._update_viewer_markers()
            self._print_targets()
            return {}

        if result.body_name is None:
            self._update_viewer_markers()
            return {}

        self._apply_target_delta(
            result.body_name,
            position_delta=result.position_delta,
            rotation_delta=result.rotation_delta,
        )
        joint_positions, info = self._solve_targets()
        self._current_joint_positions = joint_positions
        self._update_viewer_markers()
        self._log_target_status(info)
        return {
            f"{joint_name}.pos": float(joint_positions[index])
            for index, joint_name in enumerate(self.config.joint_names)
        }

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        positions = feedback.get("positions")
        if positions is not None:
            self._sync_sim_joint_positions(np.asarray(positions, dtype=np.float32), reset_targets=False)

    def run(self, robot: BHLArm) -> None:
        if not robot.is_connected:
            raise RuntimeError("Connect the robot before starting keyboard IK teleop.")

        should_disconnect = not self.is_connected
        if should_disconnect:
            self.connect()

        try:
            current_positions = robot.prepare_for_control(
                self.config.stiffness,
                self.config.damping,
                ramp_duration=self.config.soft_start_duration,
                frequency=self.config.control_frequency,
                show_progress=True,
                progress_desc="Soft start",
            )
            self._sync_sim_joint_positions(current_positions, reset_targets=True)
            self.print_controls()
            self._print_targets()
            command_positions = self._current_joint_positions.copy()

            try:
                while True:
                    cycle_start = time.perf_counter()
                    action = self.get_action()
                    if action:
                        command_positions = np.array(
                            [action[f"{joint_name}.pos"] for joint_name in self.config.joint_names],
                            dtype=np.float32,
                        )
                        robot.send_action(action)
                        self.send_feedback({"positions": robot.measured_positions.tolist()})
                    else:
                        robot.send_action({"positions": command_positions.tolist()})
                        self.send_feedback({"positions": robot.measured_positions.tolist()})
                    self._sleep_to_frequency(cycle_start)
            except KeyboardInterrupt:
                print("\nTeleop interrupted. Entering damping mode.")

            self._run_shutdown_damping_mode(robot)
        finally:
            if should_disconnect:
                self.disconnect()

    @check_if_not_connected
    def disconnect(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        if self._raw_terminal is not None:
            self._raw_terminal.__exit__(None, None, None)
            self._raw_terminal = None
        self._retargeting = None
        self._model = None
        self._data = None
        self._target_poses.clear()
        self._is_connected = False

    def _sync_sim_joint_positions(self, joint_positions: np.ndarray, *, reset_targets: bool) -> None:
        if self._retargeting is None:
            return
        self._retargeting.set_joint_positions(joint_positions)
        self._current_joint_positions = self._retargeting.get_joint_positions()
        if reset_targets or not self._target_poses:
            self._reset_targets_to_current_poses()
        self._update_viewer_markers()

    def _reset_targets_to_current_poses(self) -> None:
        if self._retargeting is None:
            return
        self._target_poses = {
            body_name: self._retargeting.get_body_pose(body_name)
            for body_name in self.config.body_names
        }

    def _apply_target_delta(
        self,
        body_name: str,
        *,
        position_delta: np.ndarray | None = None,
        rotation_delta: np.ndarray | None = None,
    ) -> None:
        target_pos, target_quat = self._target_poses[body_name]
        if position_delta is not None:
            target_pos = target_pos + position_delta
        if rotation_delta is not None:
            target_quat = quat_mul(_quat_from_rotvec(rotation_delta), target_quat)
        self._target_poses[body_name] = (target_pos, target_quat)

    def _solve_targets(self) -> tuple[np.ndarray, IKSolveInfo]:
        if self._retargeting is None:
            raise RuntimeError("IK retargeting is not initialized.")
        return self._retargeting.solve_ik(
            self._target_poses,
            max_iter=self.config.ik.max_iterations,
            error_tolerance=self.config.ik.error_tolerance,
        )

    def _update_viewer_markers(self) -> None:
        if self._viewer is None or self._retargeting is None or self._model is None or self._data is None:
            return

        for body_name in self.config.body_names:
            current_pos, current_quat = self._retargeting.get_body_pose(body_name)
            set_mocap_marker(self._model, self._data, f"current_{body_name}_frame", current_pos, current_quat)
            target_pos, target_quat = self._target_poses[body_name]
            set_mocap_marker(self._model, self._data, f"target_{body_name}_frame", target_pos, target_quat)
        self._viewer.sync()

    def _log_target_status(self, info: IKSolveInfo) -> None:
        left_pos, left_quat = self._target_poses[self.config.left_body_name]
        right_pos, right_quat = self._target_poses[self.config.right_body_name]
        print(
            f"L pos=({left_pos[0]:+.3f}, {left_pos[1]:+.3f}, {left_pos[2]:+.3f}) "
            f"quat=({left_quat[0]:+.3f}, {left_quat[1]:+.3f}, {left_quat[2]:+.3f}, {left_quat[3]:+.3f}) | "
            f"R pos=({right_pos[0]:+.3f}, {right_pos[1]:+.3f}, {right_pos[2]:+.3f}) "
            f"quat=({right_quat[0]:+.3f}, {right_quat[1]:+.3f}, {right_quat[2]:+.3f}, {right_quat[3]:+.3f}) "
            f"| IK err={info.error_norm:.5f}, iters={info.iterations}, converged={info.converged}"
        )

    def _print_targets(self) -> None:
        left_pos, left_quat = self._target_poses[self.config.left_body_name]
        right_pos, right_quat = self._target_poses[self.config.right_body_name]
        print(
            f"L pos=({left_pos[0]:+.3f}, {left_pos[1]:+.3f}, {left_pos[2]:+.3f}) "
            f"quat=({left_quat[0]:+.3f}, {left_quat[1]:+.3f}, {left_quat[2]:+.3f}, {left_quat[3]:+.3f}) | "
            f"R pos=({right_pos[0]:+.3f}, {right_pos[1]:+.3f}, {right_pos[2]:+.3f}) "
            f"quat=({right_quat[0]:+.3f}, {right_quat[1]:+.3f}, {right_quat[2]:+.3f}, {right_quat[3]:+.3f})"
        )

    def _run_shutdown_damping_mode(self, robot: BHLArm) -> None:
        robot.enter_damping_mode(
            self.config.shutdown_stiffness,
            self.config.shutdown_damping,
        )
        print(
            "Damping mode active "
            f"(kp={self.config.shutdown_stiffness:.1f}, kd={self.config.shutdown_damping:.1f}). "
            f"Press {self.config.quit_key} or Ctrl+C again to exit."
        )

        try:
            while True:
                cycle_start = time.perf_counter()
                robot.hold_current_position()
                if self._should_exit_damping_mode():
                    print("\nExit requested. Shutting down.")
                    return
                self._sleep_to_frequency(cycle_start)
        except KeyboardInterrupt:
            print("\nExit requested. Shutting down.")

    def _read_keypress(self, timeout: float) -> IKKeypressResult:
        if not select.select([sys.stdin], [], [], timeout)[0]:
            return IKKeypressResult()

        pressed_key = sys.stdin.read(1)
        if pressed_key == self.config.quit_key:
            return IKKeypressResult(should_quit=True)
        if pressed_key == self.config.reset_target_key:
            return IKKeypressResult(target_reset=True)
        keymap = {
            self.config.left_forward_key: IKKeypressResult(
                body_name=self.config.left_body_name,
                position_delta=np.array([self.config.ee_position_step, 0.0, 0.0], dtype=np.float64),
            ),
            self.config.left_backward_key: IKKeypressResult(
                body_name=self.config.left_body_name,
                position_delta=np.array([-self.config.ee_position_step, 0.0, 0.0], dtype=np.float64),
            ),
            self.config.left_left_key: IKKeypressResult(
                body_name=self.config.left_body_name,
                position_delta=np.array([0.0, self.config.ee_position_step, 0.0], dtype=np.float64),
            ),
            self.config.left_right_key: IKKeypressResult(
                body_name=self.config.left_body_name,
                position_delta=np.array([0.0, -self.config.ee_position_step, 0.0], dtype=np.float64),
            ),
            self.config.left_up_key: IKKeypressResult(
                body_name=self.config.left_body_name,
                position_delta=np.array([0.0, 0.0, self.config.ee_position_step], dtype=np.float64),
            ),
            self.config.left_down_key: IKKeypressResult(
                body_name=self.config.left_body_name,
                position_delta=np.array([0.0, 0.0, -self.config.ee_position_step], dtype=np.float64),
            ),
            self.config.left_roll_positive_key: IKKeypressResult(
                body_name=self.config.left_body_name,
                rotation_delta=np.array([self.config.ee_rotation_step, 0.0, 0.0], dtype=np.float64),
            ),
            self.config.left_roll_negative_key: IKKeypressResult(
                body_name=self.config.left_body_name,
                rotation_delta=np.array([-self.config.ee_rotation_step, 0.0, 0.0], dtype=np.float64),
            ),
            self.config.left_yaw_positive_key: IKKeypressResult(
                body_name=self.config.left_body_name,
                rotation_delta=np.array([0.0, 0.0, self.config.ee_rotation_step], dtype=np.float64),
            ),
            self.config.left_yaw_negative_key: IKKeypressResult(
                body_name=self.config.left_body_name,
                rotation_delta=np.array([0.0, 0.0, -self.config.ee_rotation_step], dtype=np.float64),
            ),
            self.config.right_forward_key: IKKeypressResult(
                body_name=self.config.right_body_name,
                position_delta=np.array([self.config.ee_position_step, 0.0, 0.0], dtype=np.float64),
            ),
            self.config.right_backward_key: IKKeypressResult(
                body_name=self.config.right_body_name,
                position_delta=np.array([-self.config.ee_position_step, 0.0, 0.0], dtype=np.float64),
            ),
            self.config.right_left_key: IKKeypressResult(
                body_name=self.config.right_body_name,
                position_delta=np.array([0.0, self.config.ee_position_step, 0.0], dtype=np.float64),
            ),
            self.config.right_right_key: IKKeypressResult(
                body_name=self.config.right_body_name,
                position_delta=np.array([0.0, -self.config.ee_position_step, 0.0], dtype=np.float64),
            ),
            self.config.right_up_key: IKKeypressResult(
                body_name=self.config.right_body_name,
                position_delta=np.array([0.0, 0.0, self.config.ee_position_step], dtype=np.float64),
            ),
            self.config.right_down_key: IKKeypressResult(
                body_name=self.config.right_body_name,
                position_delta=np.array([0.0, 0.0, -self.config.ee_position_step], dtype=np.float64),
            ),
            self.config.right_roll_positive_key: IKKeypressResult(
                body_name=self.config.right_body_name,
                rotation_delta=np.array([self.config.ee_rotation_step, 0.0, 0.0], dtype=np.float64),
            ),
            self.config.right_roll_negative_key: IKKeypressResult(
                body_name=self.config.right_body_name,
                rotation_delta=np.array([-self.config.ee_rotation_step, 0.0, 0.0], dtype=np.float64),
            ),
            self.config.right_yaw_positive_key: IKKeypressResult(
                body_name=self.config.right_body_name,
                rotation_delta=np.array([0.0, 0.0, self.config.ee_rotation_step], dtype=np.float64),
            ),
            self.config.right_yaw_negative_key: IKKeypressResult(
                body_name=self.config.right_body_name,
                rotation_delta=np.array([0.0, 0.0, -self.config.ee_rotation_step], dtype=np.float64),
            ),
        }
        if pressed_key in keymap:
            return keymap[pressed_key]
        return IKKeypressResult()

    def _sleep_to_frequency(self, cycle_start: float) -> None:
        if self.config.control_frequency <= 0:
            return
        period = 1.0 / self.config.control_frequency
        remaining = period - (time.perf_counter() - cycle_start)
        if remaining > 0:
            time.sleep(remaining)

    def _keypress_timeout(self) -> float:
        if self.config.control_frequency <= 0:
            return 0.1
        return 1.0 / self.config.control_frequency

    def _should_exit_damping_mode(self) -> bool:
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1) == self.config.quit_key
        return False


class _RawTerminal:
    def __init__(self, fd: int):
        self.fd = fd
        self._previous_settings: list[int] | None = None

    def __enter__(self) -> None:
        self._previous_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return None

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if self._previous_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._previous_settings)
