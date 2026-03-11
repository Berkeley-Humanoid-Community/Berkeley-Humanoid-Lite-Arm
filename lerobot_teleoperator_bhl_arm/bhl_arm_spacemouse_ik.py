from __future__ import annotations

import time
from typing import Any

import mujoco
import mujoco.viewer
import numpy as np
from lerobot.processor import RobotAction
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from lerobot_robot_bhl_arm.bhl_arm import BHLArm

from .config_bhl_arm_spacemouse_ik import BHLArmSpaceMouseIKTeleopConfig
from .motion_retargeting import (
    BHLMotionRetargeting,
    IKSolveInfo,
    create_frame_visualization_xml,
    set_mocap_marker,
)
from .spacemouse import SpaceMouse, SpaceMouseState, find_spacemouse
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


class BHLArmSpaceMouseIKTeleop(Teleoperator):
    """Dual SpaceMouse teleop that solves dual-arm IK targets from device deflections."""

    config_class = BHLArmSpaceMouseIKTeleopConfig
    name = "bhl_arm_spacemouse_ik"

    def __init__(self, config: BHLArmSpaceMouseIKTeleopConfig | None = None):
        config = config or BHLArmSpaceMouseIKTeleopConfig()
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._retargeting: BHLMotionRetargeting | None = None
        self._viewer = None
        self._left_mouse: SpaceMouse | None = None
        self._right_mouse: SpaceMouse | None = None
        self._target_poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._current_joint_positions = np.asarray(config.initial_joint_positions, dtype=np.float32)
        self._last_log_time = 0.0
        self._last_update_time = 0.0
        self._last_reset_buttons = {"left": False, "right": False}

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
        print("SpaceMouse IK teleop controls:")
        for line in self.config.describe_controls():
            print(f"  {line}")
        if self._left_mouse is not None:
            print(f"  left device: {self._left_mouse.device.name} ({self._left_mouse.device.path})")
        if self._right_mouse is not None:
            print(f"  right device: {self._right_mouse.device.name} ({self._right_mouse.device.path})")

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate

        xml_path = self.config.robot_xml()
        if not xml_path.exists():
            raise FileNotFoundError(
                "MuJoCo XML not found for SpaceMouse IK teleop. "
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

        self._left_mouse = SpaceMouse(find_spacemouse(self.config.left_device_path, index=0))
        right_index = 1 if self.config.right_device_path is None else 0
        self._right_mouse = SpaceMouse(find_spacemouse(self.config.right_device_path, index=right_index))
        if self._left_mouse.device.path == self._right_mouse.device.path:
            raise RuntimeError("Left and right SpaceMouse resolved to the same hidraw device. Set explicit device paths.")

        self._last_update_time = time.perf_counter()
        self._last_reset_buttons = {"left": False, "right": False}
        self._is_connected = True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        if self._left_mouse is None or self._right_mouse is None:
            raise RuntimeError("SpaceMouse devices are not initialized.")

        now = time.perf_counter()
        dt = max(1e-3, now - self._last_update_time)
        self._last_update_time = now

        left_state = self._left_mouse.poll()
        right_state = self._right_mouse.poll()
        self._apply_spacemouse_input(
            body_name=self.config.left_body_name,
            state=left_state,
            dt=dt,
            translation_signs=np.asarray(self.config.left_translation_signs, dtype=np.float64),
            rotation_signs=np.asarray(self.config.left_rotation_signs, dtype=np.float64),
            button_key="left",
        )
        self._apply_spacemouse_input(
            body_name=self.config.right_body_name,
            state=right_state,
            dt=dt,
            translation_signs=np.asarray(self.config.right_translation_signs, dtype=np.float64),
            rotation_signs=np.asarray(self.config.right_rotation_signs, dtype=np.float64),
            button_key="right",
        )

        joint_positions, info = self._solve_targets()
        self._current_joint_positions = joint_positions
        self._update_viewer_markers()
        self._log_target_status(info, left_state, right_state)
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
            raise RuntimeError("Connect the robot before starting SpaceMouse IK teleop.")

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
            self._last_update_time = time.perf_counter()
            self.print_controls()

            try:
                while True:
                    cycle_start = time.perf_counter()
                    action = self.get_action()
                    robot.send_action(action)
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
        if self._left_mouse is not None:
            self._left_mouse.close()
            self._left_mouse = None
        if self._right_mouse is not None:
            self._right_mouse.close()
            self._right_mouse = None
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        self._retargeting = None
        self._model = None
        self._data = None
        self._target_poses.clear()
        self._last_reset_buttons = {"left": False, "right": False}
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

    def _apply_spacemouse_input(
        self,
        *,
        body_name: str,
        state: SpaceMouseState,
        dt: float,
        translation_signs: np.ndarray,
        rotation_signs: np.ndarray,
        button_key: str,
    ) -> None:
        if state.left_button_pressed and not self._last_reset_buttons[button_key]:
            if self._retargeting is None:
                raise RuntimeError("IK retargeting is not initialized.")
            self._target_poses[body_name] = self._retargeting.get_body_pose(body_name)
        self._last_reset_buttons[button_key] = state.left_button_pressed

        position, orientation = self._target_poses[body_name]
        translation_input = self._normalize_axes(state.translation) * translation_signs
        rotation_input = self._normalize_axes(state.rotation) * rotation_signs
        position_delta = translation_input * self.config.translation_speed * dt
        rotation_delta = rotation_input * self.config.rotation_speed * dt
        target_quat = quat_mul(_quat_from_rotvec(rotation_delta), orientation)
        self._target_poses[body_name] = (position + position_delta, target_quat)

    def _normalize_axes(self, axes: tuple[int, int, int]) -> np.ndarray:
        values = np.asarray(axes, dtype=np.float64)
        max_value = max(self.config.input_max_value, 1.0)
        normalized = np.clip(values / max_value, -1.0, 1.0)
        if self.config.input_deadzone > 0:
            normalized[np.abs(normalized) < self.config.input_deadzone] = 0.0
        return normalized

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

    def _log_target_status(self, info: IKSolveInfo, left_state: SpaceMouseState, right_state: SpaceMouseState) -> None:
        if self.config.log_interval_s <= 0:
            return
        now = time.monotonic()
        if now - self._last_log_time < self.config.log_interval_s:
            return

        left_pos, _ = self._target_poses[self.config.left_body_name]
        right_pos, _ = self._target_poses[self.config.right_body_name]
        print(
            f"L_target=({left_pos[0]:+.3f}, {left_pos[1]:+.3f}, {left_pos[2]:+.3f}) "
            f"L_input={left_state.translation}/{left_state.rotation} | "
            f"R_target=({right_pos[0]:+.3f}, {right_pos[1]:+.3f}, {right_pos[2]:+.3f}) "
            f"R_input={right_state.translation}/{right_state.rotation} | "
            f"IK err={info.error_norm:.5f}, iters={info.iterations}, converged={info.converged}"
        )
        self._last_log_time = now

    def _run_shutdown_damping_mode(self, robot: BHLArm) -> None:
        robot.enter_damping_mode(
            self.config.shutdown_stiffness,
            self.config.shutdown_damping,
        )
        print(
            "Damping mode active "
            f"(kp={self.config.shutdown_stiffness:.1f}, kd={self.config.shutdown_damping:.1f}). "
            "Press Ctrl+C again to exit."
        )

        try:
            while True:
                cycle_start = time.perf_counter()
                robot.hold_current_position()
                self._sleep_to_frequency(cycle_start)
        except KeyboardInterrupt:
            print("\nExit requested. Shutting down.")

    def _sleep_to_frequency(self, cycle_start: float) -> None:
        if self.config.control_frequency <= 0:
            return
        period = 1.0 / self.config.control_frequency
        remaining = period - (time.perf_counter() - cycle_start)
        if remaining > 0:
            time.sleep(remaining)
