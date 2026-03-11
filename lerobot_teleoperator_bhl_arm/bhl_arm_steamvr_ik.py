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

from .config_bhl_arm_steamvr_ik import BHLArmSteamVRIKTeleopConfig
from .motion_retargeting import (
    BHLMotionRetargeting,
    IKSolveInfo,
    create_frame_visualization_xml,
    set_mocap_marker,
)
from .steamvr import SteamVRStateReceiver, SteamVRSnapshot, quat_inv, quat_mul


class BHLArmSteamVRIKTeleop(Teleoperator):
    """SteamVR teleop that solves dual-arm IK targets from controller pose deltas."""

    config_class = BHLArmSteamVRIKTeleopConfig
    name = "bhl_arm_steamvr_ik"

    def __init__(self, config: BHLArmSteamVRIKTeleopConfig | None = None):
        config = config or BHLArmSteamVRIKTeleopConfig()
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._retargeting: BHLMotionRetargeting | None = None
        self._viewer = None
        self._receiver: SteamVRStateReceiver | None = None
        self._target_poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._reference_pose_bases: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._reference_initialized = False
        self._current_joint_positions = np.asarray(config.initial_joint_positions, dtype=np.float32)
        self._last_log_time = 0.0
        self._warned_no_packet = False
        self._warned_stale_packet = False

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
        print(
            "SteamVR IK teleop active. "
            f"Listening on UDP {self.config.steamvr_host}:{self.config.steamvr_port}."
        )
        print("Ctrl+C once to enter damping mode. Ctrl+C again to exit.")

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate

        xml_path = self.config.robot_xml()
        if not xml_path.exists():
            raise FileNotFoundError(
                "MuJoCo XML not found for SteamVR IK teleop. "
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

        self._receiver = SteamVRStateReceiver(self.config.steamvr_host, self.config.steamvr_port)
        self._receiver.start()
        self._is_connected = True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        if self._receiver is None:
            raise RuntimeError("SteamVR receiver is not initialized.")

        snapshot = self._receiver.snapshot()
        self._handle_packet_state(snapshot)
        self._update_targets_from_snapshot(snapshot)
        joint_positions, info = self._solve_targets()
        self._current_joint_positions = joint_positions
        self._update_viewer_markers()
        self._log_target_status(info, snapshot)
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
            raise RuntimeError("Connect the robot before starting SteamVR IK teleop.")

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
        if self._receiver is not None:
            self._receiver.stop()
            self._receiver = None
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        self._retargeting = None
        self._model = None
        self._data = None
        self._target_poses.clear()
        self._reference_pose_bases.clear()
        self._reference_initialized = False
        self._warned_no_packet = False
        self._warned_stale_packet = False
        self._is_connected = False

    def _sync_sim_joint_positions(self, joint_positions: np.ndarray, *, reset_targets: bool) -> None:
        if self._retargeting is None:
            return
        self._retargeting.set_joint_positions(joint_positions)
        self._current_joint_positions = self._retargeting.get_joint_positions()
        if reset_targets or not self._target_poses:
            self._reset_targets_to_current_poses()
            self._reference_initialized = False
        self._update_viewer_markers()

    def _reset_targets_to_current_poses(self) -> None:
        if self._retargeting is None:
            return
        self._target_poses = {
            body_name: self._retargeting.get_body_pose(body_name)
            for body_name in self.config.body_names
        }
        self._reference_pose_bases.clear()

    def _update_targets_from_snapshot(self, snapshot: SteamVRSnapshot) -> None:
        if self._retargeting is None:
            raise RuntimeError("IK retargeting is not initialized.")

        if not self._reference_initialized:
            self._initialize_reference(snapshot)

        left_base_pos, left_base_quat = self._reference_pose_bases[self.config.left_body_name]
        right_base_pos, right_base_quat = self._reference_pose_bases[self.config.right_body_name]
        self._target_poses[self.config.left_body_name] = (
            left_base_pos + snapshot.left_delta,
            quat_mul(snapshot.left_rot_delta, left_base_quat),
        )
        self._target_poses[self.config.right_body_name] = (
            right_base_pos + snapshot.right_delta,
            quat_mul(snapshot.right_rot_delta, right_base_quat),
        )

    def _initialize_reference(self, snapshot: SteamVRSnapshot) -> None:
        if self._retargeting is None:
            raise RuntimeError("IK retargeting is not initialized.")

        left_pos, left_quat = self._retargeting.get_body_pose(self.config.left_body_name)
        right_pos, right_quat = self._retargeting.get_body_pose(self.config.right_body_name)
        self._reference_pose_bases = {
            self.config.left_body_name: (
                left_pos - snapshot.left_delta,
                quat_mul(quat_inv(snapshot.left_rot_delta), left_quat),
            ),
            self.config.right_body_name: (
                right_pos - snapshot.right_delta,
                quat_mul(quat_inv(snapshot.right_rot_delta), right_quat),
            ),
        }
        self._reference_initialized = True

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

    def _log_target_status(self, info: IKSolveInfo, snapshot: SteamVRSnapshot) -> None:
        if self.config.log_interval_s <= 0:
            return
        now = time.monotonic()
        if now - self._last_log_time < self.config.log_interval_s:
            return

        left_pos, left_quat = self._target_poses[self.config.left_body_name]
        right_pos, right_quat = self._target_poses[self.config.right_body_name]
        print(
            f"L_target=({left_pos[0]:+.3f}, {left_pos[1]:+.3f}, {left_pos[2]:+.3f}) "
            f"L_quat=({left_quat[0]:+.3f}, {left_quat[1]:+.3f}, {left_quat[2]:+.3f}, {left_quat[3]:+.3f}) | "
            f"R_target=({right_pos[0]:+.3f}, {right_pos[1]:+.3f}, {right_pos[2]:+.3f}) "
            f"R_quat=({right_quat[0]:+.3f}, {right_quat[1]:+.3f}, {right_quat[2]:+.3f}, {right_quat[3]:+.3f}) | "
            f"pkt_age={snapshot.packet_age_s:.3f}s | "
            f"IK err={info.error_norm:.5f}, iters={info.iterations}, converged={info.converged}"
        )
        self._last_log_time = now

    def _handle_packet_state(self, snapshot: SteamVRSnapshot) -> None:
        if not snapshot.has_received_packet:
            if not self._warned_no_packet:
                print(
                    "Waiting for SteamVR packets on "
                    f"{self.config.steamvr_host}:{self.config.steamvr_port}..."
                )
                self._warned_no_packet = True
            return

        self._warned_no_packet = False
        if snapshot.packet_age_s > self.config.max_packet_age_s:
            if not self._warned_stale_packet:
                print(f"SteamVR packet stream is stale ({snapshot.packet_age_s:.2f}s old). Holding last target.")
                self._warned_stale_packet = True
        else:
            self._warned_stale_packet = False

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
