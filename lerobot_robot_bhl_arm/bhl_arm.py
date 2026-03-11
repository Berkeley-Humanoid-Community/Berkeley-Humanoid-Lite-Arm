import json
import logging
import time
from functools import cached_property
from pathlib import Path
from typing import Any

import numpy as np
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from robstride_dynamics import Motor, RobstrideBus
from tqdm import tqdm

from .config_bhl_arm import BHLArmConfig

logger = logging.getLogger(__name__)


class BHLArm(Robot):
    """Berkeley Humanoid Lite Arm following the LeRobot API."""

    config_class = BHLArmConfig
    name = "berkeley_humanoid_lite_arm"

    def __init__(self, config: BHLArmConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)

        self._joint_names = list(config.actuator_configs)
        self._joint_name_to_index = {name: index for index, name in enumerate(self._joint_names)}
        self._buses: dict[str, RobstrideBus] = {}
        self._actuator_buses: dict[str, RobstrideBus] = {}
        self._is_connected = False

        self._joint_measured_positions = np.zeros(self.num_joints, dtype=np.float32)
        self._joint_measured_velocities = np.zeros(self.num_joints, dtype=np.float32)
        self._joint_measured_torques = np.zeros(self.num_joints, dtype=np.float32)
        self._joint_measured_temperatures = np.zeros(self.num_joints, dtype=np.float32)
        self._joint_target_positions = np.zeros(self.num_joints, dtype=np.float32)
        self._joint_target_velocities = np.zeros(self.num_joints, dtype=np.float32)
        self._joint_kps = np.zeros(self.num_joints, dtype=np.float32)
        self._joint_kds = np.zeros(self.num_joints, dtype=np.float32)

    @property
    def joint_names(self) -> list[str]:
        return self._joint_names

    @property
    def num_joints(self) -> int:
        return len(self._joint_names)

    @property
    def measured_positions(self) -> np.ndarray:
        return self._joint_measured_positions.copy()

    @property
    def measured_velocities(self) -> np.ndarray:
        return self._joint_measured_velocities.copy()

    @property
    def measured_torques(self) -> np.ndarray:
        return self._joint_measured_torques.copy()

    @property
    def measured_temperatures(self) -> np.ndarray:
        return self._joint_measured_temperatures.copy()

    @property
    def joint_kps(self) -> np.ndarray:
        return self._joint_kps.copy()

    @property
    def joint_kds(self) -> np.ndarray:
        return self._joint_kds.copy()

    @property
    def _camera_features(self) -> dict[str, tuple[int, int, int]]:
        return {
            camera_name: (
                self.config.cameras[camera_name].height,
                self.config.cameras[camera_name].width,
                3,
            )
            for camera_name in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple[int, int, int]]:
        features: dict[str, type | tuple[int, int, int]] = {}
        for joint_name in self.joint_names:
            features[f"{joint_name}.pos"] = float
            features[f"{joint_name}.vel"] = float
            features[f"{joint_name}.torque"] = float
            features[f"{joint_name}.temp"] = float
        return {**features, **self._camera_features}

    @cached_property
    def action_features(self) -> dict[str, type]:
        features = {f"{joint_name}.pos": float for joint_name in self.joint_names}
        features.update({f"{joint_name}.delta": float for joint_name in self.joint_names})
        return features

    @property
    def is_connected(self) -> bool:
        return self._is_connected and all(camera.is_connected for camera in self.cameras.values())

    @property
    def is_calibrated(self) -> bool:
        required_fields = {"id", "direction", "homing_offset"}
        return (
            set(self.calibration) == set(self.joint_names)
            and all(required_fields.issubset(calibration) for calibration in self.calibration.values())
        )

    def _load_calibration(self, fpath: Path | None = None) -> None:
        calibration_path = self.calibration_fpath if fpath is None else fpath
        with open(calibration_path, encoding="utf-8") as file:
            self.calibration = json.load(file)

    def _save_calibration(self, fpath: Path | None = None) -> None:
        calibration_path = self.calibration_fpath if fpath is None else fpath
        calibration_path.parent.mkdir(parents=True, exist_ok=True)
        with open(calibration_path, "w", encoding="utf-8") as file:
            json.dump(self.calibration, file, indent=4)

    def _get_all_actuators_on_bus(self, bus_name: str) -> dict[str, Motor]:
        motors: dict[str, Motor] = {}
        for actuator_name, actuator_cfg in self.config.actuator_configs.items():
            if actuator_name.startswith(f"{bus_name}_"):
                motors[actuator_name] = Motor(id=actuator_cfg.id, model=actuator_cfg.model)
        return motors

    def _connect_buses(self, calibration: dict[str, dict[str, Any]] | None) -> None:
        self._disconnect_buses()
        for bus_name, channel in self.config.bus_configs.items():
            actuators = self._get_all_actuators_on_bus(bus_name)
            if not actuators:
                continue

            bus = RobstrideBus(
                channel=channel,
                motors=actuators,
                calibration=calibration,
            )
            bus.connect()
            self._buses[bus_name] = bus

            for actuator_name in actuators:
                bus.enable(actuator_name)
                self._actuator_buses[actuator_name] = bus

        missing = [joint_name for joint_name in self.joint_names if joint_name not in self._actuator_buses]
        if missing:
            self._disconnect_buses()
            raise ValueError(f"Actuators not registered to a bus: {missing}")

    def _disconnect_buses(self) -> None:
        for bus in self._buses.values():
            bus.disconnect()
        self._buses.clear()
        self._actuator_buses.clear()

    def _as_joint_vector(self, value: float | list[float] | tuple[float, ...] | np.ndarray) -> np.ndarray:
        if np.isscalar(value):
            return np.full(self.num_joints, float(value), dtype=np.float32)

        vector = np.asarray(value, dtype=np.float32)
        if vector.shape != (self.num_joints,):
            raise ValueError(f"Expected shape ({self.num_joints},), got {vector.shape}")
        return vector

    def _clip_joint_positions(self, positions: np.ndarray) -> np.ndarray:
        lower = np.array(
            [self.config.actuator_configs[joint_name].lower_limit for joint_name in self.joint_names],
            dtype=np.float32,
        )
        upper = np.array(
            [self.config.actuator_configs[joint_name].upper_limit for joint_name in self.joint_names],
            dtype=np.float32,
        )
        return np.clip(positions, lower, upper)

    def _read_joint_states(self) -> None:
        """Read state directly from the bus.

        This should only be used immediately after a command write on hardware that
        returns MIT-frame feedback in response to the write.
        """
        for actuator_name, bus in self._actuator_buses.items():
            index = self._joint_name_to_index[actuator_name]
            position, velocity, torque, temperature = bus.read_operation_frame(actuator_name)
            self._joint_measured_positions[index] = position
            self._joint_measured_velocities[index] = velocity
            self._joint_measured_torques[index] = torque
            self._joint_measured_temperatures[index] = temperature

    def _write_joint_targets(self, target_positions: np.ndarray) -> None:
        clipped_positions = self._clip_joint_positions(target_positions)
        self._joint_target_positions[:] = clipped_positions

        for actuator_name, bus in self._actuator_buses.items():
            index = self._joint_name_to_index[actuator_name]
            bus.write_operation_frame(
                actuator_name,
                position=float(self._joint_target_positions[index]),
                kp=float(self._joint_kps[index]),
                kd=float(self._joint_kds[index]),
            )
            position, velocity, torque, temperature = bus.read_operation_frame(actuator_name)
            self._joint_measured_positions[index] = position
            self._joint_measured_velocities[index] = velocity
            self._joint_measured_torques[index] = torque
            self._joint_measured_temperatures[index] = temperature

    def _sleep_to_frequency(self, cycle_start: float) -> None:
        if self.config.control_frequency <= 0:
            return
        period = 1.0 / self.config.control_frequency
        remaining = period - (time.perf_counter() - cycle_start)
        if remaining > 0:
            time.sleep(remaining)

    def _sleep_to_custom_frequency(self, cycle_start: float, frequency: float) -> None:
        if frequency <= 0:
            return
        period = 1.0 / frequency
        remaining = period - (time.perf_counter() - cycle_start)
        if remaining > 0:
            time.sleep(remaining)

    def _control_frequency_or_default(self, frequency: float | None, default: float = 10.0) -> float:
        if frequency is not None:
            return frequency
        if self.config.control_frequency > 0:
            return self.config.control_frequency
        return default

    def configure_stiffness_and_damping(
        self,
        stiffness: float | list[float] | tuple[float, ...] | np.ndarray,
        damping: float | list[float] | tuple[float, ...] | np.ndarray,
    ) -> None:
        self._joint_kps[:] = self._as_joint_vector(stiffness)
        self._joint_kds[:] = self._as_joint_vector(damping)

    @check_if_not_connected
    def hold_joint_positions(
        self,
        positions: float | list[float] | tuple[float, ...] | np.ndarray,
    ) -> np.ndarray:
        self.send_action({"positions": self._as_joint_vector(positions).tolist()})
        return self.measured_positions

    @check_if_not_connected
    def hold_current_position(self) -> np.ndarray:
        self.send_action({"positions": self.measured_positions.tolist()})
        return self.measured_positions

    @check_if_not_connected
    def prepare_for_control(
        self,
        stiffness: float | list[float] | tuple[float, ...] | np.ndarray,
        damping: float | list[float] | tuple[float, ...] | np.ndarray,
        *,
        ramp_duration: float = 0.0,
        frequency: float | None = None,
        show_progress: bool = False,
        progress_desc: str = "Soft start",
    ) -> np.ndarray:
        """Refresh measured state at zero gains, then ramp gains while holding the current pose."""
        self.configure_stiffness_and_damping(0.0, 0.0)
        self.send_action({"positions": [0.0] * self.num_joints})

        hold_positions = self.measured_positions
        target_kp = self._as_joint_vector(stiffness)
        target_kd = self._as_joint_vector(damping)

        if ramp_duration <= 0:
            self.configure_stiffness_and_damping(target_kp, target_kd)
            self.send_action({"positions": hold_positions.tolist()})
            return self.measured_positions

        control_frequency = self._control_frequency_or_default(frequency)
        ramp_steps = max(1, int(control_frequency * ramp_duration))
        iterator = range(ramp_steps)
        if show_progress:
            iterator = tqdm(iterator, desc=progress_desc, unit="step")

        for step_index in iterator:
            alpha = (step_index + 1) / ramp_steps
            self.configure_stiffness_and_damping(target_kp * alpha, target_kd * alpha)
            cycle_start = time.perf_counter()
            self.send_action({"positions": hold_positions.tolist()})
            self._sleep_to_custom_frequency(cycle_start, control_frequency)

        return self.measured_positions

    @check_if_not_connected
    def enter_damping_mode(
        self,
        stiffness: float | list[float] | tuple[float, ...] | np.ndarray = 0.0,
        damping: float | list[float] | tuple[float, ...] | np.ndarray = 10.0,
    ) -> np.ndarray:
        self.configure_stiffness_and_damping(stiffness, damping)
        return self.hold_current_position()

    def enable(self) -> None:
        for actuator_name, bus in self._actuator_buses.items():
            bus.enable(actuator_name)

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        calibration = self.calibration if self.is_calibrated else None
        self._connect_buses(calibration)

        if not self.is_calibrated and calibrate:
            logger.info("No LeRobot calibration file found for %s. Running calibration.", self)
            self.calibrate()
            time.sleep(1.0)
            self._connect_buses(self.calibration)

        try:
            for camera in self.cameras.values():
                camera.connect()

            self._is_connected = True
            self.prepare_for_control(
                self.config.stiffness,
                self.config.damping,
                ramp_duration=self.config.soft_start_duration if self.config.soft_start_enabled else 0.0,
                frequency=self.config.control_frequency,
                show_progress=self.config.soft_start_show_progress and self.config.soft_start_enabled,
            )
        except Exception:
            self.disconnect()
            raise

        logger.info("%s connected.", self)

    def calibrate(self) -> None:
        if not self._actuator_buses:
            raise RuntimeError("Connect the arm before running calibration.")

        print("Running calibration...")
        print("Move each joint through its full range of motion. Press Ctrl+C to stop sampling.")

        previous_kps = self._joint_kps.copy()
        previous_kds = self._joint_kds.copy()
        sampled_min = np.full(self.num_joints, np.inf, dtype=np.float64)
        sampled_max = np.full(self.num_joints, -np.inf, dtype=np.float64)
        actuator_directions = np.array(
            [self.config.actuator_configs[joint_name].direction for joint_name in self.joint_names],
            dtype=np.float64,
        )

        self.configure_stiffness_and_damping(0.0, 0.0)
        zero_targets = np.zeros(self.num_joints, dtype=np.float32)

        try:
            while True:
                cycle_start = time.perf_counter()
                self._write_joint_targets(zero_targets)
                measured_positions = self._joint_measured_positions.astype(np.float64, copy=True)
                measured_positions *= actuator_directions
                sampled_min = np.minimum(sampled_min, measured_positions)
                sampled_max = np.maximum(sampled_max, measured_positions)
                print(measured_positions)
                self._sleep_to_frequency(cycle_start)
        except KeyboardInterrupt:
            pass
        finally:
            self._joint_kps[:] = previous_kps
            self._joint_kds[:] = previous_kds

        lower_limits = np.array(
            [self.config.actuator_configs[joint_name].lower_limit for joint_name in self.joint_names],
            dtype=np.float64,
        )
        upper_limits = np.array(
            [self.config.actuator_configs[joint_name].upper_limit for joint_name in self.joint_names],
            dtype=np.float64,
        )
        lower_offset = sampled_min - lower_limits
        upper_offset = sampled_max - upper_limits
        homing_offsets = ((lower_offset + upper_offset) * 0.5) * actuator_directions

        calibration: dict[str, dict[str, int | float]] = {}
        for joint_name, homing_offset in zip(self.joint_names, homing_offsets, strict=True):
            actuator_cfg = self.config.actuator_configs[joint_name]
            calibration[joint_name] = {
                "id": actuator_cfg.id,
                "direction": actuator_cfg.direction,
                "homing_offset": float(homing_offset),
            }

        self.calibration = calibration
        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self.configure_stiffness_and_damping(self.config.stiffness, self.config.damping)

    def reset(self) -> RobotObservation:
        if self._actuator_buses:
            self._write_joint_targets(self._joint_measured_positions.copy())
        return self.get_observation()

    def step(self, action: RobotAction) -> RobotObservation:
        self.send_action(action)
        return self.get_observation()

    def _observation_dict(self) -> RobotObservation:
        observation: RobotObservation = {}
        for joint_name in self.joint_names:
            index = self._joint_name_to_index[joint_name]
            observation[f"{joint_name}.pos"] = float(self._joint_measured_positions[index])
            observation[f"{joint_name}.vel"] = float(self._joint_measured_velocities[index])
            observation[f"{joint_name}.torque"] = float(self._joint_measured_torques[index])
            observation[f"{joint_name}.temp"] = float(self._joint_measured_temperatures[index])
        return observation

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        observation = self._observation_dict()
        for camera_name, camera in self.cameras.items():
            observation[camera_name] = camera.read_latest()
        return observation

    def _action_to_joint_targets(self, action: RobotAction) -> np.ndarray:
        if "positions" in action:
            return self._clip_joint_positions(self._as_joint_vector(action["positions"]))

        if "deltas" in action:
            return self._clip_joint_positions(
                self._joint_target_positions.copy() + self._as_joint_vector(action["deltas"])
            )

        target_positions = self._joint_target_positions.copy()
        has_updates = False
        for joint_name in self.joint_names:
            index = self._joint_name_to_index[joint_name]
            position_key = f"{joint_name}.pos"
            delta_key = f"{joint_name}.delta"
            if position_key in action:
                target_positions[index] = float(action[position_key])
                has_updates = True
            if delta_key in action:
                target_positions[index] += float(action[delta_key])
                has_updates = True

        if not has_updates:
            return self._joint_target_positions.copy()

        return self._clip_joint_positions(target_positions)

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        target_positions = self._action_to_joint_targets(action)
        self._write_joint_targets(target_positions)
        return {
            f"{joint_name}.pos": float(self._joint_target_positions[self._joint_name_to_index[joint_name]])
            for joint_name in self.joint_names
        }

    def disconnect(self) -> None:
        if not self._buses and not any(camera.is_connected for camera in self.cameras.values()):
            self._is_connected = False
            return

        for camera in self.cameras.values():
            if camera.is_connected:
                camera.disconnect()

        if self.config.disable_on_disconnect:
            self._joint_kps[:] = 0.0
            self._joint_kds[:] = 0.0
            if self._actuator_buses:
                self._write_joint_targets(self._joint_measured_positions.copy())

        self._disconnect_buses()
        self._is_connected = False
        logger.info("%s disconnected.", self)
