import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Any

from lerobot.processor import RobotAction
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from lerobot_robot_bhl_arm.bhl_arm import BHLArm

from .config_bhl_arm_keyboard import BHLArmKeyboardTeleopConfig


@dataclass(frozen=True)
class KeypressResult:
    should_quit: bool = False
    action: dict[str, float] | None = None


class BHLArmKeyboardTeleop(Teleoperator):
    """LeRobot-compatible keyboard teleop for BHL wrist yaw sanity tests."""

    config_class = BHLArmKeyboardTeleopConfig
    name = "bhl_arm_keyboard"

    def __init__(self, config: BHLArmKeyboardTeleopConfig | None = None):
        config = config or BHLArmKeyboardTeleopConfig()
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._raw_terminal: _RawTerminal | None = None

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{joint_name}.delta": float for joint_name in self.config.controlled_joints}

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
        print("Keyboard teleop controls:")
        for line in self.config.describe_controls():
            print(f"  {line}")

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        if not sys.stdin.isatty():
            raise RuntimeError("Keyboard teleop requires a TTY stdin.")

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
        return result.action or {}

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        del feedback

    def run(self, robot: BHLArm) -> None:
        if not robot.is_connected:
            raise RuntimeError("Connect the robot before starting keyboard teleop.")

        should_disconnect = not self.is_connected
        if should_disconnect:
            self.connect()

        current_positions = self._initialize_targets(robot)
        self.print_controls()
        self._print_targets(robot, current_positions)

        try:
            try:
                self._run_teleop_loop(robot)
            except KeyboardInterrupt:
                print("\nTeleop interrupted. Entering damping mode.")

            self._run_shutdown_damping_mode(robot)
        finally:
            if should_disconnect:
                self.disconnect()

    @check_if_not_connected
    def disconnect(self) -> None:
        if self._raw_terminal is not None:
            self._raw_terminal.__exit__(None, None, None)
            self._raw_terminal = None
        self._is_connected = False

    def _initialize_targets(self, robot: BHLArm) -> list[float]:
        print("Soft-starting KP/KD to target values...")
        return robot.prepare_for_control(
            self.config.stiffness,
            self.config.damping,
            ramp_duration=self.config.soft_start_duration,
            frequency=self.config.control_frequency,
            show_progress=True,
            progress_desc="Soft start",
        ).tolist()

    def _run_teleop_loop(self, robot: BHLArm) -> None:
        while True:
            cycle_start = time.perf_counter()
            action = self.get_action()
            if action:
                robot.send_action(action)
                self._print_targets(robot, robot.measured_positions.tolist())
            else:
                robot.hold_current_position()
            self._sleep_to_frequency(cycle_start)

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

    def _read_keypress(self, timeout: float) -> KeypressResult:
        should_quit = False
        action: dict[str, float] = {}

        if select.select([sys.stdin], [], [], timeout)[0]:
            pressed_key = sys.stdin.read(1)
            if pressed_key == self.config.quit_key:
                should_quit = True
            elif pressed_key == self.config.left_positive_key:
                action["left_wrist_yaw.delta"] = self.config.position_step
            elif pressed_key == self.config.left_negative_key:
                action["left_wrist_yaw.delta"] = -self.config.position_step
            elif pressed_key == self.config.right_positive_key:
                action["right_wrist_yaw.delta"] = self.config.position_step
            elif pressed_key == self.config.right_negative_key:
                action["right_wrist_yaw.delta"] = -self.config.position_step

        return KeypressResult(should_quit=should_quit, action=action or None)

    def _print_targets(self, robot: BHLArm, positions: list[float]) -> None:
        left_index = robot.joint_names.index("left_wrist_yaw")
        right_index = robot.joint_names.index("right_wrist_yaw")
        print(
            "Targets | "
            f"left_wrist_yaw={positions[left_index]: .3f} rad | "
            f"right_wrist_yaw={positions[right_index]: .3f} rad"
        )

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
