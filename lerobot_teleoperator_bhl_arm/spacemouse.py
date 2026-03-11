from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path


SPACE_MOUSE_VENDOR_ID = 0x256F
REPORT_ID_TRANSLATION = 1
REPORT_ID_ROTATION = 2
REPORT_ID_BUTTONS = 3
READ_SIZE = 13


@dataclass(frozen=True)
class SpaceMouseDevice:
    path: str
    name: str
    interface_number: int | None


@dataclass
class SpaceMouseState:
    translation: tuple[int, int, int] = (0, 0, 0)
    rotation: tuple[int, int, int] = (0, 0, 0)
    buttons_raw: int = 0

    @property
    def left_button_pressed(self) -> bool:
        return bool(self.buttons_raw & 0x1)

    @property
    def right_button_pressed(self) -> bool:
        return bool(self.buttons_raw & 0x2)


def _decode_axis(data: bytes, offset: int) -> int:
    value = data[offset] | (data[offset + 1] << 8)
    return value - 0x10000 if value >= 0x8000 else value


def _interface_number(hidraw_dir: Path) -> int | None:
    try:
        resolved = (hidraw_dir / "device").resolve()
    except OSError:
        return None

    for parent in (resolved, *resolved.parents):
        interface_file = parent / "bInterfaceNumber"
        if not interface_file.exists():
            continue
        try:
            return int(interface_file.read_text().strip(), 16)
        except (OSError, ValueError):
            return None
    return None


def _device_from_sysfs(hidraw_dir: Path) -> SpaceMouseDevice | None:
    try:
        lines = (hidraw_dir / "device" / "uevent").read_text().splitlines()
    except OSError:
        return None

    properties: dict[str, str] = {}
    for line in lines:
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        properties[key] = value

    hid_id = properties.get("HID_ID")
    if hid_id is None:
        return None

    try:
        _, vendor_hex, _ = hid_id.split(":")
        vendor_id = int(vendor_hex, 16)
    except ValueError:
        return None

    name = properties.get("HID_NAME", "Unknown device")
    if vendor_id != SPACE_MOUSE_VENDOR_ID and "spacemouse" not in name.lower():
        return None

    return SpaceMouseDevice(
        path=f"/dev/{hidraw_dir.name}",
        name=name,
        interface_number=_interface_number(hidraw_dir),
    )


def list_spacemice() -> list[SpaceMouseDevice]:
    devices = [
        device
        for device in (_device_from_sysfs(path) for path in sorted(Path("/sys/class/hidraw").glob("hidraw*")))
        if device is not None
    ]
    devices.sort(key=lambda device: (device.interface_number is None, device.interface_number, device.path))
    return devices


def find_spacemouse(device_path: str | None = None, index: int = 0) -> SpaceMouseDevice:
    if device_path is not None:
        hidraw_dir = Path("/sys/class/hidraw") / Path(device_path).name
        device = _device_from_sysfs(hidraw_dir)
        if device is None:
            raise RuntimeError(f"No SpaceMouse found at {device_path}.")
        return device

    devices = list_spacemice()
    if not devices:
        raise RuntimeError("No 3Dconnexion SpaceMouse hidraw device found.")
    if index < 0 or index >= len(devices):
        raise RuntimeError(f"Requested SpaceMouse index {index}, but only found {len(devices)} device(s).")
    return devices[index]


class SpaceMouse:
    def __init__(self, device: SpaceMouseDevice):
        self.device = device
        self.state = SpaceMouseState()
        try:
            self._fd = os.open(device.path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            permissions = "unknown permissions"
            try:
                permissions = oct(os.stat(device.path).st_mode & 0o777)
            except OSError:
                pass
            raise RuntimeError(
                f"Failed to open {device.path} for {device.name} ({permissions}). "
                "Check the hidraw permissions."
            ) from exc

    def close(self) -> None:
        os.close(self._fd)

    def poll(self) -> SpaceMouseState:
        while True:
            try:
                data = os.read(self._fd, READ_SIZE)
            except BlockingIOError:
                return self.state
            except OSError as exc:
                raise RuntimeError(f"Failed to read from {self.device.path}.") from exc

            if not data:
                return self.state
            self._update_state(data)

    def _update_state(self, data: bytes) -> None:
        report_id = data[0]
        if report_id == REPORT_ID_TRANSLATION and len(data) >= 7:
            self.state.translation = (
                -_decode_axis(data, 3),
                -_decode_axis(data, 1),
                -_decode_axis(data, 5),
            )
        elif report_id == REPORT_ID_ROTATION and len(data) >= 7:
            self.state.rotation = (
                -_decode_axis(data, 3),
                -_decode_axis(data, 1),
                -_decode_axis(data, 5),
            )
        elif report_id == REPORT_ID_BUTTONS and len(data) >= 2:
            self.state.buttons_raw = int(data[1])


def format_spacemouse_state(state: SpaceMouseState) -> str:
    tx, ty, tz = state.translation
    rx, ry, rz = state.rotation
    return (
        f"T[x={tx:6d} y={ty:6d} z={tz:6d}]  "
        f"R[roll={rx:6d} pitch={ry:6d} yaw={rz:6d}]  "
        f"Buttons[left={int(state.left_button_pressed)} "
        f"right={int(state.right_button_pressed)} raw={state.buttons_raw}]"
    )
