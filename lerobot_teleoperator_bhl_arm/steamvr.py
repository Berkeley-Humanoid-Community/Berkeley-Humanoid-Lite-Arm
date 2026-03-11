from __future__ import annotations

import pickle
import threading
import time
from dataclasses import dataclass

import numpy as np
from udpack import UDP


def get_controller_delta(state: dict) -> np.ndarray:
    rel_loc = state.get("relative_location")
    if not isinstance(rel_loc, (list, tuple)) or len(rel_loc) < 3:
        return np.zeros(3, dtype=np.float64)
    return np.asarray(rel_loc[:3], dtype=np.float64)


def get_controller_rot_delta(state: dict) -> np.ndarray:
    rel_quat = state.get("relative_orientation")
    if not isinstance(rel_quat, (list, tuple)) or len(rel_quat) < 4:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    quat = np.asarray(rel_quat[:4], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return quat / norm


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    quat = np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )
    norm = np.linalg.norm(quat)
    if norm < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return quat / norm


def quat_inv(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=np.float64)
    norm_sq = float(np.dot(quat, quat))
    if norm_sq < 1e-8:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    w, x, y, z = quat
    return np.array([w, -x, -y, -z], dtype=np.float64) / norm_sq


@dataclass(frozen=True)
class SteamVRSnapshot:
    left_delta: np.ndarray
    right_delta: np.ndarray
    left_rot_delta: np.ndarray
    right_rot_delta: np.ndarray
    packet_age_s: float
    has_received_packet: bool


class SteamVRStateReceiver:
    def __init__(self, host: str, port: int):
        self._udp = UDP(recv_addr=(host, port), send_addr=None)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._has_received_packet = False
        self._last_packet_time = 0.0
        self._latest_deltas = {
            "left": np.zeros(3, dtype=np.float64),
            "right": np.zeros(3, dtype=np.float64),
        }
        self._latest_rot_deltas = {
            "left": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            "right": np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        }
        self._thread = threading.Thread(target=self._receiver_loop, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=1.0)

    def snapshot(self) -> SteamVRSnapshot:
        with self._lock:
            packet_age_s = max(0.0, time.monotonic() - self._last_packet_time) if self._has_received_packet else float("inf")
            return SteamVRSnapshot(
                left_delta=self._latest_deltas["left"].copy(),
                right_delta=self._latest_deltas["right"].copy(),
                left_rot_delta=self._latest_rot_deltas["left"].copy(),
                right_rot_delta=self._latest_rot_deltas["right"].copy(),
                packet_age_s=packet_age_s,
                has_received_packet=self._has_received_packet,
            )

    def _receiver_loop(self) -> None:
        while not self._stop_event.is_set():
            buffer = self._udp.recv(bufsize=4096, timeout=0.2)
            if buffer is None:
                continue

            try:
                states = pickle.loads(buffer)
            except Exception:
                continue
            if not isinstance(states, dict):
                continue

            left_delta = get_controller_delta(states.get("left", {}))
            right_delta = get_controller_delta(states.get("right", {}))
            left_rot_delta = get_controller_rot_delta(states.get("left", {}))
            right_rot_delta = get_controller_rot_delta(states.get("right", {}))
            with self._lock:
                self._latest_deltas["left"] = left_delta
                self._latest_deltas["right"] = right_delta
                self._latest_rot_deltas["left"] = left_rot_delta
                self._latest_rot_deltas["right"] = right_rot_delta
                self._last_packet_time = time.monotonic()
                self._has_received_packet = True
