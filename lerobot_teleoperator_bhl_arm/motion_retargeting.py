from __future__ import annotations

import atexit
import os
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import mink
import mujoco
import numpy as np


@dataclass(frozen=True)
class IKSolveInfo:
    iterations: int
    error_norm: float
    converged: bool


def default_bhl_mjcf_path() -> Path:
    local_path = Path("./data/bhl_arm/mjcf/bhl_arm.xml")
    if local_path.exists():
        return local_path
    raise FileNotFoundError(f"MuJoCo XML not found: {local_path}")


def set_mocap_marker(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    marker_name: str,
    position: np.ndarray,
    orientation: np.ndarray,
) -> None:
    mocap_id = model.body(marker_name).mocapid[0]
    if mocap_id < 0:
        raise ValueError(f"Body '{marker_name}' is not a mocap body")
    data.mocap_pos[mocap_id] = position
    data.mocap_quat[mocap_id] = orientation


def _add_axis_geoms(
    body_elem: ET.Element,
    *,
    center_color: tuple[float, float, float],
    axis_length: float,
    axis_radius: float,
) -> None:
    ET.SubElement(
        body_elem,
        "geom",
        type="sphere",
        size=f"{axis_radius * 1.5:.6f}",
        rgba=f"{center_color[0]} {center_color[1]} {center_color[2]} 0.8",
        contype="0",
        conaffinity="0",
    )
    ET.SubElement(
        body_elem,
        "geom",
        type="capsule",
        fromto=f"0 0 0 {axis_length:.6f} 0 0",
        size=f"{axis_radius:.6f}",
        rgba="1 0 0 0.9",
        contype="0",
        conaffinity="0",
    )
    ET.SubElement(
        body_elem,
        "geom",
        type="capsule",
        fromto=f"0 0 0 0 {axis_length:.6f} 0",
        size=f"{axis_radius:.6f}",
        rgba="0 1 0 0.9",
        contype="0",
        conaffinity="0",
    )
    ET.SubElement(
        body_elem,
        "geom",
        type="capsule",
        fromto=f"0 0 0 0 0 {axis_length:.6f}",
        size=f"{axis_radius:.6f}",
        rgba="0 0 1 0.9",
        contype="0",
        conaffinity="0",
    )


def add_body_frames(
    xml: str,
    body_names: list[str],
    *,
    prefix: str,
    center_color: tuple[float, float, float],
    axis_length: float = 0.06,
    axis_radius: float = 0.004,
) -> str:
    root = ET.fromstring(xml)
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("Invalid MuJoCo XML: missing <worldbody>")

    for body_name in body_names:
        frame_name = f"{prefix}{body_name}_frame"
        frame_body = ET.SubElement(
            worldbody,
            "body",
            name=frame_name,
            mocap="true",
            pos="0 0 0",
            quat="1 0 0 0",
        )
        _add_axis_geoms(
            frame_body,
            center_color=center_color,
            axis_length=axis_length,
            axis_radius=axis_radius,
        )

    return ET.tostring(root, encoding="unicode")


def create_frame_visualization_xml(robot_xml: str | Path, body_names: list[str]) -> str:
    xml_path = Path(robot_xml)
    if not xml_path.exists():
        raise FileNotFoundError(f"MuJoCo XML not found: {xml_path}")

    xml = xml_path.read_text(encoding="utf-8")
    xml = add_body_frames(xml, body_names, prefix="current_", center_color=(0.0, 1.0, 1.0))
    xml = add_body_frames(xml, body_names, prefix="target_", center_color=(1.0, 0.0, 1.0))

    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".xml",
        delete=False,
        encoding="utf-8",
        dir=xml_path.resolve().parent,
    ) as file:
        file.write(xml)
        tmp_path = file.name

    atexit.register(lambda p=tmp_path: os.path.exists(p) and os.unlink(p))
    return tmp_path


class BHLMotionRetargeting:
    """Task-space pose retargeting on a plain MuJoCo model/data pair."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        joint_names: list[str],
        solver: str = "daqp",
        damping: float = 1e-3,
        step_size: float = 1.0,
        max_joint_delta: float = 0.12,
        position_weight: float = 1.0,
        orientation_weight: float = 0.25,
    ):
        self.model = model
        self.data = data
        self.joint_names = joint_names

        self.solver = solver
        self.damping = damping
        self.step_size = step_size
        self.max_joint_delta = max_joint_delta
        self.position_weight = position_weight
        self.orientation_weight = orientation_weight

        self.joint_qpos_ids = np.array(
            [self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in joint_names],
            dtype=np.int32,
        )
        self.joint_dof_ids = np.array(
            [self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in joint_names],
            dtype=np.int32,
        )

        self.configuration = mink.Configuration(self.model)
        self.frame_tasks: dict[str, mink.FrameTask] = {}
        self.tasks: list[object] = []
        self.posture_task = mink.PostureTask(self.model, cost=0.1)
        self.tasks.append(self.posture_task)
        self.limits = [mink.ConfigurationLimit(self.model)]

        self.set_joint_positions(np.zeros(len(joint_names), dtype=np.float32))

    def set_joint_positions(self, joint_positions: np.ndarray) -> None:
        joint_positions = np.asarray(joint_positions, dtype=np.float64)
        self.data.qpos[self.joint_qpos_ids] = joint_positions
        self.data.qvel[self.joint_dof_ids] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def get_joint_positions(self) -> np.ndarray:
        return self.data.qpos[self.joint_qpos_ids].astype(np.float32, copy=True)

    def get_body_pose(self, body_name: str) -> tuple[np.ndarray, np.ndarray]:
        body_id = self.model.body(body_name).id
        return self.data.xpos[body_id].copy(), self.data.xquat[body_id].copy()

    def _sync_configuration_from_data(self) -> None:
        self.configuration.data.qpos[:] = self.data.qpos
        self.configuration.data.qvel[:] = self.data.qvel
        mujoco.mj_forward(self.configuration.model, self.configuration.data)

    def _sync_data_from_configuration(self) -> None:
        self.data.qpos[:] = self.configuration.data.qpos
        self.data.qvel[:] = self.configuration.data.qvel
        mujoco.mj_forward(self.model, self.data)

    def _get_or_create_frame_task(self, body_name: str) -> mink.FrameTask:
        task = self.frame_tasks.get(body_name)
        if task is not None:
            return task

        if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name) < 0:
            raise ValueError(f"Body '{body_name}' not found in model")

        task = mink.FrameTask(
            frame_name=body_name,
            frame_type="body",
            position_cost=self.position_weight,
            orientation_cost=self.orientation_weight,
            lm_damping=1.0,
        )
        task.set_target_from_configuration(self.configuration)
        self.frame_tasks[body_name] = task
        self.tasks.append(task)
        return task

    def _set_targets(self, target_poses: dict[str, tuple[np.ndarray, np.ndarray]]) -> list[mink.FrameTask]:
        active_tasks: list[mink.FrameTask] = []
        for body_name, (target_pos, target_quat) in target_poses.items():
            task = self._get_or_create_frame_task(body_name)
            task.set_target(
                mink.SE3(
                    wxyz_xyz=np.concatenate(
                        [
                            np.asarray(target_quat, dtype=np.float64),
                            np.asarray(target_pos, dtype=np.float64),
                        ]
                    )
                )
            )
            active_tasks.append(task)
        return active_tasks

    def _calculate_error(self, active_tasks: list[mink.FrameTask]) -> float:
        if not active_tasks:
            return 0.0
        return float(
            np.linalg.norm(
                np.concatenate([task.compute_error(self.configuration) for task in active_tasks])
            )
        )

    def solve_ik(
        self,
        target_poses: dict[str, tuple[np.ndarray, np.ndarray]],
        *,
        max_iter: int = 30,
        error_tolerance: float = 1e-4,
    ) -> tuple[np.ndarray, IKSolveInfo]:
        self._sync_configuration_from_data()
        active_tasks = self._set_targets(target_poses)
        self.posture_task.set_target_from_configuration(self.configuration)

        converged = False
        error_norm = self._calculate_error(active_tasks)
        prev_error = error_norm
        dt = self.configuration.model.opt.timestep
        improvement_tolerance = 1e-3

        for iteration in range(max_iter):
            if error_norm < error_tolerance:
                converged = True
                self._sync_data_from_configuration()
                return self.get_joint_positions(), IKSolveInfo(iteration + 1, error_norm, converged)

            velocity = mink.solve_ik(
                configuration=self.configuration,
                tasks=self.tasks,
                dt=dt,
                solver=self.solver,
                damping=self.damping,
                limits=self.limits,
            )
            if self.max_joint_delta > 0:
                velocity = np.clip(
                    velocity,
                    -self.max_joint_delta / max(dt, 1e-8),
                    self.max_joint_delta / max(dt, 1e-8),
                )

            self.configuration.integrate_inplace(velocity * self.step_size, dt)
            error_norm = self._calculate_error(active_tasks)
            if prev_error - error_norm <= improvement_tolerance:
                break
            prev_error = error_norm

        self._sync_data_from_configuration()
        return self.get_joint_positions(), IKSolveInfo(max_iter, float(error_norm), converged)
