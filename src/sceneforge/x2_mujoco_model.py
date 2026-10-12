"""Frozen X2 MuJoCo 3.3.5 model contract used by motion A/B gates."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from .x2_fixed_base_stand import _effort_limit
from .x2_stand import X2_STAND_JOINT_NAMES


X2_AUXILIARY_JOINT_NAMES = (
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
    "head_yaw_joint",
    "head_pitch_joint",
)
X2_ALL_JOINT_NAMES = (*X2_STAND_JOINT_NAMES, *X2_AUXILIARY_JOINT_NAMES)
X2_REQUIRED_SENSOR_NAMES = (
    "body-orientation",
    "body-angular-velocity",
    "body-linear-pos",
    "body-linear-vel",
    "body-linear-acceleration",
)


@dataclass(frozen=True)
class X2MujocoModelContract:
    mujoco_version: str
    timestep_s: float
    nq: int
    nv: int
    nu: int
    nbody: int
    njnt: int
    ngeom: int
    nsensor: int
    robot_body_mass_kg: float
    floating_joint_name: str | None
    joint_names: tuple[str | None, ...]
    actuator_names: tuple[str | None, ...]
    actuator_joint_names: tuple[str | None, ...]
    actuator_control_ranges: tuple[tuple[float, float], ...]
    sensor_names: tuple[str | None, ...]
    left_foot_geom_names: tuple[str, ...]
    right_foot_geom_names: tuple[str, ...]

    def checks(self) -> dict[str, bool]:
        joint_set = set(self.joint_names)
        actuator_joint_set = set(self.actuator_joint_names)
        sensor_set = set(self.sensor_names)
        expected_ranges = {
            name: _effort_limit(name) for name in X2_ALL_JOINT_NAMES
        }
        observed_ranges = {
            joint: limits
            for joint, limits in zip(
                self.actuator_joint_names, self.actuator_control_ranges
            )
            if joint is not None
        }
        ranges_match = all(
            name in observed_ranges
            and math.isclose(observed_ranges[name][0], -limit, abs_tol=2.0)
            and math.isclose(observed_ranges[name][1], limit, abs_tol=2.0)
            for name, limit in expected_ranges.items()
        )
        return {
            "mujoco_version_is_3_3_5": self.mujoco_version == "3.3.5",
            "timestep_is_1ms": math.isclose(self.timestep_s, 0.001, abs_tol=1e-12),
            "floating_dimensions_match": self.nq == 38 and self.nv == 37,
            "joint_count_matches": self.njnt == 32,
            "actuator_count_matches": self.nu == 31,
            "floating_joint_matches": self.floating_joint_name == "floating_base_joint",
            "all_31_joints_present": set(X2_ALL_JOINT_NAMES).issubset(joint_set),
            "all_31_joints_actuated": set(X2_ALL_JOINT_NAMES) == actuator_joint_set,
            "effort_ranges_match_release": ranges_match,
            "base_sensors_present": set(X2_REQUIRED_SENSOR_NAMES).issubset(sensor_set),
            "seven_left_foot_geoms": len(self.left_foot_geom_names) == 7,
            "seven_right_foot_geoms": len(self.right_foot_geom_names) == 7,
            "mass_is_finite_positive": math.isfinite(self.robot_body_mass_kg)
            and self.robot_body_mass_kg > 0.0,
        }

    def as_dict(self) -> dict[str, Any]:
        checks = self.checks()
        return {
            "mujoco_version": self.mujoco_version,
            "timestep_s": self.timestep_s,
            "dimensions": {
                "nq": self.nq,
                "nv": self.nv,
                "nu": self.nu,
                "nbody": self.nbody,
                "njnt": self.njnt,
                "ngeom": self.ngeom,
                "nsensor": self.nsensor,
            },
            "robot_body_mass_kg": self.robot_body_mass_kg,
            "floating_joint_name": self.floating_joint_name,
            "joint_names": list(self.joint_names),
            "actuator_names": list(self.actuator_names),
            "actuator_joint_names": list(self.actuator_joint_names),
            "actuator_control_ranges": [list(values) for values in self.actuator_control_ranges],
            "sensor_names": list(self.sensor_names),
            "left_foot_geom_names": list(self.left_foot_geom_names),
            "right_foot_geom_names": list(self.right_foot_geom_names),
            "checks": checks,
            "passed": all(checks.values()),
            "read_only": True,
            "real_robot_commands": False,
        }


def inspect_x2_mujoco_model(mujoco: Any, model: Any) -> X2MujocoModelContract:
    """Inspect a loaded model without creating ``MjData`` or stepping physics."""

    def names(kind: Any, count: int) -> tuple[str | None, ...]:
        return tuple(mujoco.mj_id2name(model, kind, index) for index in range(count))

    joint_names = names(mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
    actuator_names = names(mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)
    sensor_names = names(mujoco.mjtObj.mjOBJ_SENSOR, model.nsensor)
    geom_names = names(mujoco.mjtObj.mjOBJ_GEOM, model.ngeom)
    actuator_joint_names = tuple(
        mujoco.mj_id2name(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            int(model.actuator_trnid[index, 0]),
        )
        for index in range(model.nu)
    )
    free_joint = next(
        (
            joint_names[index]
            for index in range(model.njnt)
            if int(model.jnt_type[index]) == int(mujoco.mjtJoint.mjJNT_FREE)
        ),
        None,
    )
    pelvis_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis"))
    if pelvis_id < 0:
        raise ValueError("X2 model has no pelvis body")

    def is_robot_body(body_id: int) -> bool:
        current = body_id
        while current > 0 and current != pelvis_id:
            current = int(model.body_parentid[current])
        return current == pelvis_id

    return X2MujocoModelContract(
        mujoco_version=str(mujoco.__version__),
        timestep_s=float(model.opt.timestep),
        nq=int(model.nq),
        nv=int(model.nv),
        nu=int(model.nu),
        nbody=int(model.nbody),
        njnt=int(model.njnt),
        ngeom=int(model.ngeom),
        nsensor=int(model.nsensor),
        robot_body_mass_kg=sum(
            float(model.body_mass[index])
            for index in range(1, model.nbody)
            if is_robot_body(index)
        ),
        floating_joint_name=free_joint,
        joint_names=joint_names,
        actuator_names=actuator_names,
        actuator_joint_names=actuator_joint_names,
        actuator_control_ranges=tuple(
            (float(values[0]), float(values[1])) for values in model.actuator_ctrlrange
        ),
        sensor_names=sensor_names,
        left_foot_geom_names=tuple(
            name for name in geom_names if name and name.startswith("left_foot")
        ),
        right_foot_geom_names=tuple(
            name for name in geom_names if name and name.startswith("right_foot")
        ),
    )
