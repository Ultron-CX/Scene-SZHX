#!/usr/bin/env python3
"""PhysX-authoritative X2 foot-contact and non-foot-contact observations."""

from __future__ import annotations

import math
from typing import Any


def _numpy(value: Any):
    import numpy as np

    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


def enable_physx_contact_reports(stage, body_paths) -> tuple[str, ...]:
    """Author zero-threshold contact reports before the physics loop starts."""
    from pxr import PhysxSchema, UsdPhysics

    enabled = []
    for path in tuple(str(value) for value in body_paths):
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError(f"contact-report target is not a rigid body: {path}")
        api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
        api.CreateThresholdAttr().Set(0.0)
        enabled.append(path)
    if len(enabled) < 2:
        raise ValueError("contact-report instrumentation requires rigid bodies")
    return tuple(enabled)


class PhysXFootContactReader:
    """Read contact wrenches directly from the active Isaac PhysX scene.

    This class creates tensor contact views only after ``EngineSession.startup``
    and is called from the post-step hook after ``fetch_results``.  It neither
    advances physics nor writes commands.
    """

    FOOT_NAMES = ("left_ankle_roll_link", "right_ankle_roll_link")

    def __init__(self, stage, body_paths, *, physics_dt_s: float, ground_path: str = "/World/Ground"):
        import omni.physics.tensors as tensors
        from pxr import UsdUtils

        if not math.isfinite(physics_dt_s) or physics_dt_s <= 0.0:
            raise ValueError("physics_dt_s must be positive and finite")
        paths = tuple(str(path) for path in body_paths)
        if len(paths) < 2 or len(set(paths)) != len(paths):
            raise ValueError("body_paths must contain unique PhysX rigid bodies")
        foot_paths = []
        for name in self.FOOT_NAMES:
            matches = [path for path in paths if path.rsplit("/", 1)[-1] == name]
            if len(matches) != 1:
                raise ValueError(f"foot body {name!r} must resolve exactly once: {matches}")
            foot_paths.append(matches[0])
        if not stage.GetPrimAtPath(ground_path).IsValid():
            raise ValueError(f"ground contact filter does not exist: {ground_path}")

        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.simulation_view = tensors.create_simulation_view("numpy", stage_id)
        self.all_view = self.simulation_view.create_rigid_contact_view(list(paths))
        self.foot_view = self.simulation_view.create_rigid_contact_view(
            list(foot_paths),
            filter_patterns=[[ground_path], [ground_path]],
            max_contact_data_count=128,
        )
        if self.all_view.sensor_count != len(paths):
            raise RuntimeError(
                f"all-body contact view mismatch: {self.all_view.sensor_count}/{len(paths)}"
            )
        if self.foot_view.sensor_count != 2 or self.foot_view.filter_count != 1:
            raise RuntimeError(
                "foot contact view mismatch: "
                f"sensors={self.foot_view.sensor_count} filters={self.foot_view.filter_count}"
            )
        self.stage = stage
        self.body_paths = paths
        self.foot_paths = tuple(foot_paths)
        self.physics_dt_s = float(physics_dt_s)
        self.ground_path = ground_path
        self.previous_sim_time: float | None = None
        self.previous_positions: dict[str, tuple[float, float, float]] = {}
        self.sequence = 0

    def __call__(self, sim_time: float) -> dict:
        import numpy as np
        from kit.stage import _xform_to_xyzwxyz

        now = float(sim_time)
        if not math.isfinite(now):
            raise ValueError("contact sample time is not finite")
        if self.previous_sim_time is not None and now <= self.previous_sim_time:
            raise ValueError("contact sample time did not advance")

        all_forces = _numpy(self.all_view.get_net_contact_forces(self.physics_dt_s)).reshape((-1, 3))
        if all_forces.shape != (len(self.body_paths), 3) or not np.isfinite(all_forces).all():
            raise ValueError(f"invalid all-body contact force shape/state: {all_forces.shape}")

        normal_force, points, normals, separations, counts, starts = self.foot_view.get_contact_data(
            self.physics_dt_s
        )
        friction_force, friction_points, friction_counts, friction_starts = self.foot_view.get_friction_data(
            self.physics_dt_s
        )
        normal_force = _numpy(normal_force).reshape(-1)
        points = _numpy(points).reshape((-1, 3))
        normals = _numpy(normals).reshape((-1, 3))
        separations = _numpy(separations).reshape(-1)
        counts = _numpy(counts).reshape((2, 1)).astype(int)
        starts = _numpy(starts).reshape((2, 1)).astype(int)
        friction_force = _numpy(friction_force).reshape((-1, 3))
        friction_points = _numpy(friction_points).reshape((-1, 3))
        friction_counts = _numpy(friction_counts).reshape((2, 1)).astype(int)
        friction_starts = _numpy(friction_starts).reshape((2, 1)).astype(int)

        all_index = {path: index for index, path in enumerate(self.body_paths)}
        feet = {}
        dt = now - self.previous_sim_time if self.previous_sim_time is not None else None
        for index, (name, path) in enumerate(zip(self.FOOT_NAMES, self.foot_paths)):
            prim = self.stage.GetPrimAtPath(path)
            pose = tuple(float(value) for value in _xform_to_xyzwxyz(prim))
            origin = np.asarray(pose[:3], dtype=np.float64)
            previous = self.previous_positions.get(path)
            slip = 0.0
            if previous is not None and dt is not None and dt > 1e-9:
                velocity = (origin - np.asarray(previous, dtype=np.float64)) / dt
                slip = float(np.linalg.norm(velocity[:2]))

            start = int(starts[index, 0]); count = int(counts[index, 0])
            torque = np.zeros(3, dtype=np.float64)
            contact_points = []
            minimum_separation = None
            for offset in range(start, start + count):
                force = float(normal_force[offset]) * normals[offset]
                point = points[offset]
                torque += np.cross(point - origin, force)
                contact_points.append(point.tolist())
                separation = float(separations[offset])
                minimum_separation = separation if minimum_separation is None else min(minimum_separation, separation)
            friction_start = int(friction_starts[index, 0]); friction_count = int(friction_counts[index, 0])
            for offset in range(friction_start, friction_start + friction_count):
                torque += np.cross(friction_points[offset] - origin, friction_force[offset])

            net = all_forces[all_index[path]]
            feet[name] = {
                "body_path": path,
                "contact": count > 0,
                "contact_count": count,
                "friction_contact_count": friction_count,
                "net_force_n": net.tolist(),
                "net_torque_about_foot_origin_nm": torque.tolist(),
                "contact_points_m": contact_points,
                "minimum_separation_m": minimum_separation,
                "foot_position_m": origin.tolist(),
                "tangential_slip_speed_mps": slip,
            }
            self.previous_positions[path] = tuple(float(value) for value in origin)

        foot_indices = {all_index[path] for path in self.foot_paths}
        non_foot_norms = [
            float(np.linalg.norm(force))
            for index, force in enumerate(all_forces)
            if index not in foot_indices
        ]
        self.previous_sim_time = now
        self.sequence += 1
        return {
            "sequence": self.sequence,
            "sim_time": now,
            "physics_authority": "genie_sim_isaac_physx",
            "ground_filter_path": self.ground_path,
            "feet": feet,
            "maximum_non_foot_contact_force_n": max(non_foot_norms, default=0.0),
            "all_body_sensor_count": len(self.body_paths),
            "real_robot_commands": False,
        }
