"""PhysX-authoritative floating-base pose and velocity observations for X2."""

from __future__ import annotations

import math
from typing import Any, Iterable

import numpy as np


def _numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value, dtype=np.float64)


def rotation_world_from_body_wxyz(quaternion: Iterable[float]) -> np.ndarray:
    """Return ``R_world_from_body`` for a normalized scalar-first quaternion."""

    values = np.asarray(tuple(float(v) for v in quaternion), dtype=np.float64)
    if values.shape != (4,) or not np.all(np.isfinite(values)):
        raise ValueError("base quaternion must contain four finite values")
    norm = float(np.linalg.norm(values))
    if norm <= 1e-12:
        raise ValueError("base quaternion norm is zero")
    w, x, y, z = values / norm
    return np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
            (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
            (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )


def body_twist_from_world(
    quaternion_wxyz: Iterable[float],
    linear_velocity_world: Iterable[float],
    angular_velocity_world: Iterable[float],
) -> tuple[float, ...]:
    """Convert PhysX world-frame rigid-body velocity into base-link coordinates."""

    linear = np.asarray(tuple(float(v) for v in linear_velocity_world), dtype=np.float64)
    angular = np.asarray(tuple(float(v) for v in angular_velocity_world), dtype=np.float64)
    if linear.shape != (3,) or angular.shape != (3,):
        raise ValueError("base velocities must each contain three values")
    if not np.all(np.isfinite(linear)) or not np.all(np.isfinite(angular)):
        raise ValueError("base velocities contain NaN or Inf")
    body_from_world = rotation_world_from_body_wxyz(quaternion_wxyz).T
    return tuple(float(v) for v in np.concatenate((body_from_world @ linear, body_from_world @ angular)))


def pose_wxyz_from_tensor_transform(transform_xyz_xyzw: Iterable[float]) -> np.ndarray:
    """Convert the PhysX tensor transform layout into the X2 pose layout."""

    transform = np.asarray(tuple(float(v) for v in transform_xyz_xyzw), dtype=np.float64)
    if transform.shape != (7,) or not np.all(np.isfinite(transform)):
        raise ValueError("PhysX tensor transform must contain seven finite values")
    return transform[[0, 1, 2, 6, 3, 4, 5]]


class PhysXBaseStateReader:
    """Read base-link state from the active PhysX rigid-body view.

    This reader is observation-only: it does not author USD, apply forces,
    set poses, publish commands, or advance simulation time.
    """

    source = "physx_rigid_prim_world_velocity_to_base"

    def __init__(self, stage: Any, body_paths: Iterable[str]) -> None:
        matches = [str(path) for path in body_paths if str(path).rsplit("/", 1)[-1] == "base_link"]
        if len(matches) != 1:
            raise ValueError(f"base_link rigid body must resolve exactly once: {matches}")
        import omni.physics.tensors as tensors
        from pxr import UsdUtils

        self.path = matches[0]
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self._simulation_view = tensors.create_simulation_view("numpy", stage_id)
        self._body_view = self._simulation_view.create_rigid_body_view(self.path)
        if self._body_view.count != 1 or tuple(self._body_view.prim_paths) != (self.path,):
            raise RuntimeError(
                f"PhysX base-link view mismatch: count={self._body_view.count} "
                f"paths={tuple(self._body_view.prim_paths)}"
            )

    def read(self, _sim_time: float) -> tuple[np.ndarray, np.ndarray]:
        transform = _numpy(self._body_view.get_transforms()).reshape((-1, 7))[0]
        velocity = _numpy(self._body_view.get_velocities()).reshape((-1, 6))[0]
        # Tensor transforms are xyz + xyzw; the X2 control contract is xyz + wxyz.
        pose = pose_wxyz_from_tensor_transform(transform)
        orientation = pose[3:]
        twist = np.asarray(
            body_twist_from_world(orientation, velocity[:3], velocity[3:]),
            dtype=np.float64,
        )
        if not np.all(np.isfinite(pose)) or not np.all(np.isfinite(twist)):
            raise RuntimeError("PhysX base state contains NaN or Inf")
        if not math.isclose(float(np.linalg.norm(orientation)), 1.0, abs_tol=1e-3):
            raise RuntimeError("PhysX base quaternion is not normalized")
        return pose, twist
