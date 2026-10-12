#!/usr/bin/env python3
"""Auditable, runtime-only X2 PhysX dynamics alignment helpers."""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path


SCHEMA = "sceneforge/x2-physx-dynamics-alignment/v1"


def _atomic_json(path: Path, payload: dict) -> None:
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name("." + path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def configure_physx_dynamics_pre_reset(
    stage,
    *,
    ground_path: str,
    static_friction: float,
    dynamic_friction: float,
    foot_collision_mode: str,
    receipt: str | Path,
) -> dict:
    """Author isolated X2 alignment changes before PhysX builds its scene."""
    static_friction = float(static_friction)
    dynamic_friction = float(dynamic_friction)
    if not all(math.isfinite(value) and value >= 0.0 for value in (static_friction, dynamic_friction)):
        raise ValueError("friction coefficients must be finite and non-negative")
    if dynamic_friction > static_friction:
        raise ValueError("dynamic friction must not exceed static friction")

    foot_collision_mode = str(foot_collision_mode).strip().lower()
    if foot_collision_mode not in {"imported_mesh", "vendor_capsules"}:
        raise ValueError(f"unsupported foot collision mode: {foot_collision_mode}")

    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    ground = stage.GetPrimAtPath(ground_path)
    if not ground.IsValid() or not ground.HasAPI(UsdPhysics.CollisionAPI):
        raise ValueError(f"ground is not a collision prim: {ground_path}")
    material_path = "/World/PhysicsMaterials/X2GroundFriction"
    material = UsdShade.Material.Define(stage, material_path)
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr().Set(static_friction)
    physics.CreateDynamicFrictionAttr().Set(dynamic_friction)
    physics.CreateRestitutionAttr().Set(0.0)
    physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx.CreateFrictionCombineModeAttr().Set("max")
    physx.CreateRestitutionCombineModeAttr().Set("min")
    binding = ground.CreateRelationship("material:binding:physics", custom=False)
    binding.SetTargets([Sdf.Path(material_path)])

    foot_paths: list[str] = []
    deinstanced_paths: list[str] = []
    disabled_collision_paths: list[str] = []
    authored_collision_paths: list[str] = []
    if foot_collision_mode == "vendor_capsules":
        capsule_rows = (
            (-0.0500, -0.040, 0.115),
            (-0.0375, -0.057, 0.133),
            (-0.0200, -0.061, 0.137),
            (0.0000, -0.062, 0.138),
            (0.0200, -0.061, 0.137),
            (0.0375, -0.057, 0.133),
            (0.0500, -0.040, 0.115),
        )
        for foot_name in ("left_ankle_roll_link", "right_ankle_roll_link"):
            matches = [
                prim for prim in stage.Traverse()
                if prim.GetName() == foot_name and prim.HasAPI(UsdPhysics.RigidBodyAPI)
            ]
            if len(matches) != 1:
                raise ValueError(f"foot rigid body must resolve once: {foot_name} -> {matches}")
            foot = matches[0]
            foot_paths.append(str(foot.GetPath()))
            for child in foot.GetChildren():
                if child.IsInstance() or child.IsInstanceable():
                    child.SetInstanceable(False)
                    deinstanced_paths.append(str(child.GetPath()))
            existing = [
                prim for prim in Usd.PrimRange(foot)
                if prim.GetPath() != foot.GetPath() and prim.HasAPI(UsdPhysics.CollisionAPI)
            ]
            if not existing:
                raise ValueError(f"no imported foot collision found below {foot.GetPath()}")
            for prim in existing:
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
                disabled_collision_paths.append(str(prim.GetPath()))
            for index, (y, x1, x2) in enumerate(capsule_rows, 1):
                path = foot.GetPath().AppendChild(f"vendor_foot_capsule_{index}")
                if stage.GetPrimAtPath(path).IsValid():
                    raise ValueError(f"vendor foot collider already exists: {path}")
                capsule = UsdGeom.Capsule.Define(stage, path)
                capsule.CreateAxisAttr().Set(UsdGeom.Tokens.x)
                capsule.CreateRadiusAttr().Set(0.01)
                capsule.CreateHeightAttr().Set(x2 - x1)
                UsdGeom.XformCommonAPI(capsule).SetTranslate(
                    Gf.Vec3d((x1 + x2) / 2.0, y, -0.063)
                )
                UsdPhysics.CollisionAPI.Apply(capsule.GetPrim()).CreateCollisionEnabledAttr().Set(True)
                capsule.GetPrim().CreateRelationship(
                    "material:binding:physics", custom=False
                ).SetTargets([Sdf.Path(material_path)])
                authored_collision_paths.append(str(path))

    payload = {
        "schema": SCHEMA,
        "status": "PASSED",
        "scope": (
            "ground_friction_only"
            if foot_collision_mode == "imported_mesh"
            else "ground_friction_and_vendor_foot_capsules"
        ),
        "pre_world_reset": True,
        "physics_authority": "genie_sim_isaac_physx",
        "ground_path": ground_path,
        "material_path": material_path,
        "static_friction": static_friction,
        "dynamic_friction": dynamic_friction,
        "restitution": 0.0,
        "friction_combine_mode": "max",
        "foot_collision_mode": foot_collision_mode,
        "foot_collision_geometry_changed": foot_collision_mode == "vendor_capsules",
        "foot_rigid_body_paths": foot_paths,
        "deinstanced_foot_geometry_paths": deinstanced_paths,
        "disabled_imported_collision_paths": disabled_collision_paths,
        "authored_vendor_capsule_paths": authored_collision_paths,
        "authored_vendor_capsule_count": len(authored_collision_paths),
        "joint_or_controller_parameters_changed": False,
        "bound_targets": [str(value) for value in binding.GetTargets()],
        "stage_root_layer": str(stage.GetRootLayer().identifier),
        "stage_root_layer_sha256": (
            hashlib.sha256(Path(stage.GetRootLayer().realPath).read_bytes()).hexdigest()
            if stage.GetRootLayer().realPath and Path(stage.GetRootLayer().realPath).is_file()
            else None
        ),
        "published_command_topics": [],
        "real_robot_commands": False,
    }
    if payload["bound_targets"] != [material_path]:
        raise RuntimeError("physics material binding did not round-trip")
    if foot_collision_mode == "vendor_capsules" and len(authored_collision_paths) != 14:
        raise RuntimeError("vendor foot collision authoring did not produce 14 capsules")
    _atomic_json(Path(receipt), payload)
    return payload
