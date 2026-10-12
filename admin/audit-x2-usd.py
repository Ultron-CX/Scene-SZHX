#!/usr/bin/env python3
"""Read-only USD schema audit for a generated X2 robot package."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from pxr import Usd, UsdPhysics


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", required=True, type=Path)
    parser.add_argument("--receipt", required=True, type=Path)
    args = parser.parse_args()

    stage = Usd.Stage.Open(str(args.stage))
    if stage is None:
        raise SystemExit(f"cannot open stage: {args.stage}")

    rigid = []
    collisions = []
    masses = []
    revolute = []
    fixed = []
    prismatic = []
    invalid_mass = []
    # AS3 geometry is instanceable.  Plain Stage.Traverse() deliberately
    # skips instance proxies and would falsely report zero colliders even
    # though PhysicsCollisionAPI is authored in payloads/instances.usda.
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid.append(path)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collisions.append(path)
        if prim.HasAPI(UsdPhysics.MassAPI):
            masses.append(path)
            value = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
            if value is not None and (not math.isfinite(float(value)) or float(value) <= 0):
                invalid_mass.append({"path": path, "mass": value})
        if prim.IsA(UsdPhysics.RevoluteJoint):
            revolute.append(path)
        elif prim.IsA(UsdPhysics.FixedJoint):
            fixed.append(path)
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            prismatic.append(path)

    receipt = {
        "schema_version": "sceneforge/x2-usd-audit/v1",
        "status": "PASSED" if rigid and revolute and collisions and not invalid_mass else "FAILED",
        "stage": str(args.stage),
        "stage_sha256": sha256(args.stage),
        "default_prim": str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None,
        "rigid_body_count": len(rigid),
        "collision_prim_count": len(collisions),
        "mass_api_count": len(masses),
        "revolute_joint_count": len(revolute),
        "fixed_joint_count": len(fixed),
        "prismatic_joint_count": len(prismatic),
        "invalid_mass": invalid_mass,
        "rigid_body_paths": rigid,
        "collision_paths": collisions,
        "revolute_joint_paths": revolute,
        "read_only": True,
        "physics_advanced": False,
        "controller_loaded": False,
        "real_robot_commands": False,
    }
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps(receipt, ensure_ascii=False, indent=2))
    if receipt["status"] != "PASSED":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
