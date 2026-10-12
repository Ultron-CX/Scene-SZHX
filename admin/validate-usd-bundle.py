#!/usr/bin/env python3
"""Validate a SceneForge USD bundle with the runtime's pxr installation."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate(bundle: Path) -> dict[str, Any]:
    from pxr import Usd

    manifest_path = bundle / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("real_robot_commands") is not False:
        raise ValueError("manifest must explicitly set real_robot_commands=false")
    for name, expected in manifest["files"].items():
        path = bundle / name
        if not path.is_file():
            raise FileNotFoundError(path)
        actual = sha256(path)
        if actual != expected:
            raise ValueError(f"hash mismatch for {name}: {actual} != {expected}")

    layout = json.loads((bundle / "building-layout.json").read_text(encoding="utf-8"))
    stage = Usd.Stage.Open(str(bundle / "scene.usda"))
    if stage is None:
        raise ValueError("Usd.Stage.Open returned None")
    default_prim = stage.GetDefaultPrim()
    if not default_prim or default_prim.GetPath().pathString != "/SceneForge":
        raise ValueError("default prim must be /SceneForge")

    wall_prims = [
        prim
        for prim in stage.Traverse()
        if prim.GetPath().pathString.startswith("/SceneForge/Structure/wall_")
    ]
    expected_walls = len(layout["walls"])
    if len(wall_prims) != expected_walls:
        raise ValueError(f"wall prim count mismatch: {len(wall_prims)} != {expected_walls}")
    floor = stage.GetPrimAtPath("/SceneForge/Structure/floor")
    if not floor:
        raise ValueError("floor prim is missing")
    collision_prims = [floor, *wall_prims]
    missing_collision = [
        prim.GetPath().pathString
        for prim in collision_prims
        if "PhysicsCollisionAPI" not in prim.GetAppliedSchemas()
    ]
    if missing_collision:
        raise ValueError(f"collision API missing on prims: {missing_collision}")

    semantic_spaces = [
        prim
        for prim in stage.Traverse()
        if prim.GetPath().pathString.startswith("/SceneForge/Semantics/")
        and prim.GetName() in {space["space_id"] for space in layout["spaces"]}
    ]
    if len(semantic_spaces) != len(layout["spaces"]):
        raise ValueError("semantic space prim count mismatch")

    return {
        "schema_version": "sceneforge/usd-runtime-validation/v1",
        "status": "PASSED",
        "bundle": str(bundle),
        "manifest_sha256": sha256(manifest_path),
        "usd_version": list(Usd.GetVersion()),
        "default_prim": default_prim.GetPath().pathString,
        "space_count": len(layout["spaces"]),
        "door_count": len(layout["doors"]),
        "wall_prim_count": len(wall_prims),
        "collision_prim_count": len(collision_prims),
        "checks": {
            "manifest_hashes": "PASSED",
            "stage_open": "PASSED",
            "default_prim": "PASSED",
            "wall_prim_count": "PASSED",
            "semantic_space_count": "PASSED",
            "physics_collision_api": "PASSED",
        },
        "limitations": [
            "usd_composition_valid_not_yet_geniesim_launcher_qualified",
            "x2_dynamic_qualification_blocked_missing_authoritative_assets",
        ],
        "real_robot_commands": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args()
    if args.receipt.exists():
        raise FileExistsError(f"refusing to overwrite receipt: {args.receipt}")
    result = validate(args.bundle.resolve())
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(args.receipt)


if __name__ == "__main__":
    main()

