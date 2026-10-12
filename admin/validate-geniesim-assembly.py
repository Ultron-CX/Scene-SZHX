#!/usr/bin/env python3
"""Validate Genie Sim assembly output without launching physics."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def resolve_manifest_path(value: str, base_path: Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else (base_path / path).resolve()


def main() -> None:
    import yaml
    from pxr import Usd

    parser = argparse.ArgumentParser()
    parser.add_argument("--assembly", type=Path, required=True)
    parser.add_argument("--expected-scene", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args()
    if args.receipt.exists():
        raise FileExistsError(f"refusing to overwrite: {args.receipt}")
    manifest_path = args.assembly / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    base_path = Path(manifest.get("base_path") or args.assembly)
    expected_scene = args.expected_scene.resolve()
    scene = resolve_manifest_path(manifest["scene_usda"], base_path)
    robot = resolve_manifest_path(manifest["robot_usda"], base_path)
    render = resolve_manifest_path(manifest["render_layer_usda"], base_path)
    scene_yaml = resolve_manifest_path(manifest["scene_yaml"], base_path)
    if scene != expected_scene:
        raise ValueError(f"assembled scene mismatch: {scene} != {expected_scene}")
    for label, path in (("scene", scene), ("robot", robot), ("render", render)):
        if not path.is_file() or Usd.Stage.Open(str(path)) is None:
            raise ValueError(f"{label} USD cannot be opened: {path}")
    config = yaml.safe_load(scene_yaml.read_text(encoding="utf-8"))
    source = config["robot"]["robot_source"]
    if source.get("pin_base_to_world") is not True or source.get("fix_base") is not True:
        raise ValueError("non-qualification A2 probe is not fixed-base")
    receipt = {
        "schema_version": "sceneforge/geniesim-assembly-validation/v1",
        "status": "PASSED",
        "purpose": "non_x2_non_qualification_assembly_smoke",
        "manifest_sha256": sha256(manifest_path),
        "scene_usda_sha256": sha256(scene),
        "robot_usda_sha256": sha256(robot),
        "render_layer_usda_sha256": sha256(render),
        "usd_version": list(Usd.GetVersion()),
        "robot_prefix": manifest["robot_prefix"],
        "scene_reference_matches": True,
        "fixed_base_probe": True,
        "qualification": False,
        "limitations": [
            "assembly_only_physics_launcher_not_started",
            "a2_probe_does_not_substitute_for_x2",
        ],
        "real_robot_commands": False,
    }
    args.receipt.parent.mkdir(parents=True, exist_ok=True)
    args.receipt.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(args.receipt)


if __name__ == "__main__":
    main()

