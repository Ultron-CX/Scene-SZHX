#!/usr/bin/env python3
"""Prepare a fixed-base A2 Scene YAML strictly for non-qualification assembly smoke."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    import yaml

    parser = argparse.ArgumentParser()
    parser.add_argument("--template", type=Path, required=True)
    parser.add_argument("--scene-usda", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args()
    for path in (args.output, args.receipt):
        if path.exists():
            raise FileExistsError(f"refusing to overwrite: {path}")
    scene_usda = args.scene_usda.resolve()
    if not scene_usda.is_file():
        raise FileNotFoundError(scene_usda)
    config = yaml.safe_load(args.template.read_text(encoding="utf-8"))
    robot = config.get("robot", {})
    source = robot.get("robot_source", {})
    if "roboease_a2" not in str(robot.get("robot_name", "")):
        raise ValueError("non-qualification smoke requires the reviewed RoboEase A2 template")
    if source.get("pin_base_to_world") is not True or source.get("fix_base") is not True:
        raise ValueError("A2 smoke template must remain fixed-base")
    if config.get("cameras") or config.get("lidars"):
        raise ValueError("assembly smoke template must not enable sensors")
    foot_contacts_removed = robot.pop("foot_contacts", None) is not None
    config["scene"] = str(scene_usda)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(config, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
        newline="\n",
    )
    receipt = {
        "schema_version": "sceneforge/geniesim-scene-preparation/v1",
        "status": "SUCCEEDED",
        "purpose": "non_x2_non_qualification_assembly_smoke",
        "robot_probe": "roboease_a2_fixed_base",
        "template_sha256": sha256(args.template),
        "scene_usda": str(scene_usda),
        "scene_usda_sha256": sha256(scene_usda),
        "output_sha256": sha256(args.output),
        "foot_contacts_removed": foot_contacts_removed,
        "qualification": False,
        "real_robot_commands": False,
    }
    args.receipt.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(args.output)


if __name__ == "__main__":
    main()
