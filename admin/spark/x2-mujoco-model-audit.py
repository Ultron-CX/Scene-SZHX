#!/usr/bin/env python3
"""Create immutable, read-only evidence for the frozen vendor X2 MJCF."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, payload: object) -> None:
    if path.exists():
        raise FileExistsError(f"refusing to overwrite {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, 0o444)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args()

    if os.environ.get("REAL_ROBOT_COMMANDS", "false").lower() != "false":
        raise RuntimeError("REAL_ROBOT_COMMANDS must be false")
    import mujoco
    from sceneforge.x2_mujoco_model import inspect_x2_mujoco_model

    model_path = args.model.resolve()
    receipt = args.receipt.resolve()
    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    contract = inspect_x2_mujoco_model(mujoco, model)
    payload = {
        "schema": "sceneforge/x2-mujoco-model-audit/v1",
        "status": "PASSED" if all(contract.checks().values()) else "FAILED",
        "qualification": False,
        "physics_advanced": False,
        "command_application_count": 0,
        "published_command_topics": [],
        "model": {
            "path": str(model_path),
            "size": model_path.stat().st_size,
            "sha256": sha256(model_path),
        },
        "contract": contract.as_dict(),
        "mujoco_state_source_used": False,
        "real_robot_commands": False,
    }
    atomic_json(receipt, payload)
    print(json.dumps({"status": payload["status"], "receipt": str(receipt)}))
    return 0 if payload["status"] == "PASSED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
