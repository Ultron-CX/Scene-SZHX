#!/usr/bin/env python3
"""Run the offline X2 sim-time differential probe with the current ONNX policy."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
from pathlib import Path

from sceneforge.x2_inprocess_shadow import _OnnxPolicy
from sceneforge.x2_sim_time_differential import (
    load_state_sequence_payload,
    run_sim_time_differential_probe,
)


def _reject_nonfinite(value: str) -> None:
    raise ValueError(f"non-finite JSON constant is forbidden: {value}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict) -> None:
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, 0o444)
    finally:
        if temporary.exists():
            temporary.unlink()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--states", type=Path, required=True)
    parser.add_argument("--walk-model", type=Path, required=True)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--pause-before-index", type=int, required=True)
    parser.add_argument("--wall-sleep-s", type=float, default=2.0)
    args = parser.parse_args()

    states_path = args.states.resolve()
    model_path = args.walk_model.resolve()
    result_path = args.result.resolve()
    if not states_path.is_file():
        raise FileNotFoundError(states_path)
    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    if os.environ.get("REAL_ROBOT_COMMANDS", "false").lower() != "false":
        raise RuntimeError("REAL_ROBOT_COMMANDS must be false")
    payload = json.loads(
        states_path.read_text(encoding="utf-8"),
        parse_constant=_reject_nonfinite,
    )
    if not isinstance(payload, dict):
        raise ValueError("state-sequence JSON root must be an object")
    states = load_state_sequence_payload(payload)
    model_sha256 = _sha256(model_path)
    result = run_sim_time_differential_probe(
        states,
        lambda: _OnnxPolicy(model_path),
        pause_before_index=args.pause_before_index,
        wall_sleep_seconds=args.wall_sleep_s,
        policy_identity={
            "kind": "onnxruntime_cpu_sequential",
            "path": str(model_path),
            "sha256": model_sha256,
        },
    )
    result["input"]["path"] = str(states_path)
    result["input"]["file_sha256"] = _sha256(states_path)
    _atomic_json(result_path, result)
    print(result_path)
    return 0 if result["status"] == "PASSED" else 2


if __name__ == "__main__":
    raise SystemExit(main())

