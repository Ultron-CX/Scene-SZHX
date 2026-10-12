#!/usr/bin/env python3
"""Extract an immutable 50 Hz X2 replay sequence from a PhysX trace."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
from pathlib import Path

from sceneforge.x2_sim_time_differential import (
    extract_state_sequence_from_physx_trace,
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
        os.link(temporary, path)
        os.chmod(path, 0o444)
    finally:
        if temporary.exists():
            temporary.unlink()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--phase", default="GAIT1_FORWARD")
    parser.add_argument("--start-offset-s", type=float, default=1.0)
    parser.add_argument("--duration-s", type=float, default=3.0)
    parser.add_argument("--sample-period-s", type=float, default=0.02)
    args = parser.parse_args()
    trace_path = args.trace.resolve()
    output_path = args.output.resolve()
    if not trace_path.is_file():
        raise FileNotFoundError(trace_path)
    trace = json.loads(
        trace_path.read_text(encoding="utf-8"),
        parse_constant=_reject_nonfinite,
    )
    if not isinstance(trace, dict):
        raise ValueError("PhysX trace JSON root must be an object")
    payload = extract_state_sequence_from_physx_trace(
        trace,
        phase=args.phase,
        start_offset_s=args.start_offset_s,
        duration_s=args.duration_s,
        sample_period_s=args.sample_period_s,
    )
    payload["source"]["path"] = str(trace_path)
    payload["source"]["sha256"] = _sha256(trace_path)
    _atomic_json(output_path, payload)
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
