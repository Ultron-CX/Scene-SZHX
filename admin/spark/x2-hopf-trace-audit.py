#!/usr/bin/env python3
"""Validate the engine-neutral Hopf implementation against vendor traces."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
import tempfile
import time
from pathlib import Path

import sys


CORE_PATH = Path(__file__).resolve().parents[2] / "src" / "sceneforge" / "x2_control.py"
CORE_SPEC = importlib.util.spec_from_file_location("sceneforge_x2_control_standalone", CORE_PATH)
if CORE_SPEC is None or CORE_SPEC.loader is None:
    raise RuntimeError(f"cannot load X2 control core: {CORE_PATH}")
CORE = importlib.util.module_from_spec(CORE_SPEC)
sys.modules[CORE_SPEC.name] = CORE
CORE_SPEC.loader.exec_module(CORE)
AdaptiveHopfParams = CORE.AdaptiveHopfParams
AdaptiveHopfSnapshot = CORE.AdaptiveHopfSnapshot
HopfMode = CORE.HopfMode
TwoLegAdaptiveHopf = CORE.TwoLegAdaptiveHopf


SCHEMA = "sceneforge/x2-hopf-trace-audit/v1"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, 0o444)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def complete(row: dict[str, str]) -> bool:
    required = [
        "policy_dt", "current_gait", "freeze", "hopf_period", "hopf_mu",
        "hopf_vx", "hopf_vy", "hopf_wz", "cmd_y", "cmd_yaw",
        *(f"obs_phase_{index}" for index in range(4)),
    ]
    try:
        values = [float(row[name]) for name in required]
    except (KeyError, TypeError, ValueError):
        return False
    return all(math.isfinite(value) for value in values)


def percentile(values: list[float], quantile: float) -> float:
    ordered = sorted(values)
    index = max(0, math.ceil(quantile * len(ordered)) - 1)
    return ordered[index]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug-csv", required=True)
    parser.add_argument("--receipt", required=True)
    parser.add_argument("--minimum-transitions", type=int, default=1000)
    parser.add_argument("--maximum-state-error", type=float, default=2e-6)
    parser.add_argument("--maximum-period-error", type=float, default=1.1e-6)
    args = parser.parse_args()

    debug_csv = Path(args.debug_csv).resolve()
    receipt = Path(args.receipt).resolve()
    if not debug_csv.is_file():
        raise FileNotFoundError(debug_csv)
    if receipt.exists():
        raise FileExistsError(f"refusing to overwrite receipt: {receipt}")
    with debug_csv.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    trailing_partial_rows_skipped = 0
    if rows and not complete(rows[-1]):
        rows.pop()
        trailing_partial_rows_skipped = 1
    if any(not complete(row) for row in rows):
        raise ValueError("vendor trace contains a malformed non-terminal row")

    params = AdaptiveHopfParams(
        x2_style_adapter=True,
        gait_blend_tau=0.08,
        reset_on_walk_entry=False,
    )
    state_errors: list[float] = []
    period_errors: list[float] = []
    mu_errors: list[float] = []
    transition_count = 0
    skipped_entry_transitions = 0
    nonzero_lateral_or_yaw_command_transitions = 0
    filtered_command_y = 0.0
    filtered_command_yaw = 0.0
    started = time.monotonic()
    for index, (current, following) in enumerate(zip(rows, rows[1:])):
        dt = float(current["policy_dt"])
        command_alpha = dt / (params.cmd_tau + dt)
        filtered_command_y += command_alpha * (
            float(current["cmd_y"]) - filtered_command_y
        )
        filtered_command_yaw += command_alpha * (
            float(current["cmd_yaw"]) - filtered_command_yaw
        )
        if current["freeze"] != "0" or following["freeze"] != "0":
            continue
        if index == 0 or rows[index - 1]["freeze"] != "0":
            # The first active observation is deliberately blended from the
            # fixed stand vector; it is not the oscillator's internal state.
            skipped_entry_transitions += 1
            continue
        if abs(float(current["cmd_y"])) > 1e-9 or abs(float(current["cmd_yaw"])) > 1e-9:
            nonzero_lateral_or_yaw_command_transitions += 1
        hopf = TwoLegAdaptiveHopf(params)
        hopf.restore(
            AdaptiveHopfSnapshot(
                mode=HopfMode.WALK,
                gait=int(current["current_gait"]),
                walk_initialized=True,
                period=float(current["hopf_period"]),
                obs4=tuple(float(current[f"obs_phase_{i}"]) for i in range(4)),
                filtered_velocity=(
                    float(current["hopf_vx"]),
                    float(current["hopf_vy"]),
                    float(current["hopf_wz"]),
                ),
                filtered_command=(filtered_command_y, filtered_command_yaw),
            )
        )
        mu_errors.append(abs(hopf.current_mu() - float(current["hopf_mu"])))
        hopf.step(dt)
        expected = tuple(float(following[f"obs_phase_{i}"]) for i in range(4))
        state_errors.extend(abs(actual - wanted) for actual, wanted in zip(hopf.obs4(), expected))
        period_errors.append(abs(hopf.period - float(following["hopf_period"])))
        transition_count += 1

    if not state_errors or not period_errors or not mu_errors:
        raise ValueError("no eligible active Hopf transitions were found")
    metrics = {
        "transition_count": transition_count,
        "state_value_count": len(state_errors),
        "maximum_state_absolute_error": max(state_errors),
        "mean_state_absolute_error": sum(state_errors) / len(state_errors),
        "p95_state_absolute_error": percentile(state_errors, 0.95),
        "maximum_period_absolute_error": max(period_errors),
        "maximum_mu_absolute_error": max(mu_errors),
    }
    passed = (
        transition_count >= args.minimum_transitions
        and metrics["maximum_state_absolute_error"] <= args.maximum_state_error
        and metrics["maximum_period_absolute_error"] <= args.maximum_period_error
    )
    payload = {
        "schema": SCHEMA,
        "status": "PASSED" if passed else "FAILED",
        "decision": (
            "engine_neutral_hopf_single_step_dynamics_reproduced"
            if passed
            else "hopf_dynamics_not_reproduced"
        ),
        "offline_only": True,
        "read_only_inputs": True,
        "published_topics": [],
        "debug_csv": {
            "path": str(debug_csv),
            "size": debug_csv.stat().st_size,
            "sha256": sha256(debug_csv),
            "complete_row_count": len(rows),
            "trailing_partial_rows_skipped": trailing_partial_rows_skipped,
            "recorded_decimal_places": 6,
        },
        "implementation": {
            "path": str(CORE_PATH),
            "size": CORE_PATH.stat().st_size,
            "sha256": sha256(CORE_PATH),
        },
        "parameters": params.__dict__,
        "metrics": metrics,
        "skipped": {
            "stand_to_walk_blended_entry_transitions": skipped_entry_transitions,
            "nonzero_lateral_or_yaw_command_transitions_exercised": (
                nonzero_lateral_or_yaw_command_transitions
            ),
        },
        "thresholds": {
            "minimum_transitions": args.minimum_transitions,
            "maximum_state_absolute_error": args.maximum_state_error,
            "maximum_period_absolute_error": args.maximum_period_error,
        },
        "limitations": [
            "The lateral/yaw command low-pass state is reconstructed from six-decimal command "
            "samples rather than captured at full precision.",
            "The stand-to-walk observation blend is outside the oscillator and is not part of "
            "this single-step Hopf gate.",
            "The vendor CSV is quantized to six decimal places; this is not bit-exact parity.",
        ],
        "runtime": {"elapsed_wall_seconds": time.monotonic() - started},
        "qualification": False,
        "real_robot_commands": False,
        "finished_at_unix": time.time(),
    }
    atomic_json(receipt, payload)
    print(receipt)
    return 0 if passed else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except Exception as exc:
        receipt_arg = None
        if "--receipt" in os.sys.argv:
            index = os.sys.argv.index("--receipt") + 1
            if index < len(os.sys.argv):
                receipt_arg = Path(os.sys.argv[index]).resolve()
        if receipt_arg is not None and not receipt_arg.exists():
            atomic_json(
                receipt_arg,
                {
                    "schema": SCHEMA,
                    "status": "FAILED",
                    "decision": "hopf_trace_audit_failed_closed",
                    "error": {"type": type(exc).__name__, "message": str(exc)},
                    "offline_only": True,
                    "published_topics": [],
                    "qualification": False,
                    "real_robot_commands": False,
                    "finished_at_unix": time.time(),
                },
            )
        raise
