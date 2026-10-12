#!/usr/bin/env python3
"""Validate the state-driven X2 active-walk action/target pipeline.

This is an offline, read-only audit.  It reads the immutable six-decimal vendor
debug trace and the exact run configuration; it creates no ROS publisher and
does not advance any physics engine.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
import sys
import tempfile
import time
from dataclasses import asdict
from pathlib import Path


CORE_PATH = Path(__file__).resolve().parents[2] / "src" / "sceneforge" / "x2_control.py"
CORE_SPEC = importlib.util.spec_from_file_location("sceneforge_x2_control_standalone", CORE_PATH)
if CORE_SPEC is None or CORE_SPEC.loader is None:
    raise RuntimeError(f"cannot load X2 control core: {CORE_PATH}")
CORE = importlib.util.module_from_spec(CORE_SPEC)
sys.modules[CORE_SPEC.name] = CORE
CORE_SPEC.loader.exec_module(CORE)
CpgWalkActionParams = CORE.CpgWalkActionParams
CpgWalkActionPipeline = CORE.CpgWalkActionPipeline
CpgWalkActionSnapshot = CORE.CpgWalkActionSnapshot
X2_CPGWALK_JOINT_NAMES = CORE.X2_CPGWALK_JOINT_NAMES


SCHEMA = "sceneforge/x2-cpgwalk-action-pipeline-audit/v1"
WIDTH = len(X2_CPGWALK_JOINT_NAMES)


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


def finite_float(row: dict[str, str], name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {name}")
    return value


def complete(row: dict[str, str]) -> bool:
    required = ["policy_dt", "current_gait", "is_standing"]
    for stem in ("action", "residual", "q_nominal", "q_target", "q_filtered"):
        required.extend(f"{stem}_{index}" for index in range(WIDTH))
    try:
        values = [float(row[name]) for name in required]
    except (KeyError, TypeError, ValueError):
        return False
    return all(math.isfinite(value) for value in values)


def vector(row: dict[str, str], stem: str) -> tuple[float, ...]:
    return tuple(finite_float(row, f"{stem}_{index}") for index in range(WIDTH))


def maximum_absolute_error(actual: tuple[float, ...], expected: tuple[float, ...]) -> float:
    if len(actual) != len(expected):
        raise ValueError("vector width mismatch")
    return max(abs(left - right) for left, right in zip(actual, expected))


def params_from_config(config: dict) -> CpgWalkActionParams:
    base = config["BaseConfig"]
    actor = config["ActorCriticConfig"]
    cpg = config["CPGWalkConfig"]
    walk = cpg["cpgwalk"]
    filter_config = cpg["filter"]
    if base["model_type"] != "CPGWalk":
        raise ValueError("configuration is not CPGWalk")
    if actor["planner"]["enable"]:
        raise ValueError("planner-enabled traces need a separate target gate")
    if not walk["enable_cpg"] or walk["cpg_profile"] != "phase_only":
        raise ValueError("only the deployed phase_only CPG profile is supported")
    return CpgWalkActionParams(
        joint_names=tuple(base["action_seq"]),
        action_scale=tuple(float(value) for value in base["action_scale"]),
        action_cutoff_hz=tuple(
            float(value) for value in filter_config["action_cutoff_freq"][:WIDTH]
        ),
        nominal_position=tuple(float(value) for value in actor["robot"]["default_dof_pos"]),
        kp=tuple(float(value) for value in actor["robot"]["kps"]),
        kd=tuple(float(value) for value in actor["robot"]["kds"]),
        action_clip=float(actor["clips"]["action_clip"]),
        waist_pitch_residual_scale=float(walk["waist_pitch_residual_scale"]),
        waist_roll_residual_scale=float(walk["waist_roll_residual_scale"]),
        waist_pitch_residual_max_abs=float(walk["waist_pitch_residual_max_abs"]),
        waist_roll_residual_max_abs=float(walk["waist_roll_residual_max_abs"]),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug-csv", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--receipt", required=True)
    parser.add_argument("--minimum-transitions", type=int, default=1100)
    parser.add_argument("--maximum-error", type=float, default=2e-6)
    args = parser.parse_args()

    debug_csv = Path(args.debug_csv).resolve()
    config_path = Path(args.config).resolve()
    receipt = Path(args.receipt).resolve()
    if not debug_csv.is_file():
        raise FileNotFoundError(debug_csv)
    if not config_path.is_file():
        raise FileNotFoundError(config_path)
    if receipt.exists():
        raise FileExistsError(f"refusing to overwrite receipt: {receipt}")

    import yaml

    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    params = params_from_config(config)
    params.validate()
    with debug_csv.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    trailing_partial_rows_skipped = 0
    if rows and not complete(rows[-1]):
        rows.pop()
        trailing_partial_rows_skipped = 1
    if any(not complete(row) for row in rows):
        raise ValueError("vendor trace contains a malformed non-terminal row")

    residual_errors: list[float] = []
    nominal_errors: list[float] = []
    target_errors: list[float] = []
    filtered_errors: list[float] = []
    active_transition_count = 0
    gait_transition_count = 0
    lateral_or_yaw_transition_count = 0
    started = time.monotonic()
    for current, following in zip(rows, rows[1:]):
        if int(finite_float(current, "is_standing")) != 0:
            continue
        if int(finite_float(following, "is_standing")) != 0:
            continue
        pipeline = CpgWalkActionPipeline(params)
        pipeline.restore(
            CpgWalkActionSnapshot(
                residual=vector(current, "residual"),
                filtered_position_target=vector(current, "q_filtered"),
            )
        )
        output = pipeline.step_active(
            vector(following, "action"), finite_float(following, "policy_dt")
        )
        residual_errors.append(maximum_absolute_error(output.residual, vector(following, "residual")))
        nominal_errors.append(
            maximum_absolute_error(output.nominal_position, vector(following, "q_nominal"))
        )
        target_errors.append(
            maximum_absolute_error(
                output.position_target_unfiltered, vector(following, "q_target")
            )
        )
        filtered_errors.append(
            maximum_absolute_error(output.position_target, vector(following, "q_filtered"))
        )
        if int(finite_float(current, "current_gait")) != int(
            finite_float(following, "current_gait")
        ):
            gait_transition_count += 1
        if abs(finite_float(following, "cmd_y")) > 1e-9 or abs(
            finite_float(following, "cmd_yaw")
        ) > 1e-9:
            lateral_or_yaw_transition_count += 1
        active_transition_count += 1

    if not residual_errors:
        raise ValueError("no active locomotion transitions found")
    metrics = {
        "active_transition_count": active_transition_count,
        "maximum_residual_absolute_error": max(residual_errors),
        "maximum_nominal_absolute_error": max(nominal_errors),
        "maximum_unfiltered_target_absolute_error": max(target_errors),
        "maximum_filtered_target_absolute_error": max(filtered_errors),
        "gait_transition_count": gait_transition_count,
        "lateral_or_yaw_transition_count": lateral_or_yaw_transition_count,
    }
    passed = active_transition_count >= args.minimum_transitions and all(
        metrics[name] <= args.maximum_error
        for name in (
            "maximum_residual_absolute_error",
            "maximum_nominal_absolute_error",
            "maximum_unfiltered_target_absolute_error",
            "maximum_filtered_target_absolute_error",
        )
    )
    payload = {
        "schema": SCHEMA,
        "status": "PASSED" if passed else "FAILED",
        "decision": (
            "active_locomotion_action_target_filter_reproduced"
            if passed
            else "active_locomotion_action_target_filter_not_reproduced"
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
        "config": {
            "path": str(config_path),
            "size": config_path.stat().st_size,
            "sha256": sha256(config_path),
        },
        "implementation": {
            "path": str(CORE_PATH),
            "size": CORE_PATH.stat().st_size,
            "sha256": sha256(CORE_PATH),
        },
        "parameters": asdict(params),
        "metrics": metrics,
        "position_pd_output_contract": {
            "joint_names": list(params.joint_names),
            "position_target_source": "filtered_active_locomotion_target",
            "velocity_target": "zero",
            "feedforward_effort": "zero_in_independent_adapter",
            "kp_source": "ActorCriticConfig.robot.kps",
            "kd_source": "ActorCriticConfig.robot.kds",
            "vendor_joint_command_numeric_parity": "not_yet_measured",
        },
        "thresholds": {
            "minimum_active_transitions": args.minimum_transitions,
            "maximum_absolute_error": args.maximum_error,
        },
        "limitations": [
            "The source CSV is quantized to six decimal places; this is not bit-exact parity.",
            "Only consecutive active-locomotion transitions are accepted by this gate.",
            "Stand entry, freeze, near-stand planning and auto-transition remain outside this gate.",
            "The position target and gain vectors are reconstructed, but numeric vendor JointCommand "
            "velocity/effort/gain lanes have not yet been captured for direct parity.",
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
        if "--receipt" in sys.argv:
            index = sys.argv.index("--receipt") + 1
            if index < len(sys.argv):
                receipt_arg = Path(sys.argv[index]).resolve()
        if receipt_arg is not None and not receipt_arg.exists():
            atomic_json(
                receipt_arg,
                {
                    "schema": SCHEMA,
                    "status": "FAILED",
                    "decision": "action_pipeline_audit_failed_closed",
                    "error": {"type": type(exc).__name__, "message": str(exc)},
                    "offline_only": True,
                    "published_topics": [],
                    "qualification": False,
                    "real_robot_commands": False,
                    "finished_at_unix": time.time(),
                },
            )
        raise
