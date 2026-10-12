#!/usr/bin/env python3
"""Convert one frozen X2 combination lifecycle into a 30 fps HUD trace.

This is a read-only presentation transform.  It performs URDF forward
kinematics from the recorded PhysX state, aligns contact and elastic-band
telemetry, and adds a versioned per-frame overlay.  It does not start a physics
engine and does not publish any robot command.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path, PurePosixPath
import sys
from typing import Any, Mapping, Sequence

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
_FK_PATH = ROOT / "admin/spark/convert-x2-gait-trace-to-render-state.py"
_SPEC = importlib.util.spec_from_file_location("sceneforge_x2_render_fk", _FK_PATH)
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError(f"cannot load FK converter: {_FK_PATH}")
FK = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(FK)


TRACE_SCHEMA = "sceneforge/x2-geniesim-render-state/v1"
RECEIPT_SCHEMA = "sceneforge/x2-geniesim-render-state-receipt/v1"
OVERLAY_SCHEMA = "sceneforge/x2-combination-video-overlay/v1"
PHYSICS_AUTHORITY = "genie_sim_isaac_physx"
SCENARIOS = ("straight", "turn_left", "turn_right")
JOINT_COUNT = 31
SOURCE_SCHEMAS = {
    "runtime": "sceneforge/x2-physx-cold-start-combination-runtime/v1",
    "trace": "sceneforge/x2-physx-cold-start-state-command-trace/v1",
    "contacts": "sceneforge/x2-physx-cold-start-contact-trace/v1",
    "lifecycle": "sceneforge/x2-physx-container-lifecycle-receipt/v1",
    "runtime_manifest": "sceneforge/x2-runtime-manifest/v1",
}
BAND_SCHEMA = "sceneforge/physx-elastic-band-status/v1"
FROZEN_INDEX_SCHEMA = "sceneforge/x2-frozen-runtime-index/v1"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def reject_nonfinite(value: str) -> None:
    raise ValueError(f"non-finite JSON constant is forbidden: {value}")


def load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(
        path.read_text(encoding="utf-8"), parse_constant=reject_nonfinite
    )
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root is not an object: {path}")
    return payload


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            row = json.loads(line, parse_constant=reject_nonfinite)
            if not isinstance(row, dict):
                raise ValueError(f"{path}:{line_number}: record is not an object")
            rows.append(row)
    if not rows:
        raise ValueError(f"JSONL has no records: {path}")
    return rows


def finite_vector(value: Any, length: int, name: str) -> tuple[float, ...]:
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain {length} numbers") from exc
    if len(result) != length or not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain {length} finite numbers")
    return result


def atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            stream.write(
                json.dumps(payload, indent=2, sort_keys=True, allow_nan=False)
                + "\n"
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def validate_frozen_runtime(run_dir: Path) -> dict[str, Any]:
    """Verify the complete immutable raw runtime set before presentation."""

    index_path = run_dir / "runtime-frozen-index.json"
    index = load_json(index_path)
    if (
        index.get("schema") != FROZEN_INDEX_SCHEMA
        or index.get("derivation_root") != "derived"
        or index.get("qualification") is not False
        or index.get("real_robot_commands") is not False
    ):
        raise ValueError("runtime frozen index identity or safety flag mismatch")
    rows = index.get("files")
    if not isinstance(rows, list) or index.get("file_count") != len(rows):
        raise ValueError("runtime frozen index count mismatch")
    indexed: set[str] = set()
    for row_index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ValueError(f"runtime frozen index row {row_index} is invalid")
        raw_name = row.get("path")
        if not isinstance(raw_name, str):
            raise ValueError(f"runtime frozen index row {row_index} has no path")
        relative = PurePosixPath(raw_name)
        if (
            relative.is_absolute()
            or not relative.parts
            or any(part in {"", ".", ".."} for part in relative.parts)
            or relative.parts[0] == "derived"
            or raw_name == "runtime-frozen-index.json"
            or raw_name in indexed
        ):
            raise ValueError(f"unsafe or duplicate frozen path: {raw_name!r}")
        indexed.add(raw_name)
        path = run_dir.joinpath(*relative.parts)
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"frozen runtime file is missing or a symlink: {raw_name}")
        if row.get("size") != path.stat().st_size or row.get("sha256") != sha256(path):
            raise ValueError(f"frozen runtime file hash mismatch: {raw_name}")
    actual = {
        path.relative_to(run_dir).as_posix()
        for path in run_dir.rglob("*")
        if path.is_file()
        and path.relative_to(run_dir).parts[0] != "derived"
        and path.name != "runtime-frozen-index.json"
    }
    if actual != indexed:
        missing = sorted(actual - indexed)
        stale = sorted(indexed - actual)
        raise ValueError(
            f"runtime frozen file set mismatch: unindexed={missing}, stale={stale}"
        )
    return index


def _compressed_scenarios(rows: Sequence[Mapping[str, Any]]) -> tuple[str, ...]:
    result = []
    for row in rows:
        scenario = row.get("scenario")
        if scenario not in SCENARIOS:
            raise ValueError(f"unknown scenario in trace: {scenario!r}")
        if not result or result[-1] != scenario:
            result.append(str(scenario))
    return tuple(result)


def _nearest_index(times: Sequence[float], target: float) -> int:
    right = bisect.bisect_left(times, target)
    if right <= 0:
        return 0
    if right >= len(times):
        return len(times) - 1
    return right - 1 if target - times[right - 1] <= times[right] - target else right


def _previous_index(times: Sequence[float], target: float) -> int:
    index = bisect.bisect_right(times, target) - 1
    if index < 0:
        raise ValueError(f"no telemetry sample at or before sim_time={target}")
    return index


def _validate_source(
    run_dir: Path,
) -> tuple[
    dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any],
    list[dict[str, Any]], dict[str, Any],
]:
    runtime = load_json(run_dir / "combination-runtime-result.json")
    trace = load_json(run_dir / "state-command-trace.json")
    contacts = load_json(run_dir / "contact-trace.json")
    lifecycle = load_json(run_dir / "lifecycle-receipt.json")
    band = load_jsonl(run_dir / "elastic-band-telemetry.jsonl")
    manifest = load_json(run_dir / "runtime-manifest.json")
    for label, payload in (
        ("runtime", runtime), ("trace", trace), ("contacts", contacts),
        ("lifecycle", lifecycle), ("runtime_manifest", manifest),
    ):
        if payload.get("schema") != SOURCE_SCHEMAS[label]:
            raise ValueError(f"{label} schema mismatch")
        if payload.get("real_robot_commands") is not False:
            raise ValueError(f"{label} is not real_robot_commands=false")
        if payload.get("qualification") not in {None, False}:
            raise ValueError(f"{label} was relabelled as qualification")
    if (
        runtime.get("status") != "PASSED"
        or runtime.get("physics_authority") != PHYSICS_AUTHORITY
        or runtime.get("single_physics_lifecycle") is not True
        or runtime.get("case_order") != list(SCENARIOS)
        or runtime.get("completed_case_count") != len(SCENARIOS)
    ):
        raise ValueError("runtime is not the passed single-PhysX combination")
    runtime_cases = runtime.get("cases")
    if (
        not isinstance(runtime_cases, list)
        or [item.get("scenario") for item in runtime_cases] != list(SCENARIOS)
        or any(item.get("status") != "PASSED" for item in runtime_cases)
    ):
        raise ValueError("runtime case evidence is incomplete or failed")
    if trace.get("physics_authority") != PHYSICS_AUTHORITY:
        raise ValueError("state trace authority mismatch")
    if contacts.get("physics_authority") != PHYSICS_AUTHORITY:
        raise ValueError("contact trace authority mismatch")
    if lifecycle.get("physics_authority_process_count") != 1:
        raise ValueError("lifecycle does not prove one physics authority")
    if lifecycle.get("container_exit_code") != 0 or lifecycle.get("teardown_complete") is not True:
        raise ValueError("source lifecycle did not stop cleanly")
    if manifest.get("single_physics_lifecycle") is not True:
        raise ValueError("runtime manifest does not seal one lifecycle")
    if manifest.get("case_order") != list(SCENARIOS):
        raise ValueError("runtime manifest case order mismatch")
    for index, row in enumerate(band):
        if row.get("schema") != BAND_SCHEMA or row.get("record_type") != "sample":
            raise ValueError(f"elastic-band sample {index} schema mismatch")
        if row.get("real_robot_commands") is not False:
            raise ValueError(
                f"elastic-band sample {index} is not real_robot_commands=false"
            )
        if row.get("state") not in {"ENGAGED", "RAMP_DOWN", "RELEASED"}:
            raise ValueError(f"elastic-band sample {index} state is invalid")
        sim_time = float(row.get("sim_time_s", math.nan))
        support = float(row.get("support_ratio", math.nan))
        force = float(row.get("force_norm_n", math.nan))
        if (
            not all(math.isfinite(value) for value in (sim_time, support, force))
            or not 0.0 <= support <= 1.0
            or force < 0.0
        ):
            raise ValueError(f"elastic-band sample {index} values are invalid")
    return runtime, trace, contacts, lifecycle, band, manifest


def build_render_state(
    *,
    run_dir: Path,
    urdf: Path,
    manifest_path: Path,
    output: Path,
    receipt_path: Path,
    fps: float = 30.0,
) -> dict[str, Any]:
    run_dir, urdf, manifest_path = (
        run_dir.resolve(), urdf.resolve(), manifest_path.resolve()
    )
    output, receipt_path = output.resolve(), receipt_path.resolve()
    if not math.isfinite(fps) or fps != 30.0:
        raise ValueError("qualification replay is fixed at exactly 30 fps")
    if output.exists() or receipt_path.exists():
        raise FileExistsError("refusing to overwrite render-state evidence")
    validate_frozen_runtime(run_dir)
    runtime, trace, contacts, lifecycle, band_rows, runtime_manifest = (
        _validate_source(run_dir)
    )
    scene_manifest = load_json(manifest_path)
    physics = trace.get("physics_samples")
    bases = trace.get("base_samples")
    commands = trace.get("commands")
    joint_names = trace.get("joint_names")
    contact_rows = contacts.get("samples")
    if not isinstance(joint_names, list) or len(joint_names) != JOINT_COUNT:
        raise ValueError("combined trace must contain exactly 31 joints")
    if not all(isinstance(value, list) and value for value in (physics, bases, commands, contact_rows)):
        raise ValueError("combined state, command and contact samples are required")
    if len(physics) != len(bases) or len(commands) != len(physics):
        raise ValueError("one state and one command per physics sample are required")
    if _compressed_scenarios(bases) != SCENARIOS:
        raise ValueError("scenario samples are not contiguous straight->left->right")
    for scenario in SCENARIOS:
        phases = {
            row.get("phase") for row in bases if row.get("scenario") == scenario
        }
        if "GAIT1_FORWARD" not in phases or "STOP_STAND" not in phases:
            raise ValueError(f"scenario {scenario} lacks gait or stable-stop samples")

    source_times = [float(row["sim_time"]) for row in bases]
    if not all(
        math.isfinite(value) and (index == 0 or value > source_times[index - 1])
        for index, value in enumerate(source_times)
    ):
        raise ValueError("base sim_time is not finite and strictly increasing")
    for index, (state, base, command) in enumerate(zip(physics, bases, commands)):
        if (
            state.get("scenario") != base.get("scenario")
            or command.get("scenario") != base.get("scenario")
        ):
            raise ValueError(f"scenario alignment changed at sample {index}")
        if len(state.get("position", [])) != JOINT_COUNT:
            raise ValueError(f"physics sample {index} lacks 31 positions")
        if len(command.get("articulation_position_target", [])) != JOINT_COUNT:
            raise ValueError(f"command sample {index} lacks 31 targets")

    contact_times = [float(row["sim_time"]) for row in contact_rows]
    band_times = [float(row["sim_time_s"]) for row in band_rows]
    if contact_times != sorted(contact_times) or band_times != sorted(band_times):
        raise ValueError("contact or elastic-band telemetry is not time ordered")
    physics_hz = float(runtime_manifest.get("physics_hz", 0.0))
    if not math.isfinite(physics_hz) or physics_hz <= 0.0:
        raise ValueError("runtime physics_hz is invalid")
    engine = lifecycle.get("engine_process")
    if not isinstance(engine, dict):
        raise ValueError("engine lifecycle process is missing")
    wall_duration = float(engine["stopped_at_unix"]) - float(engine["started_at_unix"])
    sim_duration = source_times[-1] - source_times[0]
    if not math.isfinite(wall_duration) or wall_duration <= 0.0 or sim_duration <= 0.0:
        raise ValueError("runtime duration cannot produce an RTF estimate")
    rtf_estimate = sim_duration / wall_duration

    links, children, movable = FK.load_urdf(urdf)
    missing_joints = sorted(set(movable) - set(joint_names))
    root_pose = scene_manifest.get("robot_root_pose")
    if not isinstance(root_pose, dict):
        raise ValueError("scene manifest has no robot_root_pose")
    root = FK.transform(
        (root_pose["x"], root_pose["y"], root_pose["z"]),
        (0.0, 0.0, root_pose["theta"]),
    )
    inverse_root = np.linalg.inv(root)
    path_by_link = {"base_link": "/x2/Geometry/base_link"}
    ordered_links = ["base_link"]

    def discover(parent: str) -> None:
        for joint in children.get(parent, []):
            path_by_link[joint["child"]] = (
                path_by_link[parent] + "/" + joint["child"]
            )
            ordered_links.append(joint["child"])
            discover(joint["child"])

    discover("base_link")

    frame_count = int(math.floor(sim_duration * fps)) + 1
    targets = [source_times[0] + index / fps for index in range(frame_count)]
    source_indices = [_nearest_index(source_times, target) for target in targets]
    maximum_contact_age = max(0.01, 3.0 / physics_hz)
    partial = output.with_name(output.name + ".partial")
    output.parent.mkdir(parents=True, exist_ok=True)
    if partial.exists():
        raise FileExistsError(partial)

    header = {
        "record_type": "schema",
        "schema": TRACE_SCHEMA,
        "authoritative_physics_source": "geniesim_isaac_physx",
        "source_kind": "single_lifecycle_straight_left_right_combination",
        "base_topic": "offline_authoritative_combination_trace",
        "tf_topic": "offline_urdf_fk",
        "sample_clock": "simulation_time",
        "target_sample_rate_hz": fps,
        "case_order": list(SCENARIOS),
        "overlay_schema": OVERLAY_SCHEMA,
        "overlay_required": True,
        "read_only": True,
        "published_topics": [],
        "real_robot_commands": False,
    }
    try:
        with partial.open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(header, sort_keys=True, allow_nan=False) + "\n")
            for sequence, (target, source_index) in enumerate(
                zip(targets, source_indices)
            ):
                base = bases[source_index]
                state = physics[source_index]
                scenario = str(base["scenario"])
                state_time = float(base["sim_time"])
                contact_index = _nearest_index(contact_times, state_time)
                contact = contact_rows[contact_index]
                if abs(contact_times[contact_index] - state_time) > maximum_contact_age:
                    raise ValueError(
                        f"contact evidence too old at sim_time={state_time}"
                    )
                if contact.get("scenario") != scenario:
                    raise ValueError("nearest contact crosses a scenario boundary")
                band_index = _previous_index(band_times, state_time)
                band = band_rows[band_index]
                if state_time - band_times[band_index] > 0.25:
                    raise ValueError("elastic-band telemetry is older than 250 ms")
                feet = contact.get("feet")
                if not isinstance(feet, dict):
                    raise ValueError("contact sample has no feet object")
                left = feet.get("left_ankle_roll_link")
                right = feet.get("right_ankle_roll_link")
                if not isinstance(left, dict) or not isinstance(right, dict):
                    raise ValueError("both foot contact records are required")
                if not isinstance(left.get("contact"), bool) or not isinstance(
                    right.get("contact"), bool
                ):
                    raise ValueError("foot contact states must be explicit booleans")

                qmap = dict(zip(joint_names, state["position"]))
                world = FK.quat_matrix_wxyz(base["orientation_wxyz"])
                world[:3, 3] = base["position_m"]
                base_local = inverse_root @ world
                transforms = []

                def add(link: str, matrix: np.ndarray) -> None:
                    transforms.append({
                        "child_path": path_by_link[link],
                        "parent_path": "map",
                        "translation_xyz_m": matrix[:3, 3].tolist(),
                        "rotation_xyzw": FK.matrix_quat_xyzw(matrix),
                    })

                add("base_link", base_local)

                def emit(parent: str) -> None:
                    for joint in children.get(parent, []):
                        matrix = joint["origin"]
                        if joint["type"] in ("revolute", "continuous"):
                            matrix = matrix @ FK.axis_motion(
                                joint["axis"], float(qmap.get(joint["name"], 0.0))
                            )
                        add(joint["child"], matrix)
                        emit(joint["child"])

                emit("base_link")
                measured_linear = finite_vector(
                    base.get("base_linear_velocity_body_mps"), 3,
                    "base_linear_velocity_body_mps",
                )
                measured_angular = finite_vector(
                    base.get("base_angular_velocity_body_rps"), 3,
                    "base_angular_velocity_body_rps",
                )
                overlay = {
                    "schema": OVERLAY_SCHEMA,
                    "scenario": scenario,
                    "phase": str(base["phase"]),
                    "active_gait": str(base.get("active_gait", "unknown")),
                    "requested_velocity": list(
                        finite_vector(base.get("requested_velocity"), 3, "requested_velocity")
                    ),
                    "effective_velocity_command": list(
                        finite_vector(
                            base.get("effective_velocity_command"), 3,
                            "effective_velocity_command",
                        )
                    ),
                    "measured_body_linear_velocity": list(measured_linear),
                    "measured_body_angular_velocity": list(measured_angular),
                    "linear_speed_mps": float(base["linear_speed_mps"]),
                    "angular_speed_rps": float(base["angular_speed_rad_s"]),
                    "left_foot_contact": left["contact"],
                    "right_foot_contact": right["contact"],
                    "cpg_phase_rad": float(base["cpg_phase_rad"]),
                    "cpg_period_s": float(base["cpg_period_s"]),
                    "suspension_state": str(band["state"]),
                    "suspension_support_ratio": float(band["support_ratio"]),
                    "suspension_force_norm_n": float(band["force_norm_n"]),
                    "sim_time_s": float(target),
                    "rtf_estimate": rtf_estimate,
                    "rtf_method": "global_lifecycle",
                    "real_robot_commands": False,
                }
                # Reject NaN/Inf before the record can be exposed.
                json.dumps(overlay, allow_nan=False)
                record = {
                    "record_type": "sample",
                    "schema": TRACE_SCHEMA,
                    "sequence": sequence,
                    "sim_time_ns": int(round(target * 1e9)),
                    "sim_time_s": float(target),
                    "base_position_xyz_m": list(map(float, base["position_m"])),
                    "base_orientation_xyzw": [
                        float(base["orientation_wxyz"][1]),
                        float(base["orientation_wxyz"][2]),
                        float(base["orientation_wxyz"][3]),
                        float(base["orientation_wxyz"][0]),
                    ],
                    "transforms": transforms,
                    "overlay": overlay,
                }
                stream.write(
                    json.dumps(record, separators=(",", ":"), allow_nan=False)
                    + "\n"
                )
            summary = {
                "record_type": "summary",
                "schema": TRACE_SCHEMA,
                "status": "SUCCEEDED",
                "sample_count": frame_count,
                "first_sim_time_s": targets[0],
                "last_sim_time_s": targets[-1],
                "case_order": list(SCENARIOS),
                "overlay_schema": OVERLAY_SCHEMA,
                "overlay_sample_count": frame_count,
                "read_only": True,
                "real_robot_commands": False,
            }
            stream.write(json.dumps(summary, sort_keys=True, allow_nan=False) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(partial, output)
    finally:
        partial.unlink(missing_ok=True)

    receipt = {
        "schema": RECEIPT_SCHEMA,
        "status": "SUCCEEDED",
        "authoritative_physics_source": "geniesim_isaac_physx",
        "source_kind": "single_lifecycle_straight_left_right_combination",
        "source_result_sha256": sha256(run_dir / "combination-runtime-result.json"),
        "source_trace_sha256": sha256(run_dir / "state-command-trace.json"),
        "source_contact_trace_sha256": sha256(run_dir / "contact-trace.json"),
        "source_elastic_band_telemetry_sha256": sha256(
            run_dir / "elastic-band-telemetry.jsonl"
        ),
        "source_lifecycle_sha256": sha256(run_dir / "lifecycle-receipt.json"),
        "source_frozen_index_sha256": sha256(
            run_dir / "runtime-frozen-index.json"
        ),
        "runtime_manifest_sha256": sha256(run_dir / "runtime-manifest.json"),
        "urdf_sha256": sha256(urdf),
        "manifest_sha256": sha256(manifest_path),
        "output_sha256": sha256(output),
        "output_size": output.stat().st_size,
        "sample_count": frame_count,
        "link_count": len(ordered_links),
        "case_order": list(SCENARIOS),
        "source_start_s": targets[0],
        "source_end_s": targets[-1],
        "target_sample_rate_hz": fps,
        "overlay_schema": OVERLAY_SCHEMA,
        "overlay_sample_count": frame_count,
        "rtf": {
            "value": rtf_estimate,
            "method": "global_sim_span_divided_by_engine_process_wall_lifetime",
            "sim_duration_s": sim_duration,
            "wall_duration_s": wall_duration,
            "limitation": "not an instantaneous per-frame RTF measurement",
        },
        "defaulted_joint_names": missing_joints,
        "defaulted_joint_value_rad": 0.0,
        "method": "offline_urdf_forward_kinematics_from_authoritative_combination_physx_state",
        "read_only": True,
        "published_topics": [],
        "mujoco_state_source_used": False,
        "qualification": False,
        "real_robot_commands": False,
    }
    atomic_json(receipt_path, receipt)
    output.chmod(0o444)
    receipt_path.chmod(0o444)
    return receipt


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=30.0)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    receipt = build_render_state(
        run_dir=args.run_dir,
        urdf=args.urdf,
        manifest_path=args.manifest,
        output=args.output,
        receipt_path=args.receipt,
        fps=args.fps,
    )
    print(json.dumps(receipt, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
