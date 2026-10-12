"""Finalize one X2 straight/left/right PhysX lifecycle for later sealing.

The runtime runner deliberately stops before qualification.  This module is a
read-only consumer of that frozen runtime directory: it stages the three
MuJoCo references, invokes the v2 A/B comparator, derives aggregate facts from
authoritative traces, and emits a versioned sealing input.  It never writes a
``result.json`` and never sets ``qualification=true``.

Missing evidence is distinct from an observed failure.  Legacy v1 episode
evidence still requires per-episode media.  V2 episode evidence deliberately
does not: one reviewed canonical media pair is enforced later by the v2 suite.
An explicit unsafe value or failed A/B gate is always ``FAILED``.
"""

from __future__ import annotations

from copy import deepcopy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shutil
import stat
import tempfile
from typing import Any, Callable, Mapping

from .x2_locomotion_qualification import (
    EPISODE_EVIDENCE_SCHEMA,
    EPISODE_EVIDENCE_SCHEMA_V1,
    EPISODE_EVIDENCE_SCHEMA_V2,
    REQUIRED_ARTIFACT_ROLES,
    SUITE_SCHEMA_V1,
    SUITE_SCHEMA_V2,
    SCENARIOS,
    build_suite,
    required_artifact_roles,
)
from .x2_qualification_sealing import (
    EPISODE_RESULT_SCHEMA,
    SEALING_INPUT_SCHEMA,
    SEALING_INPUT_SCHEMA_V1,
    SEALING_INPUT_SCHEMA_V2,
    build_episode_payload,
)


FINALIZATION_SCHEMA = "sceneforge/x2-cold-start-finalization/v1"
FROZEN_RUNTIME_INDEX_SCHEMA = "sceneforge/x2-frozen-runtime-index/v1"
PROFILE_MANIFEST_SCHEMA = "sceneforge/x2-locomotion-profile-manifest/v1"
THRESHOLDS_MANIFEST_SCHEMA = "sceneforge/x2-locomotion-thresholds/v1"
JOINT_MAP_MANIFEST_SCHEMA = "sceneforge/x2-joint-map-manifest/v1"
DYNAMICS_MANIFEST_SCHEMA = "sceneforge/x2-physx-dynamics-config-manifest/v1"
FROZEN_INDEX_SCHEMA = "sceneforge/x2-frozen-input-index/v1"
SOURCE_AUDIT_SCHEMA = "sceneforge/x2-combination-source-contract-audit/v1"

PHYSX_AUTHORITY = "genie_sim_isaac_physx"
MUJOCO_AUTHORITY = "google_mujoco_3_3_5"
JOINT_COUNT = 31
NON_FOOT_CONTACT_THRESHOLD_N = 20.0
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_IMAGE_DIGEST = re.compile(r"^sha256:[0-9a-f]{64}$")
_DERIVED_DIRECTORY_NAME = "derived"
_FROZEN_RUNTIME_INDEX_NAME = "runtime-frozen-index.json"

_RUNTIME_FILES = {
    "runtime_result": "combination-runtime-result.json",
    "lifecycle": "lifecycle-receipt.json",
    "reset": "reset-receipt.json",
    "freeze": "sim-time-freeze-receipt.json",
    "controlled_stop": "controlled-stop-proof.json",
    "runtime_manifest": "runtime-manifest.json",
    "source_manifest": "source-manifest.json",
    "asset_manifest": "asset-manifest.json",
    "controller_manifest": "controller-manifest.json",
    "dynamics_alignment": "dynamics-alignment.json",
    "state_trace": "state-command-trace.json",
    "contact_trace": "contact-trace.json",
    "band_telemetry": "elastic-band-telemetry.jsonl",
    "engine_log": "engine.log",
    "controller_log": "controller.log",
}

_CASE_PHYSX_RESULT = "cases/{scenario}/physx-result.json"
_CASE_PHYSX_TRACE = "cases/{scenario}/state-command-trace.json"

_PROFILE_FIELDS = (
    "supported_stand_s",
    "band_ramp_s",
    "unsupported_stand_s",
    "walk_s",
    "deceleration_s",
    "cpg_stand_handoff_s",
    "stop_s",
    "command_x_mps",
    "command_y_mps",
    "command_yaw_rps",
)

_REQUIRED_SOURCE_CONTRACT_FILES = (
    "src/sceneforge/x2_physx_gait1.py",
    "src/sceneforge/x2_cold_start_combination.py",
    "admin/spark/x2-geniesim-cold-start-combination-driver.py",
)

_FORBIDDEN_RUNTIME_CALLS = (
    "set_world_pose(",
    "set_local_pose(",
    "set_world_transform(",
    "set_joint_positions(",
    "set_joint_position(",
    "teleport(",
    "set_kinematic_target(",
)


class _Problem(RuntimeError):
    def __init__(self, severity: str, code: str, location: str, message: str):
        super().__init__(message)
        self.finding = {
            "severity": severity,
            "code": code,
            "location": location,
            "message": message,
        }


def _strict_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON constant is forbidden: {value}")


def _load_json(path: Path, location: str) -> dict[str, Any]:
    if not path.is_file():
        raise _Problem("INCOMPLETE", "EVIDENCE_FILE_MISSING", location, f"missing {path}")
    try:
        payload = json.loads(path.read_text(encoding="utf-8"), parse_constant=_strict_constant)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise _Problem("FAILED", "EVIDENCE_JSON_INVALID", location, str(exc)) from exc
    if not isinstance(payload, dict):
        raise _Problem("FAILED", "EVIDENCE_JSON_INVALID", location, "JSON root is not an object")
    return payload


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(payload: object) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _write_new_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise FileExistsError(path)
    data = json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _copy_new(source: Path, target: Path) -> None:
    if not source.is_file():
        raise _Problem("INCOMPLETE", "EVIDENCE_FILE_MISSING", str(source), "source file is missing")
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise FileExistsError(target)
    with source.open("rb") as src, target.open("xb") as dst:
        shutil.copyfileobj(src, dst, length=1024 * 1024)
        dst.flush()
        os.fsync(dst.fileno())
    if _sha256(source) != _sha256(target):
        target.unlink(missing_ok=True)
        raise _Problem("FAILED", "EVIDENCE_COPY_HASH_MISMATCH", str(target), "copied evidence changed")


def _capture(payload: Mapping[str, Any]) -> dict[str, Any]:
    value = payload.get("capture", payload)
    if not isinstance(value, dict):
        raise _Problem("FAILED", "CAPTURE_INVALID", "physx_result.capture", "capture is not an object")
    return value


def _require_false_envelope(payload: Mapping[str, Any], location: str) -> None:
    if payload.get("real_robot_commands") is not False:
        raise _Problem("FAILED", "REAL_ROBOT_COMMANDS", location, "real_robot_commands=false is required")
    if payload.get("qualification") not in {None, False}:
        raise _Problem("FAILED", "EVIDENCE_RELABELED", location, "development evidence may not claim qualification")


def _load_comparator(repo_root: Path) -> Callable[[Path, Path], dict[str, Any]]:
    path = repo_root / "admin/spark/compare-x2-locomotion-ab.py"
    if not path.is_file():
        raise _Problem("INCOMPLETE", "COMPARATOR_MISSING", "repo_root", f"missing {path}")
    spec = importlib.util.spec_from_file_location("sceneforge_x2_locomotion_ab_v2", path)
    if spec is None or spec.loader is None:
        raise _Problem("FAILED", "COMPARATOR_LOAD_FAILED", str(path), "unable to create module spec")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if getattr(module, "SCHEMA", None) != "sceneforge/x2-locomotion-ab/v2":
        raise _Problem("FAILED", "COMPARATOR_SCHEMA_MISMATCH", str(path), "v2 comparator is required")
    return module.build_report


def _relative(root: Path, path: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def _derivation_root(run_dir: Path) -> Path:
    """Select the append-only derivation area when the runner provisioned it.

    Legacy, still-writable test/development bundles continue to use the run
    root.  Production cold-start bundles contain a real ``derived`` directory;
    a symlink is rejected so post-processing cannot escape the frozen episode.
    """

    candidate = run_dir / _DERIVED_DIRECTORY_NAME
    if not candidate.exists() and not candidate.is_symlink():
        return run_dir
    if candidate.is_symlink() or not candidate.is_dir():
        raise ValueError("run derived path must be a non-symlink directory")
    if candidate.resolve().parent != run_dir.resolve():
        raise ValueError("run derived directory escaped episode root")
    return candidate


def _verify_frozen_runtime_index(run_dir: Path) -> dict[str, Any] | None:
    """Verify every frozen raw file before deriving any qualification data.

    New runner bundles always contain ``derived/`` and therefore must carry a
    complete raw-file index.  Files below ``derived/`` are deliberately absent
    because they are later, no-clobber products.  Legacy bundles without the
    reserved directory remain supported for unit tests and migration tooling.
    """

    derived = run_dir / _DERIVED_DIRECTORY_NAME
    if not derived.is_dir() or derived.is_symlink():
        return None
    index_path = run_dir / _FROZEN_RUNTIME_INDEX_NAME
    if index_path.is_symlink():
        raise _Problem(
            "FAILED", "FROZEN_RUNTIME_SYMLINK_FORBIDDEN",
            _FROZEN_RUNTIME_INDEX_NAME, "runtime index must not be a symlink",
        )
    payload = _load_json(index_path, "runtime_frozen_index")
    _require_false_envelope(payload, "runtime_frozen_index")
    if payload.get("schema") != FROZEN_RUNTIME_INDEX_SCHEMA:
        raise _Problem(
            "FAILED",
            "FROZEN_RUNTIME_INDEX_SCHEMA_INVALID",
            "runtime_frozen_index.schema",
            repr(payload.get("schema")),
        )
    if payload.get("derivation_root") != _DERIVED_DIRECTORY_NAME:
        raise _Problem(
            "FAILED",
            "FROZEN_RUNTIME_DERIVATION_ROOT_INVALID",
            "runtime_frozen_index.derivation_root",
            repr(payload.get("derivation_root")),
        )
    rows = payload.get("files")
    if not isinstance(rows, list) or not rows:
        raise _Problem(
            "INCOMPLETE",
            "FROZEN_RUNTIME_INDEX_EMPTY",
            "runtime_frozen_index.files",
            "at least one frozen raw file is required",
        )
    if payload.get("file_count") != len(rows):
        raise _Problem(
            "FAILED",
            "FROZEN_RUNTIME_FILE_COUNT_MISMATCH",
            "runtime_frozen_index.file_count",
            f"declared {payload.get('file_count')!r}, indexed {len(rows)}",
        )

    indexed: dict[str, Mapping[str, Any]] = {}
    for offset, row in enumerate(rows):
        location = f"runtime_frozen_index.files[{offset}]"
        if not isinstance(row, Mapping):
            raise _Problem("FAILED", "FROZEN_RUNTIME_ENTRY_INVALID", location, "entry is not an object")
        raw = row.get("path")
        relative = Path(raw) if isinstance(raw, str) else Path(".")
        if (
            not isinstance(raw, str)
            or not raw
            or relative.is_absolute()
            or ".." in relative.parts
            or not relative.parts
            or relative.parts[0] == _DERIVED_DIRECTORY_NAME
            or relative.as_posix() == _FROZEN_RUNTIME_INDEX_NAME
        ):
            raise _Problem("FAILED", "FROZEN_RUNTIME_PATH_INVALID", location, repr(raw))
        normalized = relative.as_posix()
        if normalized in indexed:
            raise _Problem("FAILED", "FROZEN_RUNTIME_PATH_DUPLICATE", location, normalized)
        digest = row.get("sha256")
        size = row.get("size")
        if not isinstance(digest, str) or _SHA256.fullmatch(digest) is None:
            raise _Problem("FAILED", "FROZEN_RUNTIME_HASH_INVALID", location, repr(digest))
        if isinstance(size, bool) or not isinstance(size, int) or size < 0:
            raise _Problem("FAILED", "FROZEN_RUNTIME_SIZE_INVALID", location, repr(size))
        indexed[normalized] = row

    actual: dict[str, Path] = {}
    for path in run_dir.rglob("*"):
        relative = path.relative_to(run_dir)
        if relative.parts[0] == _DERIVED_DIRECTORY_NAME:
            continue
        if path.is_symlink():
            raise _Problem(
                "FAILED",
                "FROZEN_RUNTIME_SYMLINK_FORBIDDEN",
                relative.as_posix(),
                "raw runtime entries must not be symlinks",
            )
        if path.is_file() and relative.as_posix() != _FROZEN_RUNTIME_INDEX_NAME:
            actual[relative.as_posix()] = path
    if set(actual) != set(indexed):
        missing = sorted(set(indexed) - set(actual))
        unexpected = sorted(set(actual) - set(indexed))
        raise _Problem(
            "FAILED",
            "FROZEN_RUNTIME_FILE_SET_MISMATCH",
            "runtime_frozen_index.files",
            f"missing={missing!r}, unexpected={unexpected!r}",
        )
    if run_dir.stat().st_mode & (stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH):
        raise _Problem(
            "FAILED",
            "FROZEN_RUNTIME_ROOT_WRITABLE",
            "run_dir",
            "frozen runtime root still has a POSIX write bit",
        )
    if index_path.stat().st_mode & (stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH):
        raise _Problem(
            "FAILED",
            "FROZEN_RUNTIME_FILE_WRITABLE",
            _FROZEN_RUNTIME_INDEX_NAME,
            "runtime index still has a POSIX write bit",
        )
    for relative, path in actual.items():
        row = indexed[relative]
        if path.stat().st_mode & (stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH):
            raise _Problem(
                "FAILED", "FROZEN_RUNTIME_FILE_WRITABLE", relative,
                "indexed runtime file still has a POSIX write bit",
            )
        if path.stat().st_size != row["size"]:
            raise _Problem(
                "FAILED", "FROZEN_RUNTIME_SIZE_MISMATCH", relative,
                f"expected {row['size']}, found {path.stat().st_size}",
            )
        actual_hash = _sha256(path)
        if actual_hash != row["sha256"]:
            raise _Problem(
                "FAILED", "FROZEN_RUNTIME_HASH_MISMATCH", relative,
                f"expected {row['sha256']}, found {actual_hash}",
            )
    return {
        "path": _FROZEN_RUNTIME_INDEX_NAME,
        "sha256": _sha256(index_path),
        "file_count": len(indexed),
    }


def _stage_case_inputs(
    final_dir: Path,
    scenario: str,
    mujoco_result: Path,
    physx_result: Path,
    physx_trace: Path,
) -> tuple[Path, Path, dict[str, str]]:
    mujoco_trace = mujoco_result.parent / "trace.json"
    if not mujoco_trace.is_file():
        raise _Problem(
            "INCOMPLETE",
            "MUJOCO_TRACE_MISSING",
            f"mujoco.{scenario}",
            f"A/B v2 requires {mujoco_trace}",
        )
    m_dir = final_dir / "references" / "mujoco" / scenario
    p_dir = final_dir / "references" / "physx" / scenario
    staged_mujoco = m_dir / "result.json"
    staged_physx = p_dir / "result.json"
    _copy_new(mujoco_result, staged_mujoco)
    _copy_new(mujoco_trace, m_dir / "trace.json")
    _copy_new(physx_result, staged_physx)
    # A raw PhysX result is the direct-result branch of the A/B v2 comparator;
    # that branch intentionally accepts only this immutable sibling basename.
    staged_physx_trace = p_dir / "state-command-trace.json"
    _copy_new(physx_trace, staged_physx_trace)
    return staged_mujoco, staged_physx, {
        f"mujoco_{scenario}_reference": str(staged_mujoco),
        f"mujoco_{scenario}_trace": str(m_dir / "trace.json"),
        f"physx_{scenario}_raw_result": str(staged_physx),
        f"physx_{scenario}_trace": str(staged_physx_trace),
    }


def _source_contract_audit(repo_root: Path, manifest: Mapping[str, Any]) -> dict[str, Any]:
    rows = manifest.get("files")
    if not isinstance(rows, list) or not rows:
        raise _Problem("INCOMPLETE", "SOURCE_MANIFEST_EMPTY", "source_manifest.files", "source file hashes are required")
    by_path: dict[str, Mapping[str, Any]] = {}
    for index, row in enumerate(rows):
        if not isinstance(row, dict) or not isinstance(row.get("path"), str):
            raise _Problem("FAILED", "SOURCE_MANIFEST_INVALID", f"source_manifest.files[{index}]", "path is missing")
        digest = row.get("sha256")
        if not isinstance(digest, str) or _SHA256.fullmatch(digest) is None:
            raise _Problem("FAILED", "SOURCE_MANIFEST_INVALID", f"source_manifest.files[{index}]", "SHA-256 is invalid")
        by_path[row["path"].replace("\\", "/")] = row

    audited = []
    forbidden_hits: list[dict[str, str]] = []
    for relative in _REQUIRED_SOURCE_CONTRACT_FILES:
        row = by_path.get(relative)
        if row is None:
            raise _Problem("INCOMPLETE", "SOURCE_CONTRACT_FILE_MISSING", "source_manifest", relative)
        path = repo_root / relative
        if not path.is_file():
            raise _Problem("INCOMPLETE", "SOURCE_CONTRACT_FILE_MISSING", str(path), relative)
        actual = _sha256(path)
        if actual != str(row["sha256"]).lower():
            raise _Problem("FAILED", "SOURCE_HASH_MISMATCH", relative, "runtime manifest and local frozen source differ")
        text = path.read_text(encoding="utf-8")
        for token in _FORBIDDEN_RUNTIME_CALLS:
            if token in text:
                forbidden_hits.append({"path": relative, "token": token})
        audited.append({"path": relative, "sha256": actual})

    driver = (repo_root / _REQUIRED_SOURCE_CONTRACT_FILES[-1]).read_text(encoding="utf-8")
    physx_lock_present = (
        'physics_engine != "isaac_physx"' in driver
        and '"physics_authority": "genie_sim_isaac_physx"' in driver
    )
    if forbidden_hits:
        raise _Problem("FAILED", "KINEMATIC_RUNTIME_API_PRESENT", "source_contract", repr(forbidden_hits))
    if not physx_lock_present:
        raise _Problem("FAILED", "PHYSX_LOCK_MISSING", "source_contract", "driver lacks the explicit PhysX-only lock")
    return {
        "schema": SOURCE_AUDIT_SCHEMA,
        "status": "PASSED",
        "method": "hash_bound_static_audit_of_post_start_combination_control_path",
        "audited_files": audited,
        "forbidden_runtime_calls": list(_FORBIDDEN_RUNTIME_CALLS),
        "forbidden_runtime_call_hits": [],
        "physx_only_lock_present": True,
        "no_runtime_kinematic_teleport_api": True,
        "scope_limitation": "initial pose authoring before PhysX reset is outside locomotion and remains covered by reset receipt",
        "qualification": False,
        "real_robot_commands": False,
    }


def _profile_manifest(captures: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    cases: dict[str, Any] = {}
    for scenario in SCENARIOS:
        profile = captures[scenario].get("profile")
        if not isinstance(profile, dict):
            raise _Problem("INCOMPLETE", "PROFILE_MISSING", f"physx.{scenario}.profile", "profile is required")
        missing = [name for name in _PROFILE_FIELDS if name not in profile]
        if missing:
            raise _Problem("INCOMPLETE", "PROFILE_FIELD_MISSING", f"physx.{scenario}.profile", repr(missing))
        cases[scenario] = {name: profile[name] for name in _PROFILE_FIELDS}
        cases[scenario]["velocity_command_calibration"] = captures[scenario].get("velocity_command_calibration")
    return {
        "schema": PROFILE_MANIFEST_SCHEMA,
        "case_order": list(SCENARIOS),
        "cases": cases,
        "physics_authority": PHYSX_AUTHORITY,
        "real_robot_commands": False,
    }


def _thresholds_manifest(*, qualification_suite_schema: str) -> dict[str, Any]:
    if qualification_suite_schema not in {SUITE_SCHEMA_V1, SUITE_SCHEMA_V2}:
        raise ValueError("unsupported qualification suite schema")
    return {
        "schema": THRESHOLDS_MANIFEST_SCHEMA,
        "comparator_schema": "sceneforge/x2-locomotion-ab/v2",
        "qualification_suite_schema": qualification_suite_schema,
        "minimum_base_height_m": 0.55,
        "maximum_tilt_rad": 0.35,
        "maximum_steady_speed_relative_error": 0.15,
        "between_engine_speed_tolerance": {"absolute_mps": 0.05, "relative": 0.10},
        "maximum_cross_track_m_per_m": 0.05,
        "maximum_yaw_drift_rad_per_m": 0.05,
        "turn_tolerance": {"absolute_rad": 0.10, "relative": 0.10},
        "maximum_stopped_linear_speed_mps": 0.05,
        "maximum_stopped_angular_speed_rps": 0.05,
        "maximum_stop_planar_drift_m": 0.03,
        "minimum_stable_stop_window_s": 2.0,
        "maximum_support_slip_p95_mps": 0.03,
        "maximum_leg_joint_phase_rms_rad": 0.12,
        "maximum_leg_joint_phase_peak_rad": 0.25,
        "maximum_effort_saturation_ratio": 0.01,
        "maximum_non_foot_contact_force_n": NON_FOOT_CONTACT_THRESHOLD_N,
        "required_joint_count": JOINT_COUNT,
        "minimum_freeze_wall_duration_s": 2.0,
        "real_robot_commands": False,
    }


def _joint_map_manifest(captures: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    baseline: dict[str, Any] | None = None
    for scenario in SCENARIOS:
        result = captures[scenario]
        names = result.get("joint_names")
        parameters = result.get("parameters")
        if not isinstance(names, list) or len(names) != JOINT_COUNT or len(set(names)) != JOINT_COUNT:
            raise _Problem("FAILED", "JOINT_ORDER_INVALID", f"physx.{scenario}.joint_names", "exactly 31 unique joints are required")
        if not isinstance(parameters, dict):
            raise _Problem("INCOMPLETE", "JOINT_PARAMETERS_MISSING", f"physx.{scenario}.parameters", "parameters are required")
        position = parameters.get("joint_position_limits_rad")
        velocity = parameters.get("joint_velocity_limits_rps")
        source_hash = parameters.get("joint_limit_source_sha256")
        if not isinstance(position, dict) or not isinstance(velocity, dict):
            raise _Problem("INCOMPLETE", "JOINT_LIMITS_MISSING", f"physx.{scenario}.parameters", "position and velocity limits are required")
        if set(position) != set(names) or set(velocity) != set(names):
            raise _Problem("FAILED", "JOINT_LIMIT_COVERAGE_MISMATCH", f"physx.{scenario}.parameters", "limits must cover the exact joint order")
        if not isinstance(source_hash, str) or _SHA256.fullmatch(source_hash.lower()) is None:
            raise _Problem("INCOMPLETE", "JOINT_LIMIT_HASH_MISSING", f"physx.{scenario}.parameters", "joint limit SHA-256 is required")
        value = {
            "joint_names": names,
            "joint_position_limits_rad": position,
            "joint_velocity_limits_rps": velocity,
            "joint_limit_source_sha256": source_hash.lower(),
        }
        if baseline is None:
            baseline = value
        elif value != baseline:
            raise _Problem("FAILED", "JOINT_MAP_CHANGED_BETWEEN_CASES", f"physx.{scenario}", "joint order or limits changed")
    assert baseline is not None
    return {
        "schema": JOINT_MAP_MANIFEST_SCHEMA,
        **baseline,
        "physics_authority": PHYSX_AUTHORITY,
        "real_robot_commands": False,
    }


def _dynamics_manifest(alignment: Mapping[str, Any], runtime: Mapping[str, Any]) -> dict[str, Any]:
    _require_false_envelope(alignment, "dynamics_alignment")
    if alignment.get("status") != "PASSED" or alignment.get("pre_world_reset") is not True:
        raise _Problem("FAILED", "DYNAMICS_ALIGNMENT_FAILED", "dynamics_alignment", "pre-reset alignment did not pass")
    if alignment.get("physics_authority") != PHYSX_AUTHORITY:
        raise _Problem("FAILED", "WRONG_PHYSICS_AUTHORITY", "dynamics_alignment", "PhysX authority is required")
    image_digest = runtime.get("engine_image_digest")
    if not isinstance(image_digest, str) or _IMAGE_DIGEST.fullmatch(image_digest) is None:
        raise _Problem("INCOMPLETE", "ENGINE_IMAGE_DIGEST_MISSING", "runtime_manifest", "immutable image digest is required")
    physics_hz = runtime.get("physics_hz")
    if not isinstance(physics_hz, (int, float)) or isinstance(physics_hz, bool) or not math.isfinite(float(physics_hz)) or float(physics_hz) <= 0:
        raise _Problem("FAILED", "PHYSICS_FREQUENCY_INVALID", "runtime_manifest.physics_hz", repr(physics_hz))
    fields = (
        "scope", "ground_path", "material_path", "static_friction", "dynamic_friction",
        "restitution", "friction_combine_mode", "foot_collision_mode",
        "foot_collision_geometry_changed", "authored_vendor_capsule_count",
        "joint_or_controller_parameters_changed", "stage_root_layer_sha256",
    )
    return {
        "schema": DYNAMICS_MANIFEST_SCHEMA,
        "physics_authority": PHYSX_AUTHORITY,
        "engine_image_digest": image_digest,
        "physics_hz": float(physics_hz),
        "alignment": {name: alignment.get(name) for name in fields},
        "real_robot_commands": False,
    }


def _validate_trace_joint_coverage(trace: Mapping[str, Any]) -> None:
    names = trace.get("joint_names")
    if not isinstance(names, list) or len(names) != JOINT_COUNT:
        raise _Problem("FAILED", "TRACE_JOINT_COUNT_INVALID", "state_command_trace.joint_names", "31 joints are required")
    physics = trace.get("physics_samples")
    commands = trace.get("commands")
    if not isinstance(physics, list) or not physics or not isinstance(commands, list) or not commands:
        raise _Problem("INCOMPLETE", "TRACE_SAMPLES_MISSING", "state_command_trace", "physics states and commands are required")
    for index, row in enumerate(physics):
        values = row.get("position") if isinstance(row, dict) else None
        if not isinstance(values, list) or len(values) != JOINT_COUNT:
            raise _Problem("FAILED", "TRACE_STATE_WIDTH_INVALID", f"state_command_trace.physics_samples[{index}]", "31 positions are required")
    for index, row in enumerate(commands):
        values = row.get("articulation_position_target") if isinstance(row, dict) else None
        if not isinstance(values, list) or len(values) != JOINT_COUNT:
            raise _Problem("FAILED", "TRACE_TARGET_WIDTH_INVALID", f"state_command_trace.commands[{index}]", "31 targets are required")


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file() or path.stat().st_size == 0:
        raise _Problem("INCOMPLETE", "ELASTIC_BAND_TELEMETRY_MISSING", str(path), "non-empty JSONL is required")
    rows = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            row = json.loads(line, parse_constant=_strict_constant)
        except (json.JSONDecodeError, ValueError) as exc:
            raise _Problem("FAILED", "TELEMETRY_JSON_INVALID", f"{path}:{line_number}", str(exc)) from exc
        if not isinstance(row, dict):
            raise _Problem("FAILED", "TELEMETRY_JSON_INVALID", f"{path}:{line_number}", "record is not an object")
        rows.append(row)
    if not rows:
        raise _Problem("INCOMPLETE", "ELASTIC_BAND_TELEMETRY_MISSING", str(path), "no telemetry records")
    return rows


def _aggregate_facts(
    captures: Mapping[str, Mapping[str, Any]],
    reports: Mapping[str, Mapping[str, Any]],
    state_trace: Mapping[str, Any],
    contact_trace: Mapping[str, Any],
    band_rows: list[Mapping[str, Any]],
    runtime_result: Mapping[str, Any],
    lifecycle: Mapping[str, Any],
    controlled_stop: Mapping[str, Any],
    source_audit: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    _validate_trace_joint_coverage(state_trace)
    contact_rows = contact_trace.get("samples")
    if not isinstance(contact_rows, list) or not contact_rows:
        raise _Problem("INCOMPLETE", "CONTACT_TRACE_EMPTY", "contact_trace.samples", "contact samples are required")

    fall_count = nonfinite_count = violation_count = unexpected_fail_safe_count = 0
    root_pinned = gravity_modified = False
    for scenario in SCENARIOS:
        result = captures[scenario]
        _require_false_envelope(result, f"physx.{scenario}")
        if result.get("status") != "PASSED":
            raise _Problem("FAILED", "PHYSX_CASE_FAILED", f"physx.{scenario}.status", repr(result.get("status")))
        checks = result.get("checks")
        if not isinstance(checks, dict):
            raise _Problem("INCOMPLETE", "PHYSX_CHECKS_MISSING", f"physx.{scenario}.checks", "checks are required")
        required = (
            "no_fall", "no_nan_or_inf", "zero_joint_limit_violations",
            "full_joint_state_and_target_within_limits", "band_released",
            "band_support_zero", "sustained_stable_stop",
        )
        missing = [name for name in required if name not in checks]
        if missing:
            raise _Problem("INCOMPLETE", "PHYSX_CHECK_MISSING", f"physx.{scenario}.checks", repr(missing))
        failed = [name for name in required if checks.get(name) is not True]
        if failed:
            raise _Problem("FAILED", "PHYSX_SAFETY_GATE_FAILED", f"physx.{scenario}.checks", repr(failed))
        metrics = result.get("metrics")
        if not isinstance(metrics, dict):
            raise _Problem("INCOMPLETE", "PHYSX_METRICS_MISSING", f"physx.{scenario}.metrics", "metrics are required")
        count_fields = (
            "fall_event_count", "nonfinite_sample_count",
            "joint_limit_violation_count", "position_target_limit_violation_count",
        )
        missing_counts = [name for name in count_fields if name not in metrics]
        if missing_counts:
            raise _Problem(
                "INCOMPLETE", "SAFETY_COUNT_MISSING",
                f"physx.{scenario}.metrics", repr(missing_counts),
            )
        if (
            "last_error" not in result
            or "root_pinned" not in result
            or "gravity_modified" not in result
        ):
            raise _Problem(
                "INCOMPLETE", "SAFETY_FACT_MISSING", f"physx.{scenario}",
                "last_error, root_pinned and gravity_modified are required",
            )
        fall_count += int(metrics["fall_event_count"])
        nonfinite_count += int(metrics["nonfinite_sample_count"])
        violation_count += int(metrics["joint_limit_violation_count"])
        violation_count += int(metrics["position_target_limit_violation_count"])
        unexpected_fail_safe_count += int(result["last_error"] not in {None, ""})
        root_pinned = root_pinned or result["root_pinned"] is not False
        gravity_modified = gravity_modified or result["gravity_modified"] is not False

    nonfoot_count = 0
    for index, row in enumerate(contact_rows):
        if not isinstance(row, dict):
            raise _Problem("FAILED", "CONTACT_SAMPLE_INVALID", f"contact_trace.samples[{index}]", "sample is not an object")
        value = row.get("maximum_non_foot_contact_force_n")
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
            raise _Problem("FAILED", "CONTACT_FORCE_INVALID", f"contact_trace.samples[{index}]", repr(value))
        nonfoot_count += int(float(value) > NON_FOOT_CONTACT_THRESHOLD_N)

    base_rows = state_trace.get("base_samples")
    if not isinstance(base_rows, list) or not base_rows:
        raise _Problem("INCOMPLETE", "BASE_TRACE_EMPTY", "state_command_trace.base_samples", "base samples are required")
    suspension_windows: dict[str, Any] = {}
    all_windows_zero = True
    all_released_before = True
    for scenario in SCENARIOS:
        gait = [
            row for row in base_rows
            if isinstance(row, dict) and row.get("scenario") == scenario and row.get("phase") == "GAIT1_FORWARD"
        ]
        if not gait:
            raise _Problem("INCOMPLETE", "GAIT_WINDOW_MISSING", f"state_command_trace.{scenario}", "GAIT1_FORWARD samples are required")
        start, end = float(gait[0]["sim_time"]), float(gait[-1]["sim_time"])
        during = [row for row in band_rows if start <= float(row.get("sim_time_s", -math.inf)) <= end]
        before = [row for row in band_rows if float(row.get("sim_time_s", math.inf)) <= start]
        if not during or not before:
            raise _Problem("INCOMPLETE", "BAND_WINDOW_COVERAGE_MISSING", f"elastic_band_telemetry.{scenario}", "pre-walk and in-walk samples are required")
        released_before = before[-1].get("state") == "RELEASED"
        zero = all(
            row.get("state") == "RELEASED"
            and float(row.get("support_ratio", math.nan)) == 0.0
            and float(row.get("force_norm_n", math.nan)) == 0.0
            for row in during
        )
        all_released_before = all_released_before and released_before
        all_windows_zero = all_windows_zero and zero
        suspension_windows[scenario] = {
            "start_sim_time_s": start,
            "end_sim_time_s": end,
            "telemetry_sample_count": len(during),
            "released_before_locomotion": released_before,
            "support_and_force_zero_during_locomotion": zero,
        }

    if not all_released_before or not all_windows_zero:
        raise _Problem("FAILED", "SUSPENSION_ACTIVE_DURING_LOCOMOTION", "elastic_band_telemetry", repr(suspension_windows))

    if runtime_result.get("physics_authority") != PHYSX_AUTHORITY or runtime_result.get("single_physics_lifecycle") is not True:
        raise _Problem("FAILED", "PHYSICS_AUTHORITY_INVALID", "combination_runtime", "one PhysX lifecycle is required")
    if lifecycle.get("physics_authority_process_count") != 1:
        raise _Problem("FAILED", "AUTHORITY_PROCESS_COUNT_INVALID", "lifecycle", repr(lifecycle.get("physics_authority_process_count")))
    controlled_stop_ok = (
        controlled_stop.get("status") == "PASSED"
        and controlled_stop.get("request_present") is True
        and controlled_stop.get("engine_logged_completion") is True
        and controlled_stop.get("container_exit_code") == 0
        and lifecycle.get("controlled_stop_complete") is True
        and lifecycle.get("teardown_complete") is True
    )
    if not controlled_stop_ok:
        raise _Problem("FAILED", "CONTROLLED_STOP_FAILED", "controlled_stop", "stop or teardown proof failed")

    safety = {
        "one_physics_authority": True,
        "no_fallback": source_audit.get("physx_only_lock_present") is True,
        "no_fall": fall_count == 0,
        "no_nan_or_inf": nonfinite_count == 0,
        "full_joint_state_and_target_within_limits": violation_count == 0,
        "suspension_released_before_locomotion": all_released_before,
        "suspension_force_zero_during_locomotion": all_windows_zero,
        "controlled_stop_complete": controlled_stop_ok,
        "real_robot_commands": False,
        "root_pinned": root_pinned,
        "gravity_modified": gravity_modified,
        "kinematic_teleportation_used": not bool(source_audit.get("no_runtime_kinematic_teleport_api")),
        "fall_count": fall_count,
        "nonfinite_value_count": nonfinite_count,
        "joint_limit_violation_count": violation_count,
        "nonfoot_contact_count": nonfoot_count,
        "unexpected_fail_safe_count": unexpected_fail_safe_count,
        "joint_state_count": JOINT_COUNT,
        "joint_target_count": JOINT_COUNT,
    }
    false_or_nonzero = (
        not all(safety[name] is True for name in (
            "one_physics_authority", "no_fallback", "no_fall", "no_nan_or_inf",
            "full_joint_state_and_target_within_limits", "suspension_released_before_locomotion",
            "suspension_force_zero_during_locomotion", "controlled_stop_complete",
        ))
        or any(safety[name] is not False for name in ("real_robot_commands", "root_pinned", "gravity_modified", "kinematic_teleportation_used"))
        or any(safety[name] != 0 for name in ("fall_count", "nonfinite_value_count", "joint_limit_violation_count", "nonfoot_contact_count", "unexpected_fail_safe_count"))
    )
    if false_or_nonzero:
        raise _Problem("FAILED", "AGGREGATE_SAFETY_FAILED", "safety", repr(safety))

    stable_rows = []
    case_facts = {}
    for scenario in SCENARIOS:
        report = reports[scenario]
        checks = report.get("checks")
        if not isinstance(checks, dict):
            raise _Problem("INCOMPLETE", "AB_CHECKS_MISSING", f"ab.{scenario}", "checks are required")
        stable_passed = all(checks.get(name) is True for name in (
            "physx_stable_stop_p95", "physx_stable_stop_maximum", "physx_stable_stop_drift",
        ))
        if report.get("case_gate_passed") is not True:
            comparison = report.get("phase_aligned_leg_joint_comparison")
            if isinstance(comparison, dict) and comparison.get("available") is False:
                raise _Problem("INCOMPLETE", "AB_TRACE_COVERAGE_INCOMPLETE", f"ab.{scenario}", repr(comparison))
            raise _Problem("FAILED", "AB_CASE_GATE_FAILED", f"ab.{scenario}", "v2 A/B gate failed")
        if not stable_passed:
            raise _Problem("FAILED", "AB_STABLE_STOP_FAILED", f"ab.{scenario}", "A/B stop checks failed")
        inputs = report.get("inputs")
        mujoco = inputs.get("mujoco") if isinstance(inputs, dict) else None
        reference = mujoco.get("sha256") if isinstance(mujoco, dict) else None
        if not isinstance(reference, str) or _SHA256.fullmatch(reference) is None:
            raise _Problem("INCOMPLETE", "MUJOCO_REFERENCE_HASH_MISSING", f"ab.{scenario}.inputs", "SHA-256 is required")
        case_facts[scenario] = {
            "status": "PASSED",
            "gate_passed": True,
            "stable_stop_passed": True,
            "mujoco_alignment_passed": True,
            "mujoco_reference_sha256": reference,
            "artifact_role": f"{scenario}_result",
        }
        stop = captures[scenario].get("metrics", {}).get("stable_stop")
        if not isinstance(stop, dict):
            raise _Problem("INCOMPLETE", "STABLE_STOP_MISSING", f"physx.{scenario}.metrics", "stable stop summary is required")
        stable_rows.append(stop)

    def finite_stop(name: str, reducer: Callable[[list[float]], float]) -> float:
        values = []
        for index, row in enumerate(stable_rows):
            value = row.get(name)
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
                raise _Problem("FAILED", "STABLE_STOP_VALUE_INVALID", f"stable_stop[{index}].{name}", repr(value))
            values.append(float(value))
        return reducer(values)

    stable = {
        "all_cases_sustained": True,
        # `window_s` is the requested analysis interval.  At 500 Hz, a
        # complete 1000-sample discrete window spans 999 timestamp intervals
        # (1.998 s), so preserve that measured span separately instead of
        # misclassifying a fully sampled two-second window as too short.
        "window_duration_s": finite_stop("window_s", min),
        "minimum_observed_span_s": finite_stop("observed_duration_s", min),
        "minimum_sample_count": int(finite_stop("sample_count", min)),
        "maximum_linear_speed_mps": finite_stop("maximum_linear_speed_mps", max),
        "maximum_angular_speed_rps": finite_stop("maximum_angular_speed_rps", max),
        "linear_speed_p95_mps": finite_stop("linear_speed_p95_mps", max),
        "angular_speed_p95_rps": finite_stop("angular_speed_p95_rps", max),
        "maximum_planar_drift_m": finite_stop("planar_drift_m", max),
    }
    if (
        stable["window_duration_s"] < 2.0
        or stable["minimum_observed_span_s"] < 1.90
        or stable["minimum_sample_count"] < 100
        or max(
        stable["maximum_linear_speed_mps"], stable["maximum_angular_speed_rps"],
        stable["linear_speed_p95_mps"], stable["angular_speed_p95_rps"],
        ) > 0.05
        or stable["maximum_planar_drift_m"] > 0.03
    ):
        raise _Problem("FAILED", "AGGREGATE_STABLE_STOP_FAILED", "stable_stop", repr(stable))
    return safety, stable, {"cases": case_facts, "suspension_windows": suspension_windows}


def _video_artifacts(
    run_dir: Path,
    final_dir: Path,
    runtime_result: Path,
    state_trace: Path,
    video: Path | None,
    receipt_path: Path | None,
) -> tuple[dict[str, str], list[dict[str, str]]]:
    if video is None or receipt_path is None:
        missing = []
        if video is None:
            missing.append({"severity": "INCOMPLETE", "code": "VIDEO_MISSING", "location": "video", "message": "30 fps qualification video was not provided"})
        if receipt_path is None:
            missing.append({"severity": "INCOMPLETE", "code": "VIDEO_RECEIPT_MISSING", "location": "video_receipt", "message": "source-bound video receipt was not provided"})
        return {}, missing
    video = video.resolve()
    receipt_path = receipt_path.resolve()
    if not video.is_file() or video.stat().st_size <= 0:
        raise _Problem("INCOMPLETE", "VIDEO_MISSING", "video", f"missing or empty {video}")
    receipt = _load_json(receipt_path, "video_receipt")
    _require_false_envelope(receipt, "video_receipt")
    if receipt.get("status") not in {"PASSED", "SUCCEEDED"}:
        raise _Problem("FAILED", "VIDEO_RECEIPT_FAILED", "video_receipt.status", repr(receipt.get("status")))
    actual_video_hash = _sha256(video)
    claimed_video_hash = receipt.get("output_sha256") or receipt.get("video_sha256")
    nested_video = receipt.get("video")
    if claimed_video_hash is None and isinstance(nested_video, dict):
        claimed_video_hash = nested_video.get("sha256")
    if not isinstance(claimed_video_hash, str):
        raise _Problem("INCOMPLETE", "VIDEO_HASH_MISSING", "video_receipt", "video SHA-256 is required")
    if claimed_video_hash.lower() != actual_video_hash:
        raise _Problem("FAILED", "VIDEO_HASH_MISMATCH", "video_receipt", "receipt does not bind the supplied video")

    expected_sources = {_sha256(runtime_result), _sha256(state_trace)}
    claimed_sources = []
    for name in ("source_runtime_sha256", "source_physics_result_sha256", "source_result_sha256", "source_state_command_trace_sha256"):
        value = receipt.get(name)
        if isinstance(value, str):
            claimed_sources.append(value.lower())
    source_trace = receipt.get("source_trace")
    if isinstance(source_trace, dict) and isinstance(source_trace.get("sha256"), str):
        claimed_sources.append(source_trace["sha256"].lower())
    chain_files: tuple[Path, Path] | None = None
    if not any(value in expected_sources for value in claimed_sources):
        # Native offline replay receipts bind a generated render-state trace.
        # Follow its hash-bound receipt once to the authoritative physics trace;
        # do not trust path strings without verifying every edge.
        trace_descriptor = receipt.get("source_trace")
        chain_descriptor = receipt.get("source_trace_receipt")
        if isinstance(trace_descriptor, dict) and isinstance(chain_descriptor, dict):
            trace_name = trace_descriptor.get("path")
            chain_name = chain_descriptor.get("path")
            declared_trace_hash = trace_descriptor.get("sha256")
            declared_chain_hash = chain_descriptor.get("sha256")
            if all(isinstance(value, str) for value in (
                trace_name, chain_name, declared_trace_hash, declared_chain_hash,
            )):
                trace_file = Path(str(trace_name))
                chain_file = Path(str(chain_name))
                if not trace_file.is_absolute():
                    trace_file = receipt_path.parent / trace_file
                if not chain_file.is_absolute():
                    chain_file = receipt_path.parent / chain_file
                trace_file, chain_file = trace_file.resolve(), chain_file.resolve()
                if not trace_file.is_file() or not chain_file.is_file():
                    raise _Problem(
                        "INCOMPLETE", "VIDEO_SOURCE_CHAIN_MISSING", "video_receipt",
                        "render-state trace or its receipt is unavailable",
                    )
                if _sha256(trace_file) != str(declared_trace_hash).lower():
                    raise _Problem("FAILED", "VIDEO_RENDER_TRACE_HASH_MISMATCH", "video_receipt.source_trace", str(trace_file))
                if _sha256(chain_file) != str(declared_chain_hash).lower():
                    raise _Problem("FAILED", "VIDEO_RENDER_RECEIPT_HASH_MISMATCH", "video_receipt.source_trace_receipt", str(chain_file))
                chain = _load_json(chain_file, "video_source_trace_receipt")
                _require_false_envelope(chain, "video_source_trace_receipt")
                if chain.get("status") not in {"PASSED", "SUCCEEDED"}:
                    raise _Problem("FAILED", "VIDEO_SOURCE_CHAIN_FAILED", "video_source_trace_receipt.status", repr(chain.get("status")))
                if chain.get("output_sha256") != str(declared_trace_hash).lower():
                    raise _Problem("FAILED", "VIDEO_SOURCE_CHAIN_OUTPUT_MISMATCH", "video_source_trace_receipt", "receipt does not bind render trace")
                chain_sources = [
                    str(chain[name]).lower()
                    for name in (
                        "source_runtime_sha256", "source_result_sha256",
                        "source_physics_result_sha256", "source_trace_sha256",
                    )
                    if isinstance(chain.get(name), str)
                ]
                if any(value in expected_sources for value in chain_sources):
                    chain_files = (trace_file, chain_file)
                    claimed_sources.extend(chain_sources)
    if not claimed_sources:
        raise _Problem("INCOMPLETE", "VIDEO_SOURCE_HASH_MISSING", "video_receipt", "receipt must bind the runtime result or authoritative state trace")
    if not any(value in expected_sources for value in claimed_sources):
        raise _Problem("FAILED", "VIDEO_SOURCE_HASH_MISMATCH", "video_receipt", "video is not bound to this lifecycle")
    target_video = final_dir / "video" / "replay.mp4"
    target_receipt = final_dir / "video" / "video-receipt.json"
    _copy_new(video, target_video)
    _copy_new(receipt_path, target_receipt)
    artifacts = {
        "video": _relative(run_dir, target_video),
        "video_receipt": _relative(run_dir, target_receipt),
    }
    if chain_files is not None:
        target_trace = final_dir / "video" / "render-state.jsonl"
        target_chain = final_dir / "video" / "render-state-receipt.json"
        _copy_new(chain_files[0], target_trace)
        _copy_new(chain_files[1], target_chain)
        artifacts.update({
            "video_source_trace": _relative(run_dir, target_trace),
            "video_source_trace_receipt": _relative(run_dir, target_chain),
        })
    return artifacts, []


def _preflight_sealing_input(
    run_dir: Path,
    spec: Mapping[str, Any],
    *,
    scratch_dir: Path,
) -> dict[str, Any]:
    """Run the single-attempt validator without publishing an episode result.

    The sealing builder intentionally stops at the first missing artifact role.
    A pending finalization needs the aggregator to inspect every fact block, so
    this constructs the same envelope from every artifact actually present.
    Complete maps are additionally checked by the public sealing builder.
    """

    artifacts = spec.get("artifacts")
    if not isinstance(artifacts, Mapping):
        raise _Problem(
            "INCOMPLETE", "ARTIFACT_MAP_MISSING",
            "sealing_input.artifacts", "artifact map is required",
        )
    sealing_schema = spec.get("schema")
    if sealing_schema == SEALING_INPUT_SCHEMA_V1:
        evidence_schema = EPISODE_EVIDENCE_SCHEMA_V1
        suite_schema = SUITE_SCHEMA_V1
    elif sealing_schema == SEALING_INPUT_SCHEMA_V2:
        evidence_schema = EPISODE_EVIDENCE_SCHEMA_V2
        suite_schema = SUITE_SCHEMA_V2
    else:
        raise _Problem(
            "FAILED", "SEALING_SCHEMA_UNSUPPORTED", "sealing_input.schema",
            repr(sealing_schema),
        )
    required_roles = required_artifact_roles(evidence_schema)
    if set(required_roles).issubset(artifacts):
        build_episode_payload(episode_root=run_dir, sealing_input=spec)
    descriptors = []
    seen: set[Path] = set()
    for role in sorted(artifacts):
        raw = artifacts[role]
        if not isinstance(raw, str) or not raw:
            raise _Problem(
                "INCOMPLETE", "ARTIFACT_PATH_MISSING",
                f"sealing_input.artifacts.{role}", "relative path is required",
            )
        relative = Path(raw)
        if relative.is_absolute() or ".." in relative.parts:
            raise _Problem(
                "FAILED", "ARTIFACT_PATH_ESCAPE",
                f"sealing_input.artifacts.{role}", raw,
            )
        path = (run_dir / relative).resolve()
        try:
            path.relative_to(run_dir.resolve())
        except ValueError as exc:
            raise _Problem(
                "FAILED", "ARTIFACT_PATH_ESCAPE",
                f"sealing_input.artifacts.{role}", raw,
            ) from exc
        if path in seen:
            raise _Problem(
                "FAILED", "DUPLICATE_ARTIFACT_PATH",
                f"sealing_input.artifacts.{role}", raw,
            )
        seen.add(path)
        if not path.is_file():
            raise _Problem(
                "INCOMPLETE", "ARTIFACT_FILE_MISSING",
                f"sealing_input.artifacts.{role}", raw,
            )
        descriptors.append({
            "role": role,
            "path": relative.as_posix(),
            "sha256": _sha256(path),
            "size": path.stat().st_size,
        })
    evidence = {"schema": evidence_schema, "artifacts": descriptors}
    if scratch_dir.resolve() == (run_dir / _DERIVED_DIRECTORY_NAME).resolve():
        evidence["artifact_root"] = ".."
    for name in (
        "episode_id", "physics_authority", "published_command_topics",
        "real_robot_commands", "frozen_inputs", "safety", "lifecycle", "reset",
        "sim_time_freeze", "cases", "stable_stop",
    ):
        if name in spec:
            evidence[name] = deepcopy(spec[name])
    payload = {
        "schema": EPISODE_RESULT_SCHEMA,
        "status": "PASSED",
        "qualification": False,
        "real_robot_commands": False,
        "qualification_evidence": evidence,
    }
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".x2-finalizer-preflight-", suffix=".json", dir=scratch_dir
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        report = build_suite(
            [temporary],
            suite_id="combination-finalizer-preflight",
            suite_schema=suite_schema,
        )
        return report["attempts"][0]
    finally:
        temporary.unlink(missing_ok=True)


def finalize_combination(
    *,
    run_dir: Path,
    mujoco_results: Mapping[str, Path],
    repo_root: Path,
    episode_id: str | None = None,
    video: Path | None = None,
    video_receipt: Path | None = None,
    episode_evidence_version: int = 1,
    output_dir_name: str = "qualification-finalization",
    comparator: Callable[[Path, Path], dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Create immutable post-processing evidence without sealing the episode."""

    run_dir = run_dir.resolve()
    repo_root = repo_root.resolve()
    if not run_dir.is_dir():
        raise FileNotFoundError(run_dir)
    if Path(output_dir_name).name != output_dir_name or output_dir_name in {"", ".", ".."}:
        raise ValueError("output_dir_name must be one direct child name")
    if episode_evidence_version not in {1, 2}:
        raise ValueError("episode_evidence_version must be 1 or 2")
    sealing_schema = (
        SEALING_INPUT_SCHEMA_V2
        if episode_evidence_version == 2
        else SEALING_INPUT_SCHEMA_V1
    )
    evidence_schema = (
        EPISODE_EVIDENCE_SCHEMA_V2
        if episode_evidence_version == 2
        else EPISODE_EVIDENCE_SCHEMA_V1
    )
    derivation_root = _derivation_root(run_dir)
    final_dir = derivation_root / output_dir_name
    final_dir.mkdir(exist_ok=False)
    findings: list[dict[str, str]] = []
    receipt: dict[str, Any] = {
        "schema": FINALIZATION_SCHEMA,
        "status": "INCOMPLETE",
        "qualification": False,
        "real_robot_commands": False,
        "episode_id": episode_id or run_dir.name,
        "episode_evidence_schema": evidence_schema,
        "episode_media_required": episode_evidence_version == 1,
        "output_directory": _relative(run_dir, final_dir),
        "findings": findings,
    }

    try:
        frozen_runtime = _verify_frozen_runtime_index(run_dir)
        if frozen_runtime is not None:
            receipt["frozen_runtime"] = deepcopy(frozen_runtime)
        files = {name: run_dir / relative for name, relative in _RUNTIME_FILES.items()}
        json_inputs = {
            name: _load_json(files[name], name)
            for name in (
                "runtime_result", "lifecycle", "reset", "freeze", "controlled_stop",
                "runtime_manifest", "source_manifest", "asset_manifest",
                "controller_manifest", "dynamics_alignment", "state_trace", "contact_trace",
            )
        }
        for name, payload in json_inputs.items():
            _require_false_envelope(payload, name)
        if not files["engine_log"].is_file() or not files["controller_log"].is_file():
            raise _Problem("INCOMPLETE", "RUNTIME_LOG_MISSING", "run_dir", "engine and controller logs are required")

        missing_mujoco = [scenario for scenario in SCENARIOS if scenario not in mujoco_results]
        if missing_mujoco:
            raise _Problem("INCOMPLETE", "MUJOCO_REFERENCE_MISSING", "mujoco_results", repr(missing_mujoco))
        captures: dict[str, dict[str, Any]] = {}
        physx_paths: dict[str, Path] = {}
        for scenario in SCENARIOS:
            path = run_dir / _CASE_PHYSX_RESULT.format(scenario=scenario)
            payload = _load_json(path, f"physx.{scenario}")
            _require_false_envelope(payload, f"physx.{scenario}.envelope")
            capture = dict(_capture(payload))
            case_trace_path = run_dir / _CASE_PHYSX_TRACE.format(scenario=scenario)
            case_trace = _load_json(case_trace_path, f"physx.{scenario}.trace")
            _require_false_envelope(case_trace, f"physx.{scenario}.trace")
            trace_joint_names = case_trace.get("joint_names")
            if (
                not isinstance(trace_joint_names, list)
                or len(trace_joint_names) != JOINT_COUNT
                or len(set(trace_joint_names)) != JOINT_COUNT
                or not all(isinstance(name, str) and name for name in trace_joint_names)
            ):
                raise _Problem(
                    "FAILED",
                    "CASE_TRACE_JOINT_ORDER_INVALID",
                    f"physx.{scenario}.trace.joint_names",
                    "exactly 31 unique non-empty joint names are required",
                )
            result_joint_names = capture.get("joint_names")
            if result_joint_names is None:
                # Production PhysX results seal the complete articulation in
                # their immutable sibling state-command trace.  The policy
                # itself controls only 23 joints, so the result intentionally
                # does not expose that policy order as the 31-DOF authority.
                capture["joint_names"] = list(trace_joint_names)
            elif result_joint_names != trace_joint_names:
                raise _Problem(
                    "FAILED",
                    "CASE_RESULT_TRACE_JOINT_ORDER_MISMATCH",
                    f"physx.{scenario}.joint_names",
                    "result and authoritative case trace joint orders differ",
                )
            captures[scenario] = capture
            _require_false_envelope(captures[scenario], f"physx.{scenario}.capture")
            physx_paths[scenario] = path

        source_audit = _source_contract_audit(repo_root, json_inputs["source_manifest"])
        source_audit_path = final_dir / "manifests" / "source-contract-audit.json"
        _write_new_json(source_audit_path, source_audit)

        build_report = comparator or _load_comparator(repo_root)
        reports: dict[str, dict[str, Any]] = {}
        extra_absolute: dict[str, str] = {}
        for scenario in SCENARIOS:
            staged_m, staged_p, staged = _stage_case_inputs(
                final_dir,
                scenario,
                Path(mujoco_results[scenario]).resolve(),
                physx_paths[scenario],
                run_dir / _CASE_PHYSX_TRACE.format(scenario=scenario),
            )
            extra_absolute.update(staged)
            try:
                report = build_report(staged_m, staged_p)
            except FileNotFoundError as exc:
                raise _Problem("INCOMPLETE", "AB_INPUT_MISSING", f"ab.{scenario}", str(exc)) from exc
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                raise _Problem("FAILED", "AB_COMPARATOR_ERROR", f"ab.{scenario}", f"{type(exc).__name__}: {exc}") from exc
            if not isinstance(report, dict) or report.get("schema") != "sceneforge/x2-locomotion-ab/v2":
                raise _Problem("FAILED", "AB_SCHEMA_INVALID", f"ab.{scenario}", "v2 report is required")
            if report.get("scenario") != scenario or report.get("real_robot_commands") is not False or report.get("qualification") is not False:
                raise _Problem("FAILED", "AB_IDENTITY_INVALID", f"ab.{scenario}", "scenario/safety identity mismatch")
            output = final_dir / "cases" / scenario / "result.json"
            _write_new_json(output, report)
            reports[scenario] = report

        profile = _profile_manifest(captures)
        thresholds = _thresholds_manifest(
            qualification_suite_schema=(
                SUITE_SCHEMA_V2
                if episode_evidence_version == 2
                else SUITE_SCHEMA_V1
            )
        )
        joint_map = _joint_map_manifest(captures)
        dynamics = _dynamics_manifest(json_inputs["dynamics_alignment"], json_inputs["runtime_manifest"])
        manifest_payloads = {
            "profile": profile,
            "thresholds": thresholds,
            "joint-map": joint_map,
            "dynamics-config": dynamics,
        }
        manifest_paths: dict[str, Path] = {}
        for name, payload in manifest_payloads.items():
            path = final_dir / "manifests" / f"{name}.json"
            _write_new_json(path, payload)
            manifest_paths[name] = path

        stand_hashes = {str(captures[s].get("model", {}).get("stand_sha256", "")).lower() for s in SCENARIOS}
        walk_hashes = {str(captures[s].get("model", {}).get("walk_sha256", "")).lower() for s in SCENARIOS}
        if len(stand_hashes) != 1 or not all(_SHA256.fullmatch(value) for value in stand_hashes):
            raise _Problem("FAILED", "STAND_POLICY_IDENTITY_INVALID", "physx.model", repr(stand_hashes))
        if len(walk_hashes) != 1 or not all(_SHA256.fullmatch(value) for value in walk_hashes):
            raise _Problem("FAILED", "WALK_POLICY_IDENTITY_INVALID", "physx.model", repr(walk_hashes))
        controller_model = json_inputs["controller_manifest"].get("model")
        if not isinstance(controller_model, dict):
            raise _Problem("INCOMPLETE", "CONTROLLER_MODEL_MANIFEST_MISSING", "controller_manifest.model", "policy identities are required")
        if str(controller_model.get("stand_sha256", "")).lower() != next(iter(stand_hashes)):
            raise _Problem("FAILED", "CONTROLLER_STAND_HASH_MISMATCH", "controller_manifest.model", "stand policy changed")
        if str(controller_model.get("walk_sha256", "")).lower() != next(iter(walk_hashes)):
            raise _Problem("FAILED", "CONTROLLER_WALK_HASH_MISMATCH", "controller_manifest.model", "walk policy changed")

        frozen = {
            "profile_sha256": _sha256(manifest_paths["profile"]),
            "thresholds_sha256": _sha256(manifest_paths["thresholds"]),
            "controller_source_sha256": _sha256(files["source_manifest"]),
            "stand_policy_sha256": next(iter(stand_hashes)),
            "walk_policy_sha256": next(iter(walk_hashes)),
            "asset_manifest_sha256": _sha256(files["asset_manifest"]),
            "joint_map_sha256": _sha256(manifest_paths["joint-map"]),
            "dynamics_config_sha256": _sha256(manifest_paths["dynamics-config"]),
        }
        if frozen_runtime is not None:
            frozen["runtime_frozen_index_sha256"] = frozen_runtime["sha256"]
        index = {
            "schema": FROZEN_INDEX_SCHEMA,
            "inputs": {
                "profile_sha256": {"sha256": frozen["profile_sha256"], "manifest": _relative(run_dir, manifest_paths["profile"])},
                "thresholds_sha256": {"sha256": frozen["thresholds_sha256"], "manifest": _relative(run_dir, manifest_paths["thresholds"])},
                "controller_source_sha256": {"sha256": frozen["controller_source_sha256"], "manifest": _relative(run_dir, files["source_manifest"])},
                "stand_policy_sha256": {"sha256": frozen["stand_policy_sha256"], "manifest": _relative(run_dir, files["controller_manifest"]), "json_pointer": "/model/stand_sha256"},
                "walk_policy_sha256": {"sha256": frozen["walk_policy_sha256"], "manifest": _relative(run_dir, files["controller_manifest"]), "json_pointer": "/model/walk_sha256"},
                "asset_manifest_sha256": {"sha256": frozen["asset_manifest_sha256"], "manifest": _relative(run_dir, files["asset_manifest"])},
                "joint_map_sha256": {"sha256": frozen["joint_map_sha256"], "manifest": _relative(run_dir, manifest_paths["joint-map"])},
                "dynamics_config_sha256": {"sha256": frozen["dynamics_config_sha256"], "manifest": _relative(run_dir, manifest_paths["dynamics-config"])},
            },
            "qualification": False,
            "real_robot_commands": False,
        }
        index_path = final_dir / "manifests" / "frozen-input-index.json"
        _write_new_json(index_path, index)

        band_rows = _load_jsonl(files["band_telemetry"])
        safety, stable_stop, aggregate = _aggregate_facts(
            captures,
            reports,
            json_inputs["state_trace"],
            json_inputs["contact_trace"],
            band_rows,
            json_inputs["runtime_result"],
            json_inputs["lifecycle"],
            json_inputs["controlled_stop"],
            source_audit,
        )
        aggregate_path = final_dir / "aggregate-evidence.json"
        _write_new_json(aggregate_path, {
            "schema": "sceneforge/x2-cold-start-aggregate-evidence/v1",
            "safety": safety,
            "stable_stop": stable_stop,
            "suspension_windows": aggregate["suspension_windows"],
            "qualification": False,
            "real_robot_commands": False,
        })

        artifacts = {
            "straight_result": _relative(run_dir, final_dir / "cases/straight/result.json"),
            "turn_left_result": _relative(run_dir, final_dir / "cases/turn_left/result.json"),
            "turn_right_result": _relative(run_dir, final_dir / "cases/turn_right/result.json"),
            "state_command_trace": _relative(run_dir, files["state_trace"]),
            "contact_trace": _relative(run_dir, files["contact_trace"]),
            "elastic_band_telemetry": _relative(run_dir, files["band_telemetry"]),
            "lifecycle_receipt": _relative(run_dir, files["lifecycle"]),
            "reset_receipt": _relative(run_dir, files["reset"]),
            "sim_time_freeze_receipt": _relative(run_dir, files["freeze"]),
            "controlled_stop_proof": _relative(run_dir, files["controlled_stop"]),
            "runtime_manifest": _relative(run_dir, files["runtime_manifest"]),
            "source_manifest": _relative(run_dir, files["source_manifest"]),
            "asset_manifest": _relative(run_dir, files["asset_manifest"]),
            "controller_manifest": _relative(run_dir, files["controller_manifest"]),
            "engine_log": _relative(run_dir, files["engine_log"]),
            "controller_log": _relative(run_dir, files["controller_log"]),
            "profile_manifest": _relative(run_dir, manifest_paths["profile"]),
            "thresholds_manifest": _relative(run_dir, manifest_paths["thresholds"]),
            "joint_map_manifest": _relative(run_dir, manifest_paths["joint-map"]),
            "dynamics_config_manifest": _relative(run_dir, manifest_paths["dynamics-config"]),
            "frozen_input_index": _relative(run_dir, index_path),
            "source_contract_audit": _relative(run_dir, source_audit_path),
            "aggregate_evidence": _relative(run_dir, aggregate_path),
            "dynamics_alignment_receipt": _relative(run_dir, files["dynamics_alignment"]),
        }
        if frozen_runtime is not None:
            artifacts["runtime_frozen_index"] = frozen_runtime["path"]
        artifacts.update({role: _relative(run_dir, Path(path)) for role, path in extra_absolute.items()})
        if episode_evidence_version == 1 or video is not None or video_receipt is not None:
            video_artifacts, video_findings = _video_artifacts(
                run_dir, final_dir, files["runtime_result"], files["state_trace"], video, video_receipt,
            )
        else:
            video_artifacts, video_findings = {}, []
        artifacts.update(video_artifacts)
        findings.extend(video_findings)

        spec = {
            "schema": sealing_schema,
            "episode_id": episode_id or run_dir.name,
            "physics_authority": PHYSX_AUTHORITY,
            "published_command_topics": [],
            "real_robot_commands": False,
            "qualification": False,
            "frozen_inputs": frozen,
            "safety": safety,
            "lifecycle": deepcopy(json_inputs["lifecycle"]),
            "reset": deepcopy(json_inputs["reset"]),
            "sim_time_freeze": deepcopy(json_inputs["freeze"]),
            "cases": aggregate["cases"],
            "stable_stop": stable_stop,
            "artifacts": artifacts,
        }
        pending = bool(video_findings)
        spec_name = "sealing-input.pending.json" if pending else "sealing-input.json"
        spec_path = final_dir / spec_name
        _write_new_json(spec_path, spec)
        receipt["sealing_input"] = _relative(run_dir, spec_path)
        receipt["frozen_inputs"] = frozen
        receipt["case_reports"] = {
            scenario: _relative(run_dir, final_dir / "cases" / scenario / "result.json")
            for scenario in SCENARIOS
        }
        receipt["artifact_role_count"] = len(artifacts)
        attempt = _preflight_sealing_input(
            run_dir,
            spec,
            scratch_dir=derivation_root,
        )
        if attempt["gate_status"] == "FAILED":
            raise _Problem(
                "FAILED", "SEALING_PREFLIGHT_REJECTED", "sealing_input",
                json.dumps(attempt.get("findings", []), sort_keys=True),
            )
        if pending:
            # Preserve every missing fact found by the authoritative validator,
            # not only the two media roles that made this input pending.
            findings.extend(deepcopy(attempt.get("findings", [])))
            receipt["status"] = "INCOMPLETE"
            receipt["ready_for_episode_sealing"] = False
        elif attempt["gate_status"] != "PASSED":
            raise _Problem(
                "INCOMPLETE", "SEALING_PREFLIGHT_REJECTED", "sealing_input",
                json.dumps(attempt.get("findings", []), sort_keys=True),
            )
        else:
            receipt["status"] = "PASSED"
            receipt["ready_for_episode_sealing"] = True
    except _Problem as problem:
        findings.append(problem.finding)
        receipt["status"] = problem.finding["severity"]
        receipt["ready_for_episode_sealing"] = False
    except Exception as exc:
        findings.append({
            "severity": "FAILED",
            "code": "FINALIZER_INTERNAL_ERROR",
            "location": "finalizer",
            "message": f"{type(exc).__name__}: {exc}",
        })
        receipt["status"] = "FAILED"
        receipt["ready_for_episode_sealing"] = False

    receipt["qualification"] = False
    receipt["real_robot_commands"] = False
    receipt_path = final_dir / "finalization-receipt.json"
    _write_new_json(receipt_path, receipt)
    for path in final_dir.rglob("*"):
        if path.is_file():
            try:
                path.chmod(0o444)
            except OSError:
                # Hashes and no-clobber publication remain authoritative on
                # filesystems that do not expose POSIX mode bits.
                pass
    for path in sorted(
        (item for item in final_dir.rglob("*") if item.is_dir()),
        key=lambda item: len(item.parts),
        reverse=True,
    ):
        try:
            path.chmod(0o555)
        except OSError:
            pass
    try:
        final_dir.chmod(0o555)
    except OSError:
        pass
    return receipt
