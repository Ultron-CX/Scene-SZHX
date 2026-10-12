"""Fail-closed aggregation for X2 PhysX cold-start qualification evidence.

The aggregator deliberately does not infer qualification facts from legacy
episode fields.  Every input must contain a ``qualification_evidence`` block
whose referenced artifacts are immutable and hash-verifiable.  Individual
episodes remain development evidence; only a complete five-attempt suite may
set ``qualification=true``.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Iterable


SUITE_SCHEMA_V1 = "sceneforge/x2-locomotion-qualification-suite/v1"
SUITE_SCHEMA_V2 = "sceneforge/x2-locomotion-qualification-suite/v2"
SUITE_SCHEMA = SUITE_SCHEMA_V1
EPISODE_EVIDENCE_SCHEMA_V1 = (
    "sceneforge/x2-locomotion-cold-start-episode-evidence/v1"
)
EPISODE_EVIDENCE_SCHEMA_V2 = (
    "sceneforge/x2-locomotion-cold-start-episode-evidence/v2"
)
EPISODE_EVIDENCE_SCHEMA = EPISODE_EVIDENCE_SCHEMA_V1
CANONICAL_MEDIA_RECEIPT_SCHEMA = (
    "sceneforge/x2-locomotion-canonical-media-receipt/v2"
)
COMBINATION_VIDEO_OVERLAY_SCHEMA = (
    "sceneforge/x2-combination-video-overlay/v1"
)
EXPECTED_ATTEMPT_COUNT = 5
PHYSX_AUTHORITY = "genie_sim_isaac_physx"
SCENARIOS = ("straight", "turn_left", "turn_right")

REQUIRED_TRUE_SAFETY_FLAGS = (
    "one_physics_authority",
    "no_fallback",
    "no_fall",
    "no_nan_or_inf",
    "full_joint_state_and_target_within_limits",
    "suspension_released_before_locomotion",
    "suspension_force_zero_during_locomotion",
    "controlled_stop_complete",
)
REQUIRED_FALSE_SAFETY_FLAGS = (
    "real_robot_commands",
    "root_pinned",
    "gravity_modified",
    "kinematic_teleportation_used",
)
REQUIRED_ZERO_SAFETY_COUNTS = (
    "fall_count",
    "nonfinite_value_count",
    "joint_limit_violation_count",
    "nonfoot_contact_count",
    "unexpected_fail_safe_count",
)
REQUIRED_TRUE_RESET_FLAGS = (
    "physics_reset_complete",
    "controller_history_reset",
    "cpg_state_reset",
    "command_sequence_reset",
    "initial_state_finite",
    "initial_pose_within_tolerance",
)
REQUIRED_TRUE_LIFECYCLE_FLAGS = (
    "cold_start",
    "container_recreated",
    "started_cleanly",
    "controlled_stop_complete",
    "teardown_complete",
)
REQUIRED_FROZEN_INPUT_HASHES = (
    "profile_sha256",
    "thresholds_sha256",
    "controller_source_sha256",
    "stand_policy_sha256",
    "walk_policy_sha256",
    "asset_manifest_sha256",
    "joint_map_sha256",
    "dynamics_config_sha256",
)
REQUIRED_EPISODE_ARTIFACT_ROLES = (
    "straight_result",
    "turn_left_result",
    "turn_right_result",
    "state_command_trace",
    "contact_trace",
    "elastic_band_telemetry",
    "lifecycle_receipt",
    "reset_receipt",
    "sim_time_freeze_receipt",
    "controlled_stop_proof",
    "runtime_manifest",
    "source_manifest",
    "asset_manifest",
    "controller_manifest",
    "engine_log",
    "controller_log",
)
EPISODE_MEDIA_ARTIFACT_ROLES = ("video", "video_receipt")
REQUIRED_ARTIFACT_ROLES_V1 = (
    *REQUIRED_EPISODE_ARTIFACT_ROLES,
    *EPISODE_MEDIA_ARTIFACT_ROLES,
)
REQUIRED_ARTIFACT_ROLES_V2 = REQUIRED_EPISODE_ARTIFACT_ROLES
# Compatibility alias for existing v1 callers.
REQUIRED_ARTIFACT_ROLES = REQUIRED_ARTIFACT_ROLES_V1
DIRECT_PHYSX_CASE_CHECKS = (
    "completed_profile",
    "no_fall",
    "no_nan_or_inf",
    "zero_joint_limit_violations",
    "full_joint_state_and_target_within_limits",
    "sustained_stable_stop",
    "sim_time_freeze_probe",
    "one_command_per_physics_state",
    "bounded_command_execution_age_sim_time",
    "walk_policy_strict_sim_time",
)

_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_IMAGE_DIGEST = re.compile(r"^sha256:[0-9a-f]{64}$")
_SUITE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")


def required_artifact_roles(evidence_schema: str) -> tuple[str, ...]:
    if evidence_schema == EPISODE_EVIDENCE_SCHEMA_V1:
        return REQUIRED_ARTIFACT_ROLES_V1
    if evidence_schema == EPISODE_EVIDENCE_SCHEMA_V2:
        return REQUIRED_ARTIFACT_ROLES_V2
    raise ValueError(f"unsupported episode evidence schema: {evidence_schema!r}")


@dataclass(frozen=True)
class Finding:
    severity: str
    code: str
    location: str
    message: str

    def to_dict(self) -> dict[str, str]:
        return {
            "severity": self.severity,
            "code": self.code,
            "location": self.location,
            "message": self.message,
        }


def _reject_nonfinite_json(value: str) -> None:
    raise ValueError(f"non-finite JSON constant is forbidden: {value}")


def load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=_reject_nonfinite_json,
    )
    if not isinstance(payload, dict):
        raise ValueError("JSON root must be an object")
    return payload


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _missing(
    findings: list[Finding], code: str, location: str, message: str
) -> None:
    findings.append(Finding("INCOMPLETE", code, location, message))


def _failed(
    findings: list[Finding], code: str, location: str, message: str
) -> None:
    findings.append(Finding("FAILED", code, location, message))


def _object(
    parent: dict[str, Any],
    name: str,
    findings: list[Finding],
    location: str,
) -> dict[str, Any] | None:
    value = parent.get(name)
    if not isinstance(value, dict):
        _missing(findings, "MISSING_OBJECT", f"{location}.{name}", "required object is missing")
        return None
    return value


def _required_string(
    parent: dict[str, Any],
    name: str,
    findings: list[Finding],
    location: str,
) -> str | None:
    value = parent.get(name)
    if not isinstance(value, str) or not value.strip():
        _missing(findings, "MISSING_STRING", f"{location}.{name}", "required string is missing")
        return None
    return value


def _required_number(
    parent: dict[str, Any],
    name: str,
    findings: list[Finding],
    location: str,
) -> float | None:
    value = parent.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _missing(findings, "MISSING_NUMBER", f"{location}.{name}", "required finite number is missing")
        return None
    result = float(value)
    if not math.isfinite(result):
        _failed(findings, "NONFINITE_NUMBER", f"{location}.{name}", "number is NaN or Inf")
        return None
    return result


def _require_true_flags(
    parent: dict[str, Any],
    names: Iterable[str],
    findings: list[Finding],
    location: str,
) -> None:
    for name in names:
        if name not in parent:
            _missing(findings, "MISSING_GATE", f"{location}.{name}", "required gate is missing")
        elif parent[name] is not True:
            _failed(findings, "GATE_FAILED", f"{location}.{name}", "required gate is not true")


def _require_false_flags(
    parent: dict[str, Any],
    names: Iterable[str],
    findings: list[Finding],
    location: str,
) -> None:
    for name in names:
        if name not in parent:
            _missing(findings, "MISSING_GATE", f"{location}.{name}", "required safety value is missing")
        elif parent[name] is not False:
            _failed(findings, "UNSAFE_VALUE", f"{location}.{name}", "required safety value is not false")


def _valid_sha(value: object) -> bool:
    return isinstance(value, str) and _SHA256.fullmatch(value) is not None


def _scenario_from_case(payload: dict[str, Any]) -> str | None:
    scenario = payload.get("scenario")
    if scenario in SCENARIOS:
        return str(scenario)
    result = payload.get("capture", payload)
    if not isinstance(result, dict):
        return None
    profile = result.get("profile")
    if not isinstance(profile, dict):
        return None
    yaw = profile.get("command_yaw_rps")
    if isinstance(yaw, bool) or not isinstance(yaw, (int, float)):
        return None
    yaw = float(yaw)
    if not math.isfinite(yaw):
        return None
    return "turn_left" if yaw > 1e-9 else "turn_right" if yaw < -1e-9 else "straight"


def _validate_case_result(
    path: Path,
    expected_scenario: str,
    expected_mujoco_reference_sha256: str | None,
    findings: list[Finding],
    location: str,
) -> None:
    try:
        payload = load_json(path)
    except (OSError, UnicodeError) as exc:
        _missing(findings, "CASE_RESULT_UNREADABLE", location, str(exc))
        return
    except (json.JSONDecodeError, ValueError) as exc:
        _failed(findings, "CASE_RESULT_INVALID", location, str(exc))
        return

    result = payload.get("capture", payload)
    if not isinstance(result, dict):
        _failed(findings, "CASE_RESULT_INVALID", location, "capture is not an object")
        return
    if payload.get("status") != "PASSED" or result.get("status") != "PASSED":
        _failed(findings, "CASE_RESULT_FAILED", location, "case result did not pass")
    if result.get("real_robot_commands") is not False:
        _failed(findings, "REAL_ROBOT_COMMANDS", location, "case result is not real_robot_commands=false")
    if result.get("qualification") is not False:
        _failed(findings, "EPISODE_RELABELLED", location, "case result must remain qualification=false")
    actual_scenario = _scenario_from_case(payload)
    if actual_scenario is None:
        _missing(findings, "CASE_SCENARIO_MISSING", location, "case scenario cannot be established")
    elif actual_scenario != expected_scenario:
        _failed(
            findings,
            "CASE_SCENARIO_MISMATCH",
            location,
            f"expected {expected_scenario}, found {actual_scenario}",
        )

    if str(payload.get("schema", "")).startswith("sceneforge/x2-locomotion-ab/"):
        if payload.get("case_gate_passed") is not True:
            _failed(findings, "CASE_GATE_FAILED", location, "A/B case gate did not pass")
        inputs = payload.get("inputs")
        mujoco = inputs.get("mujoco") if isinstance(inputs, dict) else None
        reference = mujoco.get("sha256") if isinstance(mujoco, dict) else None
        if not _valid_sha(reference):
            _missing(
                findings,
                "MUJOCO_REFERENCE_UNSEALED",
                location,
                "A/B result does not seal its MuJoCo input SHA-256",
            )
        elif reference != expected_mujoco_reference_sha256:
            _failed(
                findings,
                "MUJOCO_REFERENCE_MISMATCH",
                location,
                "case declaration and A/B input refer to different MuJoCo evidence",
            )
        return

    checks = result.get("checks")
    if not isinstance(checks, dict):
        _missing(findings, "CASE_CHECKS_MISSING", location, "direct PhysX case checks are missing")
        return
    _require_true_flags(checks, DIRECT_PHYSX_CASE_CHECKS, findings, f"{location}.checks")
    if result.get("physics_authority") != PHYSX_AUTHORITY:
        _failed(findings, "WRONG_PHYSICS_AUTHORITY", location, "direct case is not PhysX-authoritative")
    reference = result.get("mujoco_reference_sha256")
    if not _valid_sha(reference):
        _missing(
            findings,
            "MUJOCO_REFERENCE_UNSEALED",
            location,
            "direct PhysX result does not seal its MuJoCo reference SHA-256",
        )
    elif reference != expected_mujoco_reference_sha256:
        _failed(
            findings,
            "MUJOCO_REFERENCE_MISMATCH",
            location,
            "case declaration and direct result refer to different MuJoCo evidence",
        )


def _validate_process(
    process: dict[str, Any] | None,
    findings: list[Finding],
    location: str,
    *,
    require_exit_code: bool,
) -> tuple[str | None, int | None]:
    if process is None:
        return None, None
    identity = _required_string(process, "identity", findings, location)
    pid_value = process.get("pid")
    pid: int | None = None
    if isinstance(pid_value, bool) or not isinstance(pid_value, int) or pid_value <= 0:
        _missing(findings, "PROCESS_PID_MISSING", f"{location}.pid", "positive process PID is required")
    else:
        pid = pid_value
    started = _required_number(process, "started_at_unix", findings, location)
    stopped = _required_number(process, "stopped_at_unix", findings, location)
    if started is not None and stopped is not None and stopped <= started:
        _failed(findings, "PROCESS_LIFETIME_INVALID", location, "process stop time must follow start time")
    if require_exit_code:
        if "exit_code" not in process:
            _missing(findings, "PROCESS_EXIT_MISSING", f"{location}.exit_code", "exit code is required")
        elif process.get("exit_code") != 0:
            _failed(findings, "PROCESS_EXIT_FAILED", f"{location}.exit_code", "process exit code is not zero")
    return identity, pid


def _validate_artifacts(
    episode_path: Path,
    evidence: dict[str, Any],
    findings: list[Finding],
    location: str,
    required_roles: Iterable[str],
) -> dict[str, tuple[Path, dict[str, Any]]]:
    artifacts = evidence.get("artifacts")
    if not isinstance(artifacts, list):
        _missing(findings, "ARTIFACT_INDEX_MISSING", f"{location}.artifacts", "artifact index is missing")
        return {}
    by_role: dict[str, tuple[Path, dict[str, Any]]] = {}
    seen_paths: set[str] = set()
    result_parent = episode_path.parent.resolve()
    artifact_root = evidence.get("artifact_root")
    if artifact_root is None:
        root = result_parent
    elif (
        artifact_root == ".."
        and result_parent.name == "derived"
        and not episode_path.parent.is_symlink()
    ):
        root = result_parent.parent.resolve()
        if (root / "derived").resolve() != result_parent:
            _failed(
                findings,
                "ARTIFACT_ROOT_INVALID",
                f"{location}.artifact_root",
                "derived result does not resolve to the reserved episode child",
            )
            return {}
    else:
        _failed(
            findings,
            "ARTIFACT_ROOT_INVALID",
            f"{location}.artifact_root",
            "artifact_root may only be '..' for a result directly inside derived/",
        )
        return {}
    for index, item in enumerate(artifacts):
        item_location = f"{location}.artifacts[{index}]"
        if not isinstance(item, dict):
            _missing(findings, "ARTIFACT_DESCRIPTOR_INVALID", item_location, "descriptor is not an object")
            continue
        role = _required_string(item, "role", findings, item_location)
        relative = _required_string(item, "path", findings, item_location)
        expected_hash = item.get("sha256")
        expected_size = item.get("size")
        if not _valid_sha(expected_hash):
            _missing(findings, "ARTIFACT_HASH_MISSING", f"{item_location}.sha256", "lower-case SHA-256 is required")
        if isinstance(expected_size, bool) or not isinstance(expected_size, int) or expected_size < 0:
            _missing(findings, "ARTIFACT_SIZE_MISSING", f"{item_location}.size", "non-negative size is required")
        if role is None or relative is None:
            continue
        if role in by_role:
            _failed(findings, "DUPLICATE_ARTIFACT_ROLE", item_location, f"role {role} occurs more than once")
            continue
        if relative in seen_paths:
            _failed(findings, "DUPLICATE_ARTIFACT_PATH", item_location, f"path {relative} occurs more than once")
            continue
        seen_paths.add(relative)
        rel_path = Path(relative)
        if rel_path.is_absolute() or ".." in rel_path.parts:
            _failed(findings, "ARTIFACT_PATH_ESCAPE", item_location, "artifact path must remain inside episode directory")
            continue
        resolved = (root / rel_path).resolve()
        try:
            resolved.relative_to(root)
        except ValueError:
            _failed(findings, "ARTIFACT_PATH_ESCAPE", item_location, "resolved artifact escapes episode directory")
            continue
        by_role[role] = (resolved, item)
        if not resolved.is_file():
            _missing(findings, "ARTIFACT_FILE_MISSING", item_location, f"artifact is absent: {relative}")
            continue
        actual_size = resolved.stat().st_size
        if isinstance(expected_size, int) and actual_size != expected_size:
            _failed(findings, "ARTIFACT_SIZE_MISMATCH", item_location, f"expected {expected_size}, found {actual_size}")
        if _valid_sha(expected_hash):
            actual_hash = sha256(resolved)
            if actual_hash != expected_hash:
                _failed(findings, "ARTIFACT_HASH_MISMATCH", item_location, f"expected {expected_hash}, found {actual_hash}")
    for role in required_roles:
        if role not in by_role:
            _missing(findings, "ARTIFACT_ROLE_MISSING", f"{location}.artifacts", f"required role {role} is missing")
    return by_role


def _validate_episode(
    path: Path,
    index: int,
    *,
    expected_evidence_schema: str = EPISODE_EVIDENCE_SCHEMA_V1,
) -> dict[str, Any]:
    findings: list[Finding] = []
    location = f"episodes[{index}]"
    summary: dict[str, Any] = {
        "index": index,
        "path": str(path),
        "sha256": None,
        "size": None,
        "episode_id": None,
        "container_id": None,
        "reset_id": None,
        "engine_process_identity": None,
        "controller_process_identity": None,
        "frozen_inputs": None,
        "container_image_digest": None,
        "evidence_schema": None,
        "artifact_sha256_by_role": {},
    }
    if not path.is_file():
        _missing(findings, "EPISODE_FILE_MISSING", location, f"episode file is absent: {path}")
        summary["findings"] = [item.to_dict() for item in findings]
        summary["gate_status"] = "INCOMPLETE"
        return summary
    summary["sha256"] = sha256(path)
    summary["size"] = path.stat().st_size
    try:
        payload = load_json(path)
    except (OSError, UnicodeError) as exc:
        _missing(findings, "EPISODE_UNREADABLE", location, str(exc))
        payload = {}
    except (json.JSONDecodeError, ValueError) as exc:
        _failed(findings, "EPISODE_INVALID_JSON", location, str(exc))
        payload = {}

    if payload:
        if payload.get("status") != "PASSED":
            if "status" not in payload:
                _missing(findings, "EPISODE_STATUS_MISSING", f"{location}.status", "status is missing")
            else:
                _failed(findings, "EPISODE_FAILED", f"{location}.status", "episode status is not PASSED")
        if payload.get("qualification") is not False:
            if "qualification" not in payload:
                _missing(findings, "EPISODE_QUALIFICATION_MISSING", f"{location}.qualification", "qualification=false is missing")
            else:
                _failed(findings, "EPISODE_RELABELLED", f"{location}.qualification", "single episode may not claim qualification")
        if payload.get("real_robot_commands") is not False:
            if "real_robot_commands" not in payload:
                _missing(findings, "REAL_ROBOT_FLAG_MISSING", f"{location}.real_robot_commands", "real_robot_commands=false is missing")
            else:
                _failed(findings, "REAL_ROBOT_COMMANDS", f"{location}.real_robot_commands", "real robot commands are forbidden")

    evidence = _object(payload, "qualification_evidence", findings, location)
    if evidence is not None:
        summary["evidence_schema"] = evidence.get("schema")
        if evidence.get("schema") != expected_evidence_schema:
            if "schema" not in evidence:
                _missing(findings, "EVIDENCE_SCHEMA_MISSING", f"{location}.qualification_evidence.schema", "sealed schema is missing")
            else:
                _failed(findings, "EVIDENCE_SCHEMA_UNSUPPORTED", f"{location}.qualification_evidence.schema", "sealed schema is unsupported")
        summary["episode_id"] = _required_string(evidence, "episode_id", findings, f"{location}.qualification_evidence")
        if evidence.get("physics_authority") != PHYSX_AUTHORITY:
            if "physics_authority" not in evidence:
                _missing(findings, "PHYSICS_AUTHORITY_MISSING", f"{location}.qualification_evidence.physics_authority", "physics authority is missing")
            else:
                _failed(findings, "WRONG_PHYSICS_AUTHORITY", f"{location}.qualification_evidence.physics_authority", "only Genie Sim/PhysX is accepted")
        if evidence.get("real_robot_commands") is not False:
            if "real_robot_commands" not in evidence:
                _missing(findings, "REAL_ROBOT_FLAG_MISSING", f"{location}.qualification_evidence.real_robot_commands", "real_robot_commands=false is missing")
            else:
                _failed(findings, "REAL_ROBOT_COMMANDS", f"{location}.qualification_evidence.real_robot_commands", "real robot commands are forbidden")
        topics = evidence.get("published_command_topics")
        if topics != []:
            if topics is None:
                _missing(findings, "COMMAND_TOPICS_MISSING", f"{location}.qualification_evidence.published_command_topics", "empty command-topic list is missing")
            else:
                _failed(findings, "COMMAND_TOPICS_PUBLISHED", f"{location}.qualification_evidence.published_command_topics", "qualification evidence published external command topics")

        artifacts = _validate_artifacts(
            path,
            evidence,
            findings,
            f"{location}.qualification_evidence",
            required_artifact_roles(expected_evidence_schema),
        )
        summary["artifact_sha256_by_role"] = {
            role: descriptor.get("sha256")
            for role, (_, descriptor) in artifacts.items()
            if _valid_sha(descriptor.get("sha256"))
        }

        frozen = _object(evidence, "frozen_inputs", findings, f"{location}.qualification_evidence")
        if frozen is not None:
            for name in REQUIRED_FROZEN_INPUT_HASHES:
                if not _valid_sha(frozen.get(name)):
                    _missing(findings, "FROZEN_INPUT_HASH_MISSING", f"{location}.qualification_evidence.frozen_inputs.{name}", "lower-case SHA-256 is required")
            summary["frozen_inputs"] = {
                name: frozen.get(name) for name in REQUIRED_FROZEN_INPUT_HASHES
            }

        safety = _object(evidence, "safety", findings, f"{location}.qualification_evidence")
        if safety is not None:
            _require_true_flags(safety, REQUIRED_TRUE_SAFETY_FLAGS, findings, f"{location}.qualification_evidence.safety")
            _require_false_flags(safety, REQUIRED_FALSE_SAFETY_FLAGS, findings, f"{location}.qualification_evidence.safety")
            for name in REQUIRED_ZERO_SAFETY_COUNTS:
                if name not in safety:
                    _missing(findings, "SAFETY_COUNT_MISSING", f"{location}.qualification_evidence.safety.{name}", "zero-valued safety count is missing")
                elif safety.get(name) != 0:
                    _failed(findings, "SAFETY_COUNT_NONZERO", f"{location}.qualification_evidence.safety.{name}", "safety count is not zero")
            for name in ("joint_state_count", "joint_target_count"):
                if name not in safety:
                    _missing(findings, "FULL_JOINT_COUNT_MISSING", f"{location}.qualification_evidence.safety.{name}", "31-joint coverage is missing")
                elif safety.get(name) != 31:
                    _failed(findings, "FULL_JOINT_COUNT_FAILED", f"{location}.qualification_evidence.safety.{name}", "all 31 joints must be covered")

        lifecycle = _object(evidence, "lifecycle", findings, f"{location}.qualification_evidence")
        if lifecycle is not None:
            _require_true_flags(lifecycle, REQUIRED_TRUE_LIFECYCLE_FLAGS, findings, f"{location}.qualification_evidence.lifecycle")
            summary["container_id"] = _required_string(lifecycle, "container_id", findings, f"{location}.qualification_evidence.lifecycle")
            image_digest = lifecycle.get("container_image_digest")
            if not isinstance(image_digest, str) or _IMAGE_DIGEST.fullmatch(image_digest) is None:
                _missing(findings, "IMAGE_DIGEST_MISSING", f"{location}.qualification_evidence.lifecycle.container_image_digest", "immutable sha256 image digest is required")
            else:
                summary["container_image_digest"] = image_digest
            if lifecycle.get("physics_authority_process_count") != 1:
                if "physics_authority_process_count" not in lifecycle:
                    _missing(findings, "AUTHORITY_PROCESS_COUNT_MISSING", f"{location}.qualification_evidence.lifecycle.physics_authority_process_count", "authority process count is missing")
                else:
                    _failed(findings, "MULTIPLE_PHYSICS_AUTHORITIES", f"{location}.qualification_evidence.lifecycle.physics_authority_process_count", "exactly one physics authority is required")
            for name in ("residual_process_count", "target_processes_before_start", "target_processes_after_stop"):
                if name not in lifecycle:
                    _missing(findings, "PROCESS_COUNT_MISSING", f"{location}.qualification_evidence.lifecycle.{name}", "zero process count is missing")
                elif lifecycle.get(name) != 0:
                    _failed(findings, "RESIDUAL_PROCESS", f"{location}.qualification_evidence.lifecycle.{name}", "target process count is not zero")
            if lifecycle.get("container_exit_code") != 0:
                if "container_exit_code" not in lifecycle:
                    _missing(findings, "CONTAINER_EXIT_MISSING", f"{location}.qualification_evidence.lifecycle.container_exit_code", "container exit code is missing")
                else:
                    _failed(findings, "CONTAINER_EXIT_FAILED", f"{location}.qualification_evidence.lifecycle.container_exit_code", "container did not exit cleanly")
            container_started = _required_number(
                lifecycle,
                "container_started_at_unix",
                findings,
                f"{location}.qualification_evidence.lifecycle",
            )
            container_removed = _required_number(
                lifecycle,
                "container_removed_at_unix",
                findings,
                f"{location}.qualification_evidence.lifecycle",
            )
            if (
                container_started is not None
                and container_removed is not None
                and container_removed <= container_started
            ):
                _failed(
                    findings,
                    "CONTAINER_LIFETIME_INVALID",
                    f"{location}.qualification_evidence.lifecycle",
                    "container removal time must follow start time",
                )
            engine = _object(lifecycle, "engine_process", findings, f"{location}.qualification_evidence.lifecycle")
            controller = _object(lifecycle, "controller_process", findings, f"{location}.qualification_evidence.lifecycle")
            engine_identity, engine_pid = _validate_process(engine, findings, f"{location}.qualification_evidence.lifecycle.engine_process", require_exit_code=True)
            mode = controller.get("mode") if controller else None
            if controller is not None and mode not in {"in_process", "separate"}:
                _missing(findings, "CONTROLLER_MODE_MISSING", f"{location}.qualification_evidence.lifecycle.controller_process.mode", "mode must be in_process or separate")
            controller_identity, controller_pid = _validate_process(
                controller,
                findings,
                f"{location}.qualification_evidence.lifecycle.controller_process",
                require_exit_code=mode == "separate",
            )
            if mode == "in_process" and (
                engine_pid != controller_pid or engine_identity != controller_identity
            ):
                _failed(findings, "IN_PROCESS_IDENTITY_MISMATCH", f"{location}.qualification_evidence.lifecycle.controller_process", "in-process controller must share engine identity")
            if mode == "separate" and engine_identity == controller_identity:
                _failed(findings, "SEPARATE_IDENTITY_COLLISION", f"{location}.qualification_evidence.lifecycle.controller_process", "separate controller must have a distinct identity")
            summary["engine_process_identity"] = engine_identity
            summary["controller_process_identity"] = controller_identity

        reset = _object(evidence, "reset", findings, f"{location}.qualification_evidence")
        if reset is not None:
            if reset.get("status") != "PASSED":
                if "status" not in reset:
                    _missing(findings, "RESET_STATUS_MISSING", f"{location}.qualification_evidence.reset.status", "reset status is missing")
                else:
                    _failed(findings, "RESET_FAILED", f"{location}.qualification_evidence.reset.status", "reset did not pass")
            summary["reset_id"] = _required_string(reset, "reset_id", findings, f"{location}.qualification_evidence.reset")
            _require_true_flags(reset, REQUIRED_TRUE_RESET_FLAGS, findings, f"{location}.qualification_evidence.reset")
            if not _valid_sha(reset.get("initial_state_sha256")):
                _missing(findings, "INITIAL_STATE_HASH_MISSING", f"{location}.qualification_evidence.reset.initial_state_sha256", "initial state SHA-256 is required")

        freeze = _object(evidence, "sim_time_freeze", findings, f"{location}.qualification_evidence")
        if freeze is not None:
            if freeze.get("status") != "PASSED":
                if "status" not in freeze:
                    _missing(findings, "FREEZE_STATUS_MISSING", f"{location}.qualification_evidence.sim_time_freeze.status", "freeze status is missing")
                else:
                    _failed(findings, "FREEZE_FAILED", f"{location}.qualification_evidence.sim_time_freeze.status", "sim-time freeze gate did not pass")
            wall_duration = _required_number(freeze, "observed_wall_duration_s", findings, f"{location}.qualification_evidence.sim_time_freeze")
            if wall_duration is not None and wall_duration < 2.0:
                _failed(findings, "FREEZE_TOO_SHORT", f"{location}.qualification_evidence.sim_time_freeze.observed_wall_duration_s", "freeze must last at least 2 wall seconds")
            tolerance = 1e-12
            for name in (
                "sim_time_advance_s",
                "clock_update_count",
                "physics_state_update_count",
                "controller_update_count",
                "policy_update_count",
                "command_update_count",
                "cpg_phase_advance_rad",
            ):
                value = _required_number(freeze, name, findings, f"{location}.qualification_evidence.sim_time_freeze")
                if value is not None and abs(value) > tolerance:
                    _failed(findings, "SIM_TIME_FREEZE_ADVANCED", f"{location}.qualification_evidence.sim_time_freeze.{name}", "controller state advanced while sim time was frozen")
            if freeze.get("resume_completed") is not True:
                if "resume_completed" not in freeze:
                    _missing(findings, "FREEZE_RESUME_MISSING", f"{location}.qualification_evidence.sim_time_freeze.resume_completed", "post-freeze resume evidence is missing")
                else:
                    _failed(findings, "FREEZE_RESUME_FAILED", f"{location}.qualification_evidence.sim_time_freeze.resume_completed", "controller did not resume cleanly")

        cases = _object(evidence, "cases", findings, f"{location}.qualification_evidence")
        if cases is not None:
            for scenario in SCENARIOS:
                case = _object(cases, scenario, findings, f"{location}.qualification_evidence.cases")
                if case is None:
                    continue
                _require_true_flags(
                    case,
                    ("gate_passed", "stable_stop_passed", "mujoco_alignment_passed"),
                    findings,
                    f"{location}.qualification_evidence.cases.{scenario}",
                )
                if case.get("status") != "PASSED":
                    if "status" not in case:
                        _missing(findings, "CASE_STATUS_MISSING", f"{location}.qualification_evidence.cases.{scenario}.status", "case status is missing")
                    else:
                        _failed(findings, "CASE_FAILED", f"{location}.qualification_evidence.cases.{scenario}.status", "case did not pass")
                if not _valid_sha(case.get("mujoco_reference_sha256")):
                    _missing(findings, "MUJOCO_REFERENCE_HASH_MISSING", f"{location}.qualification_evidence.cases.{scenario}.mujoco_reference_sha256", "MuJoCo reference SHA-256 is required")
                role = case.get("artifact_role")
                expected_role = f"{scenario}_result"
                if role != expected_role:
                    if role is None:
                        _missing(findings, "CASE_ARTIFACT_ROLE_MISSING", f"{location}.qualification_evidence.cases.{scenario}.artifact_role", f"role {expected_role} is required")
                    else:
                        _failed(findings, "CASE_ARTIFACT_ROLE_MISMATCH", f"{location}.qualification_evidence.cases.{scenario}.artifact_role", f"expected role {expected_role}")
                artifact = artifacts.get(expected_role)
                if artifact is not None and artifact[0].is_file():
                    _validate_case_result(
                        artifact[0],
                        scenario,
                        case.get("mujoco_reference_sha256"),
                        findings,
                        f"{location}.qualification_evidence.cases.{scenario}",
                    )

        stop = _object(evidence, "stable_stop", findings, f"{location}.qualification_evidence")
        if stop is not None:
            _require_true_flags(stop, ("all_cases_sustained",), findings, f"{location}.qualification_evidence.stable_stop")
            duration = _required_number(stop, "window_duration_s", findings, f"{location}.qualification_evidence.stable_stop")
            if duration is not None and duration < 2.0:
                _failed(findings, "STOP_WINDOW_TOO_SHORT", f"{location}.qualification_evidence.stable_stop.window_duration_s", "stable-stop window must be at least 2 seconds")
            thresholds = {
                "maximum_linear_speed_mps": 0.05,
                "maximum_angular_speed_rps": 0.05,
                "linear_speed_p95_mps": 0.05,
                "angular_speed_p95_rps": 0.05,
                "maximum_planar_drift_m": 0.03,
            }
            for name, threshold in thresholds.items():
                value = _required_number(stop, name, findings, f"{location}.qualification_evidence.stable_stop")
                if value is not None and value > threshold:
                    _failed(findings, "STABLE_STOP_FAILED", f"{location}.qualification_evidence.stable_stop.{name}", f"{value} exceeds {threshold}")

    severities = {item.severity for item in findings}
    summary["gate_status"] = (
        "FAILED" if "FAILED" in severities
        else "INCOMPLETE" if "INCOMPLETE" in severities
        else "PASSED"
    )
    summary["findings"] = [item.to_dict() for item in findings]
    return summary


def _duplicate_findings(
    attempts: list[dict[str, Any]],
    field: str,
    code: str,
    label: str,
) -> list[Finding]:
    found: dict[Any, list[int]] = {}
    for attempt in attempts:
        value = attempt.get(field)
        if value is not None:
            found.setdefault(value, []).append(int(attempt["index"]))
    return [
        Finding(
            "FAILED",
            code,
            "suite",
            f"{label} {value!r} is shared by attempts {indices}",
        )
        for value, indices in found.items()
        if len(indices) > 1
    ]


def _validate_canonical_media(
    *,
    suite_id: str,
    attempts: list[dict[str, Any]],
    canonical_episode: Path | None,
    canonical_video: Path | None,
    canonical_video_receipt: Path | None,
) -> tuple[dict[str, Any] | None, list[Finding]]:
    findings: list[Finding] = []
    supplied = (
        canonical_episode is not None,
        canonical_video is not None,
        canonical_video_receipt is not None,
    )
    if not all(supplied):
        for present, name in zip(
            supplied,
            ("canonical_episode", "canonical_video", "canonical_video_receipt"),
        ):
            if not present:
                _missing(
                    findings,
                    "CANONICAL_MEDIA_MISSING",
                    f"suite.canonical_media.{name}",
                    "qualification-suite/v2 requires exactly one canonical media pair bound to one attempt",
                )
        return None, findings

    assert canonical_episode is not None
    assert canonical_video is not None
    assert canonical_video_receipt is not None
    selected_path = canonical_episode.resolve()
    selected = next(
        (item for item in attempts if Path(str(item["path"])).resolve() == selected_path),
        None,
    )
    if selected is None:
        _failed(
            findings,
            "CANONICAL_ATTEMPT_NOT_IN_SUITE",
            "suite.canonical_media.canonical_episode",
            "canonical episode must be one of the five exact sealed results",
        )
        return None, findings
    if selected.get("gate_status") != "PASSED":
        _failed(
            findings,
            "CANONICAL_ATTEMPT_NOT_PASSED",
            "suite.canonical_media.canonical_episode",
            "canonical media cannot bind a rejected or incomplete attempt",
        )

    video = canonical_video.resolve()
    receipt_path = canonical_video_receipt.resolve()
    if video.is_symlink():
        _failed(findings, "CANONICAL_VIDEO_SYMLINK", "suite.canonical_media.video", "canonical video may not be a symlink")
    if receipt_path.is_symlink():
        _failed(findings, "CANONICAL_RECEIPT_SYMLINK", "suite.canonical_media.receipt", "canonical receipt may not be a symlink")
    if not video.is_file() or video.stat().st_size <= 0:
        _missing(
            findings,
            "CANONICAL_VIDEO_MISSING",
            "suite.canonical_media.video",
            f"canonical video is missing or empty: {video}",
        )
    if not receipt_path.is_file():
        _missing(
            findings,
            "CANONICAL_RECEIPT_MISSING",
            "suite.canonical_media.receipt",
            f"canonical media receipt is missing: {receipt_path}",
        )
        return None, findings
    try:
        receipt = load_json(receipt_path)
    except (OSError, UnicodeError) as exc:
        _missing(findings, "CANONICAL_RECEIPT_UNREADABLE", "suite.canonical_media.receipt", str(exc))
        return None, findings
    except (json.JSONDecodeError, ValueError) as exc:
        _failed(findings, "CANONICAL_RECEIPT_INVALID", "suite.canonical_media.receipt", str(exc))
        return None, findings

    if "schema" not in receipt:
        _missing(findings, "CANONICAL_RECEIPT_SCHEMA_MISSING", "suite.canonical_media.receipt.schema", "canonical receipt schema is required")
    elif receipt.get("schema") != CANONICAL_MEDIA_RECEIPT_SCHEMA:
        _failed(findings, "CANONICAL_RECEIPT_SCHEMA_UNSUPPORTED", "suite.canonical_media.receipt.schema", f"expected {CANONICAL_MEDIA_RECEIPT_SCHEMA}")
    if "suite_id" not in receipt:
        _missing(findings, "CANONICAL_SUITE_ID_MISSING", "suite.canonical_media.receipt.suite_id", "suite ID binding is required")
    elif receipt.get("suite_id") != suite_id:
        _failed(findings, "CANONICAL_SUITE_ID_MISMATCH", "suite.canonical_media.receipt.suite_id", "receipt is not issued for this suite ID")
    if receipt.get("status") != "PASSED":
        if "status" not in receipt:
            _missing(findings, "CANONICAL_RECEIPT_STATUS_MISSING", "suite.canonical_media.receipt.status", "PASSED status is required")
        else:
            _failed(findings, "CANONICAL_RECEIPT_FAILED", "suite.canonical_media.receipt.status", repr(receipt.get("status")))
    if not _valid_sha(receipt.get("source_video_receipt_sha256")):
        _missing(findings, "CANONICAL_SOURCE_RECEIPT_HASH_MISSING", "suite.canonical_media.receipt.source_video_receipt_sha256", "immutable renderer receipt SHA-256 is required")
    if not _valid_sha(receipt.get("human_review_attestation_sha256")):
        _missing(findings, "CANONICAL_REVIEW_ATTESTATION_HASH_MISSING", "suite.canonical_media.receipt.human_review_attestation_sha256", "immutable human-review attestation SHA-256 is required")
    if "qualification" not in receipt:
        _missing(findings, "CANONICAL_QUALIFICATION_FLAG_MISSING", "suite.canonical_media.receipt.qualification", "qualification=false is required")
    elif receipt.get("qualification") is not False:
        _failed(findings, "CANONICAL_RECEIPT_RELABELLED", "suite.canonical_media.receipt.qualification", "media receipt may not claim qualification")
    if "real_robot_commands" not in receipt:
        _missing(findings, "REAL_ROBOT_FLAG_MISSING", "suite.canonical_media.receipt.real_robot_commands", "real_robot_commands=false is required")
    elif receipt.get("real_robot_commands") is not False:
        _failed(findings, "REAL_ROBOT_COMMANDS", "suite.canonical_media.receipt.real_robot_commands", "real robot commands are forbidden")
    if "published_command_topics" not in receipt:
        _missing(findings, "COMMAND_TOPICS_MISSING", "suite.canonical_media.receipt.published_command_topics", "empty command topic list is required")
    elif receipt.get("published_command_topics") != []:
        _failed(findings, "COMMAND_TOPICS_PUBLISHED", "suite.canonical_media.receipt.published_command_topics", "canonical replay must publish no command topics")
    authority = receipt.get("authority")
    if not isinstance(authority, dict):
        _missing(findings, "CANONICAL_AUTHORITY_MISSING", "suite.canonical_media.receipt.authority", "authority receipt is required")
        authority = {}
    elif (
        authority.get("source") != PHYSX_AUTHORITY
        or authority.get("replay") != "none_state_only_renderer"
        or authority.get("renderer_advances_physics") is not False
    ):
        _failed(findings, "CANONICAL_MEDIA_AUTHORITY_INVALID", "suite.canonical_media.receipt", "canonical media must be an offline replay of the selected PhysX attempt")

    expected_episode_hash = selected.get("sha256")
    state_hashes = selected.get("artifact_sha256_by_role")
    expected_state_hash = (
        state_hashes.get("state_command_trace")
        if isinstance(state_hashes, dict)
        else None
    )
    canonical_attempt = receipt.get("canonical_episode")
    if not isinstance(canonical_attempt, dict):
        _missing(findings, "CANONICAL_EPISODE_DESCRIPTOR_MISSING", "suite.canonical_media.receipt.canonical_episode", "canonical attempt descriptor is required")
        canonical_attempt = {}
    elif canonical_attempt.get("episode_id") != selected.get("episode_id"):
        _failed(findings, "CANONICAL_EPISODE_ID_MISMATCH", "suite.canonical_media.receipt.canonical_episode.episode_id", "receipt names a different attempt")
    if isinstance(canonical_attempt, dict) and not canonical_attempt:
        pass
    elif canonical_attempt.get("result_sha256") != expected_episode_hash:
        _failed(findings, "CANONICAL_EPISODE_HASH_MISMATCH", "suite.canonical_media.receipt.canonical_episode_result_sha256", "receipt does not bind the selected sealed episode result")
    if not _valid_sha(expected_state_hash):
        _missing(findings, "CANONICAL_STATE_TRACE_HASH_MISSING", "suite.canonical_media.canonical_episode", "selected attempt has no sealed state-command trace hash")
    elif canonical_attempt and canonical_attempt.get("state_command_trace_sha256") != expected_state_hash:
        _failed(findings, "CANONICAL_STATE_TRACE_HASH_MISMATCH", "suite.canonical_media.receipt.source_state_command_trace_sha256", "receipt does not bind the selected attempt state-command trace")

    media = receipt.get("video")
    actual_video_hash = sha256(video) if video.is_file() else None
    if not isinstance(media, dict):
        _missing(findings, "CANONICAL_VIDEO_DESCRIPTOR_MISSING", "suite.canonical_media.receipt.video", "video descriptor is required")
        media = {}
    claimed_video_hash = media.get("sha256")
    if not _valid_sha(claimed_video_hash):
        _missing(findings, "CANONICAL_VIDEO_HASH_MISSING", "suite.canonical_media.receipt.video.sha256", "video SHA-256 is required")
    elif actual_video_hash is not None and claimed_video_hash != actual_video_hash:
        _failed(findings, "CANONICAL_VIDEO_HASH_MISMATCH", "suite.canonical_media.receipt.video.sha256", "receipt does not bind the supplied video")
    if "size" not in media:
        _missing(findings, "CANONICAL_VIDEO_SIZE_MISSING", "suite.canonical_media.receipt.video.size", "video size is required")
    elif video.is_file() and media.get("size") != video.stat().st_size:
        _failed(findings, "CANONICAL_VIDEO_SIZE_MISMATCH", "suite.canonical_media.receipt.video.size", "video size differs from receipt")
    if "codec" not in media:
        _missing(findings, "CANONICAL_VIDEO_CODEC_MISSING", "suite.canonical_media.receipt.video.codec", "H.264 codec receipt is required")
    elif str(media.get("codec", "")).lower() != "h264":
        _failed(findings, "CANONICAL_VIDEO_CODEC_FAILED", "suite.canonical_media.receipt.video.codec", "H.264 is required")
    fps = media.get("fps")
    if isinstance(fps, bool) or not isinstance(fps, (int, float)):
        _missing(findings, "CANONICAL_VIDEO_FPS_MISSING", "suite.canonical_media.receipt.video.fps", "30 fps receipt is required")
    elif not math.isfinite(float(fps)) or abs(float(fps) - 30.0) > 1e-9:
        _failed(findings, "CANONICAL_VIDEO_FPS_FAILED", "suite.canonical_media.receipt.video.fps", "video must be exactly 30 fps")
    for name, minimum in (("width", 1280), ("height", 720)):
        value = media.get(name)
        if isinstance(value, bool) or not isinstance(value, int):
            _missing(findings, "CANONICAL_VIDEO_DIMENSION_MISSING", f"suite.canonical_media.receipt.video.{name}", f"{name} receipt is required")
        elif value < minimum:
            _failed(findings, "CANONICAL_VIDEO_DIMENSION_FAILED", f"suite.canonical_media.receipt.video.{name}", f"{name} must be at least {minimum}")
    frame_count = media.get("frame_count")
    if isinstance(frame_count, bool) or not isinstance(frame_count, int):
        _missing(findings, "CANONICAL_VIDEO_FRAME_COUNT_MISSING", "suite.canonical_media.receipt.video.frame_count", "encoded frame count is required")
    elif frame_count < 300:
        _failed(findings, "CANONICAL_VIDEO_FRAME_COUNT_FAILED", "suite.canonical_media.receipt.video.frame_count", "complete combination replay requires at least 300 frames")

    overlay = receipt.get("overlay")
    if not isinstance(overlay, dict):
        _missing(findings, "CANONICAL_HUD_MISSING", "suite.canonical_media.receipt.overlay", "burned qualification HUD receipt is required")
        overlay = {}
    elif overlay.get("schema") != COMBINATION_VIDEO_OVERLAY_SCHEMA or overlay.get("position") != "top_right":
        _failed(findings, "CANONICAL_HUD_IDENTITY_FAILED", "suite.canonical_media.receipt.overlay", "required top-right X2 combination HUD is absent")
    if overlay and "frame_count" not in overlay:
        _missing(findings, "CANONICAL_HUD_FRAME_COUNT_MISSING", "suite.canonical_media.receipt.overlay.frame_count", "HUD frame count is required")
    elif overlay and isinstance(frame_count, int) and overlay.get("frame_count") != frame_count:
        _failed(findings, "CANONICAL_HUD_FRAME_COVERAGE_FAILED", "suite.canonical_media.receipt.overlay.frame_count", "HUD must be present on every video frame")
    required_overlay_fields = {
        "scenario_phase_gait",
        "requested_and_effective_velocity",
        "measured_velocity",
        "left_and_right_foot_contact",
        "cpg_phase",
        "suspension",
        "sim_time",
        "global_lifecycle_rtf_estimate",
    }
    overlay_fields = overlay.get("fields")
    if not isinstance(overlay_fields, list):
        _missing(findings, "CANONICAL_HUD_FIELDS_MISSING", "suite.canonical_media.receipt.overlay.fields", "HUD field list is required")
    elif not all(isinstance(item, str) and item for item in overlay_fields):
        _failed(findings, "CANONICAL_HUD_FIELDS_INVALID", "suite.canonical_media.receipt.overlay.fields", "HUD fields must be non-empty strings")
    elif not required_overlay_fields.issubset(set(overlay_fields)):
        _failed(findings, "CANONICAL_HUD_FIELDS_FAILED", "suite.canonical_media.receipt.overlay.fields", "HUD omits required locomotion evidence")

    camera = receipt.get("camera")
    if not isinstance(camera, dict):
        _missing(findings, "CANONICAL_CAMERA_RECEIPT_MISSING", "suite.canonical_media.receipt.camera", "full-body camera receipt is required")
        camera = {}
    elif "mode" not in camera:
        _missing(findings, "CANONICAL_CAMERA_MODE_MISSING", "suite.canonical_media.receipt.camera.mode", "full-body follow camera profile is required")
    elif camera.get("mode") != "robot_follow_full_body_profile_v1":
        _failed(findings, "CANONICAL_CAMERA_MODE_FAILED", "suite.canonical_media.receipt.camera.mode", "full-body follow camera profile is required")
    if camera:
        if "left_menu_visible" not in camera:
            _missing(findings, "CANONICAL_CAMERA_MENU_FLAG_MISSING", "suite.canonical_media.receipt.camera.left_menu_visible", "left-menu visibility receipt is required")
        elif camera.get("left_menu_visible") is not False:
            _failed(findings, "CANONICAL_CAMERA_MENU_FAILED", "suite.canonical_media.receipt.camera.left_menu_visible", "left menu must be absent")
    for name in ("offset_xyz_m", "target_xyz_m"):
        if name not in camera:
            _missing(findings, "CANONICAL_CAMERA_POSE_MISSING", f"suite.canonical_media.receipt.camera.{name}", "finite camera vec3 is required")
            continue
        value = camera[name]
        if (
            not isinstance(value, (list, tuple))
            or len(value) != 3
            or any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value)
            or not all(math.isfinite(float(item)) for item in value)
        ):
            _failed(findings, "CANONICAL_CAMERA_POSE_INVALID", f"suite.canonical_media.receipt.camera.{name}", "camera vec3 is malformed or non-finite")
    verification = camera.get("visual_full_body_verification") if camera else None
    if camera and verification != "PASSED":
        if verification in {None, "pending_user_review"}:
            _missing(findings, "CANONICAL_FULL_BODY_REVIEW_PENDING", "suite.canonical_media.receipt.camera.visual_full_body_verification", "first/middle/last frames require explicit full-body review")
        else:
            _failed(findings, "CANONICAL_FULL_BODY_REVIEW_FAILED", "suite.canonical_media.receipt.camera.visual_full_body_verification", repr(verification))

    qa_frames = receipt.get("qa_frames")
    if not isinstance(qa_frames, list):
        _missing(findings, "CANONICAL_QA_FRAMES_MISSING", "suite.canonical_media.receipt.qa_frames", "first/middle/last QA descriptors are required")
    else:
        labels = [item.get("label") for item in qa_frames if isinstance(item, dict)]
        if labels != ["first", "middle", "last"]:
            _failed(findings, "CANONICAL_QA_FRAME_ORDER_FAILED", "suite.canonical_media.receipt.qa_frames", "QA labels must be first, middle, last")
        for index, item in enumerate(qa_frames):
            if not isinstance(item, dict) or not _valid_sha(item.get("sha256")):
                _missing(findings, "CANONICAL_QA_FRAME_HASH_MISSING", f"suite.canonical_media.receipt.qa_frames[{index}]", "QA frame SHA-256 is required")
            if not isinstance(item, dict) or isinstance(item.get("frame_index"), bool) or not isinstance(item.get("frame_index"), int):
                _missing(findings, "CANONICAL_QA_FRAME_INDEX_MISSING", f"suite.canonical_media.receipt.qa_frames[{index}]", "QA frame index is required")
        indices = [item.get("frame_index") for item in qa_frames if isinstance(item, dict)]
        if len(indices) == 3 and all(isinstance(value, int) and not isinstance(value, bool) for value in indices):
            if (
                not (indices[0] == 0 < indices[1] < indices[2])
                or (isinstance(frame_count, int) and indices[1] != frame_count // 2)
                or (isinstance(frame_count, int) and indices[2] != frame_count - 1)
            ):
                _failed(findings, "CANONICAL_QA_FRAME_INDEX_FAILED", "suite.canonical_media.receipt.qa_frames", "QA frames must cover exact first, middle and last indices")

    review = receipt.get("human_review")
    if not isinstance(review, dict):
        _missing(findings, "CANONICAL_REVIEW_MISSING", "suite.canonical_media.receipt.human_review", "explicit human full-body review is required")
    else:
        review_status = review.get("status")
        if review_status in {None, "PENDING"}:
            _missing(findings, "CANONICAL_REVIEW_PENDING", "suite.canonical_media.receipt.human_review.status", "human review has not been completed")
        elif review_status != "PASSED":
            _failed(findings, "CANONICAL_REVIEW_FAILED", "suite.canonical_media.receipt.human_review.status", repr(review_status))
        if review.get("method") != "human_review_first_middle_last_full_body":
            _failed(findings, "CANONICAL_REVIEW_METHOD_FAILED", "suite.canonical_media.receipt.human_review.method", "review must inspect exact first/middle/last frames")
        for name in ("reviewer", "reviewed_at"):
            if not isinstance(review.get(name), str) or not review[name].strip():
                _missing(findings, "CANONICAL_REVIEW_IDENTITY_MISSING", f"suite.canonical_media.receipt.human_review.{name}", "review identity and timestamp are required")
        reviewed_at = review.get("reviewed_at")
        if isinstance(reviewed_at, str) and reviewed_at.strip():
            try:
                parsed = datetime.fromisoformat(reviewed_at.replace("Z", "+00:00"))
            except ValueError:
                _failed(findings, "CANONICAL_REVIEW_TIME_INVALID", "suite.canonical_media.receipt.human_review.reviewed_at", "reviewed_at must be ISO-8601")
            else:
                if parsed.tzinfo is None or parsed.utcoffset() != timezone.utc.utcoffset(parsed):
                    _failed(findings, "CANONICAL_REVIEW_TIME_INVALID", "suite.canonical_media.receipt.human_review.reviewed_at", "reviewed_at must use UTC")
        for name in (
            "full_robot_visible",
            "head_visible",
            "hands_visible",
            "legs_and_feet_visible",
            "left_menu_hidden",
            "hud_readable",
        ):
            if name not in review:
                _missing(findings, "CANONICAL_REVIEW_GATE_MISSING", f"suite.canonical_media.receipt.human_review.{name}", "explicit visual review gate is required")
            elif review.get(name) is not True:
                _failed(findings, "CANONICAL_REVIEW_GATE_FAILED", f"suite.canonical_media.receipt.human_review.{name}", "visual review gate did not pass")

    summary = {
        "canonical_attempt_index": selected.get("index"),
        "canonical_episode_id": selected.get("episode_id"),
        "canonical_episode_path": str(selected_path),
        "canonical_episode_result_sha256": expected_episode_hash,
        "source_state_command_trace_sha256": expected_state_hash,
        "video": str(video),
        "video_sha256": actual_video_hash,
        "video_size": video.stat().st_size if video.is_file() else None,
        "receipt": str(receipt_path),
        "receipt_sha256": sha256(receipt_path),
        "gate_status": "FAILED" if any(item.severity == "FAILED" for item in findings) else "INCOMPLETE" if findings else "PASSED",
        "real_robot_commands_validated_false": receipt.get("real_robot_commands") is False,
    }
    return summary, findings


def build_suite(
    episode_paths: Iterable[Path],
    *,
    suite_id: str,
    suite_schema: str = SUITE_SCHEMA_V1,
    canonical_episode: Path | None = None,
    canonical_video: Path | None = None,
    canonical_video_receipt: Path | None = None,
) -> dict[str, Any]:
    if _SUITE_ID.fullmatch(suite_id) is None:
        raise ValueError("suite_id must be a stable identifier without path separators")
    if suite_schema not in {SUITE_SCHEMA_V1, SUITE_SCHEMA_V2}:
        raise ValueError(f"unsupported qualification suite schema: {suite_schema!r}")
    if suite_schema == SUITE_SCHEMA_V1 and any(
        item is not None
        for item in (canonical_episode, canonical_video, canonical_video_receipt)
    ):
        raise ValueError("canonical media arguments require qualification-suite/v2")
    evidence_schema = (
        EPISODE_EVIDENCE_SCHEMA_V2
        if suite_schema == SUITE_SCHEMA_V2
        else EPISODE_EVIDENCE_SCHEMA_V1
    )
    paths = [Path(item).resolve() for item in episode_paths]
    attempts = [
        _validate_episode(
            path,
            index + 1,
            expected_evidence_schema=evidence_schema,
        )
        for index, path in enumerate(paths)
    ]
    suite_findings: list[Finding] = []
    if len(paths) != EXPECTED_ATTEMPT_COUNT:
        _missing(
            suite_findings,
            "ATTEMPT_COUNT_INCOMPLETE",
            "suite",
            f"expected {EXPECTED_ATTEMPT_COUNT} attempts, received {len(paths)}",
        )
    for field, code, label in (
        ("path", "DUPLICATE_EPISODE_PATH", "episode path"),
        ("sha256", "DUPLICATE_EPISODE_HASH", "episode hash"),
        ("episode_id", "DUPLICATE_EPISODE_ID", "episode ID"),
        ("container_id", "DUPLICATE_CONTAINER_ID", "container ID"),
        ("reset_id", "DUPLICATE_RESET_ID", "reset ID"),
        ("engine_process_identity", "DUPLICATE_ENGINE_PROCESS", "engine identity"),
    ):
        suite_findings.extend(_duplicate_findings(attempts, field, code, label))

    # Separate controller identities need to be unique.  In-process controller
    # identity legitimately equals its engine identity within the same attempt.
    controller_values: dict[str, list[int]] = {}
    for attempt in attempts:
        value = attempt.get("controller_process_identity")
        if value is not None:
            controller_values.setdefault(str(value), []).append(int(attempt["index"]))
    for value, indices in controller_values.items():
        if len(indices) > 1:
            _failed(
                suite_findings,
                "DUPLICATE_CONTROLLER_PROCESS",
                "suite",
                f"controller identity {value!r} is shared by attempts {indices}",
            )

    complete_frozen = [attempt.get("frozen_inputs") for attempt in attempts]
    if attempts and all(isinstance(item, dict) for item in complete_frozen):
        baseline = complete_frozen[0]
        if any(item != baseline for item in complete_frozen[1:]):
            _failed(
                suite_findings,
                "FROZEN_INPUT_MISMATCH",
                "suite",
                "controller, profile, thresholds, assets or dynamics changed between attempts",
            )
    image_digests = [attempt.get("container_image_digest") for attempt in attempts]
    if attempts and all(isinstance(item, str) for item in image_digests):
        if len(set(image_digests)) != 1:
            _failed(
                suite_findings,
                "IMAGE_DIGEST_MISMATCH",
                "suite",
                "container image digest changed between attempts",
            )

    canonical_media = None
    if suite_schema == SUITE_SCHEMA_V2:
        canonical_media, canonical_findings = _validate_canonical_media(
            suite_id=suite_id,
            attempts=attempts,
            canonical_episode=canonical_episode,
            canonical_video=canonical_video,
            canonical_video_receipt=canonical_video_receipt,
        )
        suite_findings.extend(canonical_findings)

    all_findings = [
        Finding(**item)
        for attempt in attempts
        for item in attempt["findings"]
    ] + suite_findings
    severities = {item.severity for item in all_findings}
    gate_status = (
        "FAILED" if "FAILED" in severities
        else "INCOMPLETE" if "INCOMPLETE" in severities
        else "PASSED"
    )
    qualification = gate_status == "PASSED" and len(attempts) == EXPECTED_ATTEMPT_COUNT
    checks = {
        "exactly_five_attempts": len(attempts) == EXPECTED_ATTEMPT_COUNT,
        "all_attempts_passed": (
            len(attempts) == EXPECTED_ATTEMPT_COUNT
            and all(item["gate_status"] == "PASSED" for item in attempts)
        ),
        "unique_episode_identity": not any(
            item.code in {
                "DUPLICATE_EPISODE_PATH",
                "DUPLICATE_EPISODE_HASH",
                "DUPLICATE_EPISODE_ID",
            }
            for item in suite_findings
        ),
        "unique_container_and_process_lifecycle": not any(
            item.code in {
                "DUPLICATE_CONTAINER_ID",
                "DUPLICATE_RESET_ID",
                "DUPLICATE_ENGINE_PROCESS",
                "DUPLICATE_CONTROLLER_PROCESS",
            }
            for item in suite_findings
        ),
        "frozen_inputs_identical": not any(
            item.code == "FROZEN_INPUT_MISMATCH" for item in suite_findings
        ),
        "container_image_identical": not any(
            item.code == "IMAGE_DIGEST_MISMATCH" for item in suite_findings
        ),
        "real_robot_commands_false": not any(
            item.code in {"REAL_ROBOT_COMMANDS", "REAL_ROBOT_FLAG_MISSING"}
            for item in all_findings
        ),
    }
    if suite_schema == SUITE_SCHEMA_V2:
        checks["canonical_media_passed"] = (
            canonical_media is not None
            and canonical_media.get("gate_status") == "PASSED"
        )
    return {
        "schema": suite_schema,
        "suite_id": suite_id,
        "execution_status": "COMPLETED",
        "gate_status": gate_status,
        "status": gate_status,
        "decision": (
            "X2_PHYSX_LOCOMOTION_QUALIFIED"
            if qualification
            else "X2_PHYSX_LOCOMOTION_REJECTED"
            if gate_status == "FAILED"
            else "QUALIFICATION_EVIDENCE_INCOMPLETE"
        ),
        "expected_attempt_count": EXPECTED_ATTEMPT_COUNT,
        "observed_attempt_count": len(attempts),
        "qualification": qualification,
        "functional_equivalence": qualification,
        "physics_authority": PHYSX_AUTHORITY,
        "checks": checks,
        "attempts": attempts,
        **({"canonical_media": canonical_media} if suite_schema == SUITE_SCHEMA_V2 else {}),
        "findings": [item.to_dict() for item in all_findings],
        "published_command_topics": [],
        "real_robot_commands": False,
        "limitations": [] if qualification else [
            "No individual development episode is relabelled as qualification.",
            "Missing evidence yields INCOMPLETE; explicit gate failure, identity reuse or hash mismatch yields FAILED.",
        ],
    }


def atomic_write(path: Path, payload: dict[str, Any]) -> None:
    path = path.resolve()
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, 0o444)
    finally:
        if temporary.exists():
            temporary.unlink()
