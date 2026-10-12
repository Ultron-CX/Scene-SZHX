#!/usr/bin/env python3
"""Create an immutable metric-level MuJoCo/PhysX locomotion A/B receipt."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re


SCHEMA = "sceneforge/x2-locomotion-ab/v2"
_SHA256 = re.compile(r"[0-9a-fA-F]{64}")
EXPECTED_JOINT_COUNT = 31
STABLE_STOP_WINDOW_MIN_S = 1.90
STABLE_STOP_WINDOW_MAX_S = 2.10
STABLE_STOP_MIN_SAMPLE_COUNT = 100
REQUIRED_FROZEN_HASH_KEYS = frozenset({
    "mujoco_model_sha256",
    "mujoco_stand_policy_sha256",
    "mujoco_walk_policy_sha256",
    "mujoco_joint_limit_source_sha256",
    "physx_stand_policy_sha256",
    "physx_walk_policy_sha256",
    "physx_joint_limit_source_sha256",
})


SEALED_INPUT_CHECKS = (
    "completed_profile",
    "no_fall",
    "no_nan_or_inf",
    "zero_joint_limit_violations",
    "full_joint_state_and_target_within_limits",
    "bounded_policy_position_target_saturation",
    "final_linear_speed",
    "final_angular_speed",
    "sustained_stable_stop",
)


def _reject_nonfinite_json(value: str) -> None:
    raise ValueError(f"non-finite JSON constant is forbidden: {value}")


def load_json(path: Path) -> dict:
    payload = json.loads(
        path.read_text(encoding="utf-8"),
        parse_constant=_reject_nonfinite_json,
    )
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return payload


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _optional_nested_sha256(payload: dict, path: tuple[str, ...]) -> str | None:
    value: object = payload
    for name in path:
        if not isinstance(value, dict) or name not in value:
            return None
        value = value[name]
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise ValueError(f"{'.'.join(path)} must be a SHA-256 hex digest")
    return value.lower()


def frozen_input_hashes(mujoco: dict, physx_payload: dict, physx: dict) -> dict[str, str]:
    """Retain all immutable locomotion identities available in both inputs."""

    locators = {
        "mujoco_model_sha256": (mujoco, ("runtime", "model_sha256")),
        "mujoco_stand_policy_sha256": (
            mujoco, ("runtime", "stand_model_sha256")
        ),
        "mujoco_walk_policy_sha256": (
            mujoco, ("runtime", "walk_model_sha256")
        ),
        "mujoco_joint_limit_source_sha256": (
            mujoco, ("parameters", "joint_limit_source_sha256")
        ),
        "physx_stand_policy_sha256": (physx, ("model", "stand_sha256")),
        "physx_walk_policy_sha256": (physx, ("model", "walk_sha256")),
        "physx_joint_limit_source_sha256": (
            physx, ("parameters", "joint_limit_source_sha256")
        ),
    }
    result = {}
    for name, (payload, path) in locators.items():
        value = _optional_nested_sha256(payload, path)
        if value is not None:
            result[name] = value

    source_manifest = physx_payload.get("source_manifest")
    if source_manifest is not None:
        if not isinstance(source_manifest, list) or not source_manifest:
            raise ValueError("PhysX source_manifest must be a non-empty array")
        canonical = []
        for index, item in enumerate(source_manifest):
            if not isinstance(item, dict) or not isinstance(item.get("path"), str):
                raise ValueError(f"PhysX source_manifest[{index}] is malformed")
            digest = item.get("sha256")
            if not isinstance(digest, str) or _SHA256.fullmatch(digest) is None:
                raise ValueError(
                    f"PhysX source_manifest[{index}].sha256 must be a SHA-256 digest"
                )
            canonical.append((item["path"], digest.lower()))
        encoded = json.dumps(sorted(canonical), separators=(",", ":")).encode("utf-8")
        result["physx_source_manifest_sha256"] = hashlib.sha256(encoded).hexdigest()
    missing = sorted(REQUIRED_FROZEN_HASH_KEYS - set(result))
    if missing:
        raise ValueError(f"input results lack frozen identity hashes: {missing}")
    return result


def load_result(path: Path, expected_authority: str) -> tuple[dict, dict]:
    payload = load_json(path)
    result = payload.get("capture", payload)
    if not isinstance(result, dict):
        raise ValueError(f"input capture must be an object: {path}")
    if payload.get("status") != "PASSED" or result.get("status") != "PASSED":
        raise ValueError(f"input result is not a passed development sample: {path}")
    for layer_name, layer in (("envelope", payload), ("capture", result)):
        if layer.get("real_robot_commands") is not False:
            raise ValueError(
                f"{layer_name} real_robot_commands must be false in {path}"
            )
        if layer.get("qualification") is not False:
            raise ValueError(
                f"{layer_name} development input was relabelled as qualification: {path}"
            )
        if layer.get("published_command_topics") != []:
            raise ValueError(
                f"{layer_name} input published command transport in {path}"
            )
    if result.get("physics_authority") != expected_authority:
        raise ValueError(f"unexpected physics authority in {path}")
    checks = result.get("checks")
    if not isinstance(checks, dict):
        raise ValueError(f"input result has no explicit checks object: {path}")
    missing = [name for name in SEALED_INPUT_CHECKS if name not in checks]
    if missing:
        raise ValueError(f"input result lacks sealed checks {missing}: {path}")
    failed = [name for name in SEALED_INPUT_CHECKS if checks.get(name) is not True]
    if failed:
        raise ValueError(f"input result failed sealed checks {failed}: {path}")
    return payload, result


def finite_metric(result: dict, name: str) -> float:
    value = float(result["metrics"][name])
    if not math.isfinite(value):
        raise ValueError(f"metric {name} is not finite")
    return value


def finite_nested_metric(result: dict, group: str, name: str) -> float:
    metrics = result.get("metrics")
    nested = metrics.get(group) if isinstance(metrics, dict) else None
    if not isinstance(nested, dict) or name not in nested:
        raise ValueError(f"metric {group}.{name} is missing")
    value = float(nested[name])
    if not math.isfinite(value):
        raise ValueError(f"metric {group}.{name} is not finite")
    return value


def finite_integer(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be an integer")
    numeric = float(value)
    if not math.isfinite(numeric) or not numeric.is_integer():
        raise ValueError(f"{name} must be an integer")
    return int(numeric)


def joint_coverage_facts(result: dict) -> dict:
    parameters = result.get("parameters")
    if not isinstance(parameters, dict):
        raise ValueError("input result parameters are missing")
    names = result.get("joint_names")
    name_count = None
    if names is not None:
        if not isinstance(names, list) or not all(isinstance(name, str) for name in names):
            raise ValueError("input result joint_names are invalid")
        if len(names) != len(set(names)):
            raise ValueError("input result joint_names are not unique")
        name_count = len(names)
    articulation_count = parameters.get("articulation_joint_count")
    if articulation_count is not None:
        articulation_count = finite_integer(
            articulation_count, "parameters.articulation_joint_count"
        )
    if name_count is not None and articulation_count is not None and name_count != articulation_count:
        raise ValueError("joint_names and articulation_joint_count disagree")
    state_count = name_count if name_count is not None else articulation_count
    if state_count is None:
        raise ValueError("input result has no authoritative joint-state count")

    position_limits = parameters.get("joint_position_limits_rad")
    velocity_limits = parameters.get("joint_velocity_limits_rps")
    if not isinstance(position_limits, dict) or not isinstance(velocity_limits, dict):
        raise ValueError("input result has no full joint-limit maps")
    if names is not None:
        name_set = set(names)
        if set(position_limits) != name_set or set(velocity_limits) != name_set:
            raise ValueError("joint_names and joint-limit map coverage disagree")
    target_count = len(position_limits)
    return {
        "joint_state_count": state_count,
        # The applied-target audit iterates the complete position-limit map.
        "joint_target_count": target_count,
        "applied_position_target_count": target_count,
        "joint_position_limit_count": len(position_limits),
        "joint_velocity_limit_count": len(velocity_limits),
    }


LEG_JOINT_NAMES = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
)


def _missing_trace(path: Path) -> dict:
    return {
        "available": False,
        "reason": "trace_missing",
        "expected_path": str(path.resolve()),
    }


def _loaded_trace(path: Path, joint_names: object, samples: list[dict]) -> dict:
    if not isinstance(joint_names, list) or not all(
        isinstance(name, str) for name in joint_names
    ):
        raise ValueError(f"phase trace has invalid joint_names: {path}")
    return {
        "available": True,
        "joint_names": tuple(joint_names),
        "samples": samples,
        "trace": {
            "path": str(path.resolve()),
            "size": path.stat().st_size,
            "sha256": sha256(path),
        },
    }


def _load_phase_samples(
    result_path: Path,
    result: dict,
    *,
    direct_physx_result: bool = False,
) -> dict:
    authority = result["physics_authority"]
    if authority == "google_mujoco_3_3_5":
        path = result_path.parent / "trace.json"
        if not path.is_file():
            return _missing_trace(path)
        trace = load_json(path)
        if trace.get("physics_authority") != authority or trace.get("real_robot_commands") is not False:
            raise ValueError("MuJoCo trace authority/safety mismatch")
        samples = trace.get("samples")
        if not isinstance(samples, list):
            raise ValueError("MuJoCo phase trace samples are missing")
        return _loaded_trace(path, result.get("joint_names"), samples)
    if authority == "genie_sim_isaac_physx":
        path = result_path.parent / (
            "state-command-trace.json"
            if direct_physx_result else "gait1-trace.json"
        )
        if not path.is_file():
            return _missing_trace(path)
        actual_hash = sha256(path)
        if direct_physx_result:
            trace_receipt = result.get("trace")
            if not isinstance(trace_receipt, dict):
                raise ValueError("direct PhysX result lacks immutable trace receipt")
            claimed_path = trace_receipt.get("path")
            claimed_hash = trace_receipt.get("sha256")
            if not isinstance(claimed_path, str) or Path(claimed_path).name != path.name:
                raise ValueError("direct PhysX trace receipt does not name state-command-trace.json")
            if not isinstance(claimed_hash, str) or _SHA256.fullmatch(claimed_hash) is None:
                raise ValueError("direct PhysX trace receipt has no valid SHA-256")
            if claimed_hash.lower() != actual_hash:
                raise ValueError("direct PhysX sibling trace SHA-256 mismatch")
        trace = load_json(path)
        if trace.get("physics_authority") != authority or trace.get("real_robot_commands") is not False:
            raise ValueError("PhysX trace authority/safety mismatch")
        physics = trace.get("physics_samples")
        bases = trace.get("base_samples")
        if not isinstance(physics, list) or not isinstance(bases, list):
            raise ValueError("PhysX phase/state trace arrays are missing")
        if len(physics) != len(bases):
            raise ValueError("PhysX phase/state trace lengths differ")
        samples = []
        for state, base in zip(physics, bases):
            if state["physics_sequence"] != base["physics_sequence"]:
                raise ValueError("PhysX phase/state sequences differ")
            samples.append({
                "sim_time": state["sim_time"],
                "phase": state["phase"],
                "joint_position": state["position"],
                "cpg_phase_rad": base.get("cpg_phase_rad"),
            })
        loaded = _loaded_trace(path, trace.get("joint_names"), samples)
        # Reuse the hash already computed for the direct immutable binding.
        if direct_physx_result:
            loaded["trace"]["sha256"] = actual_hash
        return loaded
    raise ValueError(f"unsupported authority {authority}")


def _phase_profile(joint_names: tuple[str, ...], samples: list[dict], bins: int = 72) -> dict[str, list[float]] | None:
    walk = [
        row for row in samples
        if row.get("phase") == "GAIT1_FORWARD" and row.get("cpg_phase_rad") is not None
    ]
    if not walk:
        return None
    start = float(walk[0]["sim_time"]) + 2.0
    end = float(walk[-1]["sim_time"]) - 1.0
    selected = [row for row in walk if start <= float(row["sim_time"]) <= end]
    indices = [joint_names.index(name) for name in LEG_JOINT_NAMES]
    accumulators = [[[0.0, 0] for _ in LEG_JOINT_NAMES] for _ in range(bins)]
    for row in selected:
        phase = math.atan2(math.sin(float(row["cpg_phase_rad"])), math.cos(float(row["cpg_phase_rad"])))
        bin_index = min(bins - 1, int((phase + math.pi) / (2.0 * math.pi) * bins))
        positions = row["joint_position"]
        for output_index, joint_index in enumerate(indices):
            value = float(positions[joint_index])
            if not math.isfinite(value):
                raise ValueError("joint phase trace contains NaN or Inf")
            accumulators[bin_index][output_index][0] += value
            accumulators[bin_index][output_index][1] += 1
    if any(any(count == 0 for _, count in row) for row in accumulators):
        return None
    return {
        name: [row[index][0] / row[index][1] for row in accumulators]
        for index, name in enumerate(LEG_JOINT_NAMES)
    }


def phase_aligned_leg_joint_metrics(
    mujoco_path: Path,
    physx_path: Path,
    mujoco: dict,
    physx: dict,
    *,
    direct_physx_result: bool = False,
) -> dict:
    m_loaded = _load_phase_samples(mujoco_path, mujoco)
    p_loaded = _load_phase_samples(
        physx_path,
        physx,
        direct_physx_result=direct_physx_result,
    )
    def trace_input(loaded: dict) -> dict:
        if "trace" in loaded:
            return loaded["trace"]
        return {
            "available": False,
            "expected_path": loaded["expected_path"],
        }

    trace_inputs = {
        "mujoco": trace_input(m_loaded),
        "physx": trace_input(p_loaded),
    }
    if m_loaded["available"] is not True or p_loaded["available"] is not True:
        return {
            "available": False,
            "reason": "trace_missing",
            "trace_inputs": trace_inputs,
        }
    m_profile = _phase_profile(m_loaded["joint_names"], m_loaded["samples"])
    p_profile = _phase_profile(p_loaded["joint_names"], p_loaded["samples"])
    if m_profile is None or p_profile is None:
        return {
            "available": False,
            "reason": "phase_coverage_incomplete",
            "trace_inputs": trace_inputs,
        }
    per_joint = {}
    all_differences = []
    for name in LEG_JOINT_NAMES:
        differences = [p - m for m, p in zip(m_profile[name], p_profile[name])]
        all_differences.extend(differences)
        per_joint[name] = {
            "rms_rad": math.sqrt(sum(value * value for value in differences) / len(differences)),
            "peak_abs_rad": max(abs(value) for value in differences),
        }
    return {
        "available": True,
        "phase_bin_count": len(next(iter(m_profile.values()))),
        "joint_count": len(LEG_JOINT_NAMES),
        "aggregate_rms_rad": math.sqrt(sum(value * value for value in all_differences) / len(all_differences)),
        "peak_abs_rad": max(abs(value) for value in all_differences),
        "per_joint": per_joint,
        "trace_inputs": trace_inputs,
    }


def build_report(mujoco_path: Path, physx_path: Path) -> dict:
    _, mujoco = load_result(mujoco_path, "google_mujoco_3_3_5")
    physx_payload, physx = load_result(physx_path, "genie_sim_isaac_physx")
    mujoco_profile = mujoco["profile"]
    physx_profile = physx["profile"]
    profile_fields = (
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
    profile_matches = all(
        math.isclose(float(mujoco_profile[name]), float(physx_profile[name]), abs_tol=1e-12)
        for name in profile_fields
    )
    calibration_matches = mujoco.get("velocity_command_calibration") == physx.get("velocity_command_calibration")
    input_hashes = frozen_input_hashes(mujoco, physx_payload, physx)
    command_x = float(mujoco_profile["command_x_mps"])
    command_y = float(mujoco_profile["command_y_mps"])
    command_yaw = float(mujoco_profile["command_yaw_rps"])
    walk_s = float(mujoco_profile["walk_s"])
    if not all(math.isfinite(value) for value in (command_x, command_y, command_yaw, walk_s)):
        raise ValueError("command profile contains NaN or Inf")
    turning = abs(command_yaw) > 1e-9
    scenario = "turn_left" if command_yaw > 0.0 else "turn_right" if turning else "straight"

    def side(result: dict) -> dict:
        result_checks = result["checks"]
        target_safety = result.get("position_target_safety_envelope")
        if not isinstance(target_safety, dict):
            raise ValueError("position-target safety envelope is missing")
        if target_safety.get("schema") != "sceneforge/x2-position-target-safety-envelope/v1":
            raise ValueError("position-target safety envelope schema is unsupported")
        target_safety_contract = {
            "schema": target_safety["schema"],
            "maximum_saturation_ratio": float(
                target_safety["maximum_saturation_ratio"]
            ),
            "maximum_raw_limit_excess_rad": float(
                target_safety["maximum_raw_limit_excess_rad"]
            ),
        }
        if not all(
            math.isfinite(value) and value >= 0.0
            for key, value in target_safety_contract.items()
            if key != "schema"
        ):
            raise ValueError("position-target safety envelope contains invalid limits")
        coverage = joint_coverage_facts(result)
        state_violation_count = finite_integer(
            result["metrics"]["joint_limit_violation_count"],
            "metrics.joint_limit_violation_count",
        )
        applied_target_violation_count = finite_integer(
            result["metrics"]["position_target_limit_violation_count"],
            "metrics.position_target_limit_violation_count",
        )
        state_maximum_excess = finite_metric(result, "maximum_joint_limit_excess_rad")
        applied_target_maximum_excess = finite_metric(
            result, "maximum_position_target_limit_excess_rad"
        )
        stable_stop_window_duration = finite_nested_metric(
            result, "stable_stop", "observed_duration_s"
        )
        stable_stop_sample_count = finite_integer(
            result["metrics"]["stable_stop"]["sample_count"],
            "metrics.stable_stop.sample_count",
        )
        return {
            "walk_body_forward_distance_m": finite_metric(result, "walk_body_forward_distance_m"),
            "walk_local_forward_displacement_m": finite_metric(result, "walk_local_forward_displacement_m"),
            "walk_local_cross_track_m": finite_metric(result, "walk_local_cross_track_m"),
            "walk_local_cross_track_per_forward_m": finite_metric(
                result, "walk_local_cross_track_per_forward_m"
            ),
            "minimum_base_height_m": finite_metric(result, "minimum_base_height_m"),
            "maximum_tilt_rad": finite_metric(result, "maximum_tilt_rad"),
            "left_support_slip_p95_mps": finite_metric(result, "left_support_slip_p95_mps"),
            "right_support_slip_p95_mps": finite_metric(result, "right_support_slip_p95_mps"),
            "left_steady_support_slip_p95_mps": finite_metric(result, "left_steady_support_slip_p95_mps"),
            "right_steady_support_slip_p95_mps": finite_metric(result, "right_steady_support_slip_p95_mps"),
            "yaw_drift_rad_per_forward_m": finite_metric(result, "walk_yaw_drift_rad_per_forward_m"),
            "steady_forward_speed_mps": finite_metric(result, "steady_forward_speed_mps"),
            "steady_forward_speed_relative_error": finite_metric(result, "steady_forward_speed_relative_error"),
            "walk_turn_angle_rad": finite_metric(result, "walk_turn_angle_rad"),
            "steady_yaw_rate_rps": finite_metric(result, "steady_yaw_rate_rps"),
            "steady_yaw_rate_relative_error": finite_metric(
                result, "steady_yaw_rate_relative_error"
            ),
            "stop_distance_xy_m": finite_metric(result, "stop_distance_xy_m"),
            "effort_saturation_ratio": finite_metric(result, "effort_saturation_ratio"),
            "final_linear_speed_mps": finite_metric(result, "final_linear_speed_mps"),
            "final_angular_speed_rps": finite_metric(result, "final_angular_speed_rps"),
            "joint_limit_violation_count": float(state_violation_count),
            "maximum_joint_limit_excess_rad": state_maximum_excess,
            "position_target_limit_violation_count": float(
                applied_target_violation_count
            ),
            "maximum_position_target_limit_excess_rad": (
                applied_target_maximum_excess
            ),
            "maximum_joint_velocity_ratio": finite_metric(
                result, "maximum_joint_velocity_ratio"
            ),
            "maximum_joint_velocity_exceedance_duration_s": finite_metric(
                result, "maximum_joint_velocity_exceedance_duration_s"
            ),
            "raw_policy_position_target_sample_count": finite_metric(
                result, "raw_policy_position_target_sample_count"
            ),
            "raw_policy_position_target_saturation_count": finite_metric(
                result, "raw_policy_position_target_saturation_count"
            ),
            "raw_policy_position_target_saturation_ratio": finite_metric(
                result, "raw_policy_position_target_saturation_ratio"
            ),
            "maximum_raw_policy_position_target_limit_excess_rad": finite_metric(
                result, "maximum_raw_policy_position_target_limit_excess_rad"
            ),
            "position_target_safety_contract": target_safety_contract,
            "stable_stop": {
                "linear_speed_p95_mps": finite_nested_metric(
                    result, "stable_stop", "linear_speed_p95_mps"
                ),
                "angular_speed_p95_rps": finite_nested_metric(
                    result, "stable_stop", "angular_speed_p95_rps"
                ),
                "maximum_linear_speed_mps": finite_nested_metric(
                    result, "stable_stop", "maximum_linear_speed_mps"
                ),
                "maximum_angular_speed_rps": finite_nested_metric(
                    result, "stable_stop", "maximum_angular_speed_rps"
                ),
                "planar_drift_m": finite_nested_metric(
                    result, "stable_stop", "planar_drift_m"
                ),
                "observed_duration_s": stable_stop_window_duration,
                "sample_count": stable_stop_sample_count,
            },
            "sealed_input_checks": {
                name: result_checks[name] for name in SEALED_INPUT_CHECKS
            },
            "sealed_facts": {
                **coverage,
                "joint_limit_violation_count": state_violation_count,
                "maximum_joint_limit_excess_rad": state_maximum_excess,
                "joint_position_limit_violation_count": state_violation_count,
                "maximum_joint_position_limit_excess_rad": state_maximum_excess,
                "position_target_limit_violation_count": (
                    applied_target_violation_count
                ),
                "maximum_position_target_limit_excess_rad": (
                    applied_target_maximum_excess
                ),
                "applied_position_target_limit_violation_count": (
                    applied_target_violation_count
                ),
                "maximum_applied_position_target_limit_excess_rad": (
                    applied_target_maximum_excess
                ),
                "stable_stop_window_duration_s": stable_stop_window_duration,
                "stable_stop_sample_count": stable_stop_sample_count,
            },
        }

    m = side(mujoco)
    p = side(physx)
    joint_metrics = phase_aligned_leg_joint_metrics(
        mujoco_path,
        physx_path,
        mujoco,
        physx,
        direct_physx_result=physx_payload is physx,
    )
    speed_delta = abs(p["steady_forward_speed_mps"] - m["steady_forward_speed_mps"])
    body_distance_tolerance = max(
        0.1, 0.10 * abs(m["walk_body_forward_distance_m"])
    )
    stop_distance_tolerance = max(0.1, 0.20 * abs(m["stop_distance_xy_m"]))
    checks = {
        "same_command_profile": profile_matches,
        "same_velocity_command_calibration": calibration_matches,
        "same_stand_policy_hash": (
            input_hashes["mujoco_stand_policy_sha256"]
            == input_hashes["physx_stand_policy_sha256"]
        ),
        "same_walk_policy_hash": (
            input_hashes["mujoco_walk_policy_sha256"]
            == input_hashes["physx_walk_policy_sha256"]
        ),
        "same_joint_limit_source_hash": (
            input_hashes["mujoco_joint_limit_source_sha256"]
            == input_hashes["physx_joint_limit_source_sha256"]
        ),
        "same_position_target_safety_envelope": (
            m["position_target_safety_contract"]
            == p["position_target_safety_contract"]
        ),
        "authority_pair_is_mujoco_and_physx": True,
        "both_development_samples_passed": True,
        "mujoco_no_fall": m["sealed_input_checks"]["no_fall"] is True,
        "physx_no_fall": p["sealed_input_checks"]["no_fall"] is True,
        "mujoco_no_nan_or_inf": m["sealed_input_checks"]["no_nan_or_inf"] is True,
        "physx_no_nan_or_inf": p["sealed_input_checks"]["no_nan_or_inf"] is True,
        "mujoco_full_joint_state_and_target_within_limits": (
            m["sealed_input_checks"]["full_joint_state_and_target_within_limits"] is True
            and m["sealed_input_checks"]["zero_joint_limit_violations"] is True
            and m["joint_limit_violation_count"] == 0.0
            and m["maximum_joint_limit_excess_rad"] <= 1e-4
            and m["position_target_limit_violation_count"] == 0.0
            and m["maximum_position_target_limit_excess_rad"] <= 1e-4
        ),
        "physx_full_joint_state_and_target_within_limits": (
            p["sealed_input_checks"]["full_joint_state_and_target_within_limits"] is True
            and p["sealed_input_checks"]["zero_joint_limit_violations"] is True
            and p["joint_limit_violation_count"] == 0.0
            and p["maximum_joint_limit_excess_rad"] <= 1e-4
            and p["position_target_limit_violation_count"] == 0.0
            and p["maximum_position_target_limit_excess_rad"] <= 1e-4
        ),
        "mujoco_full_31_joint_coverage": all(
            m["sealed_facts"][name] == EXPECTED_JOINT_COUNT
            for name in (
                "joint_state_count",
                "joint_target_count",
                "applied_position_target_count",
                "joint_position_limit_count",
                "joint_velocity_limit_count",
            )
        ),
        "physx_full_31_joint_coverage": all(
            p["sealed_facts"][name] == EXPECTED_JOINT_COUNT
            for name in (
                "joint_state_count",
                "joint_target_count",
                "applied_position_target_count",
                "joint_position_limit_count",
                "joint_velocity_limit_count",
            )
        ),
        "mujoco_policy_position_target_saturation_bounded": (
            m["sealed_input_checks"]["bounded_policy_position_target_saturation"]
            is True
            and m["raw_policy_position_target_sample_count"] > 0.0
            and m["raw_policy_position_target_saturation_ratio"]
            <= m["position_target_safety_contract"]["maximum_saturation_ratio"]
            and m["maximum_raw_policy_position_target_limit_excess_rad"]
            <= m["position_target_safety_contract"][
                "maximum_raw_limit_excess_rad"
            ]
        ),
        "physx_policy_position_target_saturation_bounded": (
            p["sealed_input_checks"]["bounded_policy_position_target_saturation"]
            is True
            and p["raw_policy_position_target_sample_count"] > 0.0
            and p["raw_policy_position_target_saturation_ratio"]
            <= p["position_target_safety_contract"]["maximum_saturation_ratio"]
            and p["maximum_raw_policy_position_target_limit_excess_rad"]
            <= p["position_target_safety_contract"][
                "maximum_raw_limit_excess_rad"
            ]
        ),
        "physx_joint_velocity_peak_no_worse_than_mujoco_reference": (
            p["maximum_joint_velocity_ratio"]
            <= max(1.0, m["maximum_joint_velocity_ratio"])
        ),
        "physx_joint_velocity_exceedance_not_longer_than_mujoco_reference": (
            p["maximum_joint_velocity_exceedance_duration_s"]
            <= m["maximum_joint_velocity_exceedance_duration_s"] + 0.002
        ),
        "both_walk_body_forward_distance_observed": (
            m["walk_body_forward_distance_m"] >= 0.50
            and p["walk_body_forward_distance_m"] >= 0.50
        ),
        "walk_body_forward_distance_between_engines": (
            abs(
                p["walk_body_forward_distance_m"]
                - m["walk_body_forward_distance_m"]
            ) <= body_distance_tolerance
        ),
        "steady_forward_speed_between_engines": speed_delta <= max(
            0.05, 0.10 * abs(m["steady_forward_speed_mps"])
        ),
        "mujoco_steady_support_slip": max(m["left_steady_support_slip_p95_mps"], m["right_steady_support_slip_p95_mps"]) <= 0.03,
        "physx_steady_support_slip": max(p["left_steady_support_slip_p95_mps"], p["right_steady_support_slip_p95_mps"]) <= 0.03,
        "mujoco_steady_speed_vs_command": m["steady_forward_speed_relative_error"] <= 0.15,
        "physx_steady_speed_vs_command": p["steady_forward_speed_relative_error"] <= 0.15,
        "stop_distance_between_engines": (
            abs(p["stop_distance_xy_m"] - m["stop_distance_xy_m"])
            <= stop_distance_tolerance
        ),
        "mujoco_stable_stop_p95": (
            m["stable_stop"]["linear_speed_p95_mps"] <= 0.05
            and m["stable_stop"]["angular_speed_p95_rps"] <= 0.05
        ),
        "physx_stable_stop_p95": (
            p["stable_stop"]["linear_speed_p95_mps"] <= 0.05
            and p["stable_stop"]["angular_speed_p95_rps"] <= 0.05
        ),
        "mujoco_stable_stop_maximum": (
            m["stable_stop"]["maximum_linear_speed_mps"] <= 0.05
            and m["stable_stop"]["maximum_angular_speed_rps"] <= 0.05
        ),
        "physx_stable_stop_maximum": (
            p["stable_stop"]["maximum_linear_speed_mps"] <= 0.05
            and p["stable_stop"]["maximum_angular_speed_rps"] <= 0.05
        ),
        "mujoco_stable_stop_drift": m["stable_stop"]["planar_drift_m"] <= 0.03,
        "physx_stable_stop_drift": p["stable_stop"]["planar_drift_m"] <= 0.03,
        "mujoco_stable_stop_window_valid": (
            STABLE_STOP_WINDOW_MIN_S
            <= m["sealed_facts"]["stable_stop_window_duration_s"]
            <= STABLE_STOP_WINDOW_MAX_S
            and m["sealed_facts"]["stable_stop_sample_count"]
            >= STABLE_STOP_MIN_SAMPLE_COUNT
        ),
        "physx_stable_stop_window_valid": (
            STABLE_STOP_WINDOW_MIN_S
            <= p["sealed_facts"]["stable_stop_window_duration_s"]
            <= STABLE_STOP_WINDOW_MAX_S
            and p["sealed_facts"]["stable_stop_sample_count"]
            >= STABLE_STOP_MIN_SAMPLE_COUNT
        ),
        "phase_aligned_leg_joint_rms": joint_metrics.get("available") is True and joint_metrics["aggregate_rms_rad"] <= 0.12,
        "phase_aligned_leg_joint_peak": joint_metrics.get("available") is True and joint_metrics["peak_abs_rad"] <= 0.25,
        "both_stopped": max(
            m["final_linear_speed_mps"], m["final_angular_speed_rps"],
            p["final_linear_speed_mps"], p["final_angular_speed_rps"],
        ) <= 0.05,
        "both_height_and_tilt_safe": (
            min(m["minimum_base_height_m"], p["minimum_base_height_m"]) >= 0.55
            and max(m["maximum_tilt_rad"], p["maximum_tilt_rad"]) <= 0.35
        ),
        "both_effort_saturation_bounded": max(
            m["effort_saturation_ratio"], p["effort_saturation_ratio"]
        ) <= 0.01,
        "real_robot_commands_false": True,
    }

    turn_metrics: dict[str, float | bool] | None = None
    if turning:
        expected_turn_angle = command_yaw * walk_s
        turn_angle_tolerance = max(0.1, 0.10 * abs(expected_turn_angle))
        yaw_rate_tolerance = max(0.025, 0.10 * abs(command_yaw))
        engine_angle_tolerance = max(0.1, 0.10 * abs(m["walk_turn_angle_rad"]))
        engine_rate_tolerance = max(0.025, 0.10 * abs(m["steady_yaw_rate_rps"]))
        turn_metrics = {
            "expected_walk_turn_angle_rad": expected_turn_angle,
            "walk_turn_angle_tolerance_rad": turn_angle_tolerance,
            "steady_yaw_rate_tolerance_rps": yaw_rate_tolerance,
            "engine_turn_angle_tolerance_rad": engine_angle_tolerance,
            "engine_steady_yaw_rate_tolerance_rps": engine_rate_tolerance,
        }
        checks.update({
            "mujoco_turn_direction": (
                m["walk_turn_angle_rad"] * command_yaw > 0.0
                and m["steady_yaw_rate_rps"] * command_yaw > 0.0
            ),
            "physx_turn_direction": (
                p["walk_turn_angle_rad"] * command_yaw > 0.0
                and p["steady_yaw_rate_rps"] * command_yaw > 0.0
            ),
            "mujoco_full_walk_turn_angle": (
                abs(m["walk_turn_angle_rad"] - expected_turn_angle)
                <= turn_angle_tolerance
            ),
            "physx_full_walk_turn_angle": (
                abs(p["walk_turn_angle_rad"] - expected_turn_angle)
                <= turn_angle_tolerance
            ),
            "mujoco_steady_yaw_rate": (
                abs(m["steady_yaw_rate_rps"] - command_yaw) <= yaw_rate_tolerance
            ),
            "physx_steady_yaw_rate": (
                abs(p["steady_yaw_rate_rps"] - command_yaw) <= yaw_rate_tolerance
            ),
            "turn_angle_between_engines": (
                abs(p["walk_turn_angle_rad"] - m["walk_turn_angle_rad"])
                <= engine_angle_tolerance
            ),
            "steady_yaw_rate_between_engines": (
                abs(p["steady_yaw_rate_rps"] - m["steady_yaw_rate_rps"])
                <= engine_rate_tolerance
            ),
        })
        remaining_required_metrics = [
            "opposite_turn_direction_and_pair_symmetry",
            "five_cold_start_sequences",
        ]
    else:
        checks.update({
            "mujoco_local_cross_track": abs(
                m["walk_local_cross_track_per_forward_m"]
            ) <= 0.05,
            "physx_local_cross_track": abs(
                p["walk_local_cross_track_per_forward_m"]
            ) <= 0.05,
            "mujoco_yaw_drift": m["yaw_drift_rad_per_forward_m"] <= 0.05,
            "physx_yaw_drift": p["yaw_drift_rad_per_forward_m"] <= 0.05,
        })
        remaining_required_metrics = [
            "paired_left_and_right_turning_angle_accuracy",
            "five_cold_start_sequences",
        ]

    common_metric_subset_passed = all(checks.values())
    if joint_metrics.get("available") is not True:
        remaining_required_metrics.insert(0, "phase_aligned_leg_joint_rms_and_peak")
    return {
        "schema": SCHEMA,
        "execution_status": "COMPLETED",
        "gate_status": "PASSED" if common_metric_subset_passed else "FAILED",
        "status": "PASSED" if common_metric_subset_passed else "FAILED",
        "decision": (
            "record_case_and_continue_equivalence_matrix"
            if common_metric_subset_passed else "dynamics_alignment_required"
        ),
        "qualification": False,
        "scenario": scenario,
        "case_gate_passed": common_metric_subset_passed,
        "common_metric_subset_passed": common_metric_subset_passed,
        "functional_equivalence": False,
        "remaining_required_metrics": remaining_required_metrics,
        "checks": checks,
        "command": {
            "x_mps": command_x,
            "y_mps": command_y,
            "yaw_rps": command_yaw,
            "walk_s": walk_s,
        },
        "velocity_command_calibration": (
            mujoco.get("velocity_command_calibration")
            if calibration_matches else None
        ),
        "frozen_input_hashes": input_hashes,
        "mujoco": m,
        "physx": p,
        "turn": turn_metrics,
        "tolerances": {
            "walk_body_forward_distance_between_engines_m": (
                body_distance_tolerance
            ),
            "steady_forward_speed_between_engines_mps": max(
                0.05, 0.10 * abs(m["steady_forward_speed_mps"])
            ),
            "stop_distance_between_engines_m": stop_distance_tolerance,
        },
        "phase_aligned_leg_joint_comparison": joint_metrics,
        "delta": {
            "steady_forward_speed_mps_abs": speed_delta,
            "walk_body_forward_distance_m_physx_minus_mujoco": (
                p["walk_body_forward_distance_m"] - m["walk_body_forward_distance_m"]
            ),
            "walk_local_forward_displacement_m_physx_minus_mujoco": (
                p["walk_local_forward_displacement_m"]
                - m["walk_local_forward_displacement_m"]
            ),
            "walk_local_cross_track_m_physx_minus_mujoco": (
                p["walk_local_cross_track_m"] - m["walk_local_cross_track_m"]
            ),
            "left_support_slip_mps_physx_minus_mujoco": p["left_support_slip_p95_mps"] - m["left_support_slip_p95_mps"],
            "right_support_slip_mps_physx_minus_mujoco": p["right_support_slip_p95_mps"] - m["right_support_slip_p95_mps"],
            "left_steady_support_slip_mps_physx_minus_mujoco": p["left_steady_support_slip_p95_mps"] - m["left_steady_support_slip_p95_mps"],
            "right_steady_support_slip_mps_physx_minus_mujoco": p["right_steady_support_slip_p95_mps"] - m["right_steady_support_slip_p95_mps"],
            "steady_forward_speed_mps_physx_minus_mujoco": p["steady_forward_speed_mps"] - m["steady_forward_speed_mps"],
            "walk_turn_angle_rad_physx_minus_mujoco": (
                p["walk_turn_angle_rad"] - m["walk_turn_angle_rad"]
            ),
            "steady_yaw_rate_rps_physx_minus_mujoco": (
                p["steady_yaw_rate_rps"] - m["steady_yaw_rate_rps"]
            ),
            "stop_distance_xy_m_physx_minus_mujoco": p["stop_distance_xy_m"] - m["stop_distance_xy_m"],
            "maximum_joint_velocity_ratio_physx_minus_mujoco": (
                p["maximum_joint_velocity_ratio"]
                - m["maximum_joint_velocity_ratio"]
            ),
            "raw_policy_position_target_saturation_count_physx_minus_mujoco": (
                p["raw_policy_position_target_saturation_count"]
                - m["raw_policy_position_target_saturation_count"]
            ),
            "maximum_raw_policy_position_target_limit_excess_rad_physx_minus_mujoco": (
                p["maximum_raw_policy_position_target_limit_excess_rad"]
                - m["maximum_raw_policy_position_target_limit_excess_rad"]
            ),
        },
        "inputs": {
            "mujoco": {"path": str(mujoco_path), "sha256": sha256(mujoco_path)},
            "physx": {"path": str(physx_path), "sha256": sha256(physx_path)},
        },
        "limitations": [
            "Distance and cross-track metrics are derived in the walk-entry/body frame; world X/Y displacement is not used for pass/fail.",
            "A single turning report does not establish left/right pair symmetry.",
            "Five cold-start sequences remain outside this single-case development comparison.",
            "The vendor MuJoCo reference itself exceeds the URDF nominal joint-velocity value transiently; this A/B checks that PhysX is no worse and does not resolve final qualification semantics.",
            "execution_status records whether comparison completed; gate_status/status reflect this case's metric gate, while functional_equivalence remains false until every planned metric and cold-start gate is present and passed.",
            "Small raw policy position-target excursions may be bounded by the identical auditable safety envelope in both engines; applied physics targets must still have zero limit violations.",
        ],
        "published_command_topics": [],
        "real_robot_commands": False,
    }


def atomic_write(path: Path, payload: dict) -> None:
    if path.exists():
        raise FileExistsError(path)
    path.parent.mkdir(parents=True, exist_ok=False)
    temporary = path.parent / ".result.json.tmp"
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            stream.write(
                json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n"
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mujoco", type=Path, required=True)
    parser.add_argument("--physx", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = build_report(args.mujoco.resolve(), args.physx.resolve())
    atomic_write(args.output.resolve(), report)
    print(args.output.resolve())
    return 0 if report["status"] == "PASSED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
