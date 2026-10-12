"""Seal one reviewed X2 combination replay as suite-v2 canonical media.

The renderer receipt remains immutable and may truthfully say that visual
review is pending.  After a human inspects the exact first/middle/last QA
frames, this module verifies a separate immutable attestation and emits a new
canonical receipt.  The resulting pair (video + canonical receipt) binds the
selected sealed episode result and its already-verified state-command trace.
"""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping

from .x2_locomotion_qualification import (
    CANONICAL_MEDIA_RECEIPT_SCHEMA,
    COMBINATION_VIDEO_OVERLAY_SCHEMA,
    EPISODE_EVIDENCE_SCHEMA_V2,
    PHYSX_AUTHORITY,
    SUITE_SCHEMA_V2,
    build_suite,
    load_json,
    sha256,
)


SOURCE_VIDEO_RECEIPT_SCHEMA = "sceneforge/x2-combination-video-receipt/v1"
REVIEW_ATTESTATION_SCHEMA = "sceneforge/x2-locomotion-full-body-review/v1"
REQUIRED_REVIEW_CHECKS = (
    "full_robot_visible",
    "head_visible",
    "hands_visible",
    "legs_and_feet_visible",
    "left_menu_hidden",
    "hud_readable",
)
REQUIRED_OVERLAY_FIELDS = (
    "scenario_phase_gait",
    "requested_and_effective_velocity",
    "measured_velocity",
    "left_and_right_foot_contact",
    "cpg_phase",
    "suspension",
    "sim_time",
    "global_lifecycle_rtf_estimate",
)


class CanonicalMediaError(RuntimeError):
    pass


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise CanonicalMediaError(message)


def _strict_utc(value: object) -> str:
    _require(isinstance(value, str) and bool(value.strip()), "reviewed_at is required")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise CanonicalMediaError("reviewed_at must be ISO-8601") from exc
    _require(parsed.tzinfo is not None and parsed.utcoffset() == timezone.utc.utcoffset(parsed), "reviewed_at must use UTC")
    return value


def _strict_vec3(value: object, name: str) -> list[float]:
    _require(isinstance(value, (list, tuple)) and len(value) == 3, f"{name} must be a vec3")
    _require(
        all(
            not isinstance(item, bool)
            and isinstance(item, (int, float))
            and math.isfinite(float(item))
            for item in value
        ),
        f"{name} must contain finite numbers",
    )
    return [float(item) for item in value]


def _qa_path(receipt_path: Path, raw_path: object) -> Path:
    _require(isinstance(raw_path, str) and bool(raw_path), "QA path is required")
    path = Path(raw_path)
    resolved = path.resolve() if path.is_absolute() else (receipt_path.parent / path).resolve()
    try:
        resolved.relative_to(receipt_path.parent.resolve())
    except ValueError as exc:
        raise CanonicalMediaError("QA frame escapes the renderer receipt directory") from exc
    _require(not resolved.is_symlink() and resolved.is_file(), f"QA frame is missing: {resolved}")
    return resolved


def _atomic_write(path: Path, payload: Mapping[str, Any]) -> None:
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
        temporary.unlink(missing_ok=True)


def build_full_body_review_attestation(
    *,
    video: Path,
    source_video_receipt: Path,
    reviewer: str,
    reviewed_at: str,
    checks: Mapping[str, bool],
) -> dict[str, Any]:
    video = video.resolve()
    source_video_receipt = source_video_receipt.resolve()
    _require(not video.is_symlink() and video.is_file() and video.stat().st_size > 0, "review video is missing, empty or a symlink")
    _require(not source_video_receipt.is_symlink() and source_video_receipt.is_file(), "renderer receipt is missing or a symlink")
    _require(isinstance(reviewer, str) and bool(reviewer.strip()), "reviewer is required")
    reviewed_at = _strict_utc(reviewed_at)
    source = load_json(source_video_receipt)
    _require(source.get("schema") == SOURCE_VIDEO_RECEIPT_SCHEMA, "renderer receipt schema mismatch")
    _require(source.get("status") == "PASSED", "renderer receipt did not pass")
    _require(source.get("qualification") is False and source.get("real_robot_commands") is False, "renderer receipt safety envelope failed")
    media = source.get("video")
    _require(isinstance(media, dict), "renderer video descriptor is missing")
    video_hash = sha256(video)
    _require(media.get("sha256") == video_hash and media.get("size") == video.stat().st_size, "renderer receipt does not bind review video")
    overlay = source.get("overlay")
    frame_count = overlay.get("frame_count") if isinstance(overlay, dict) else None
    _require(isinstance(frame_count, int) and not isinstance(frame_count, bool) and frame_count >= 300, "renderer receipt has no complete HUD frame count")
    qa_rows = source.get("qa_frames")
    _require(isinstance(qa_rows, list) and len(qa_rows) == 3, "first/middle/last QA receipts are required")
    expected = (("first", 0), ("middle", frame_count // 2), ("last", frame_count - 1))
    qa_hashes: dict[str, str] = {}
    for row, (label, frame_index) in zip(qa_rows, expected):
        _require(isinstance(row, dict) and row.get("label") == label and row.get("frame_index") == frame_index, f"QA {label} identity mismatch")
        path = _qa_path(source_video_receipt, row.get("path"))
        actual = sha256(path)
        _require(row.get("sha256") == actual and row.get("size") == path.stat().st_size, f"QA {label} hash or size mismatch")
        _require(path.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n", f"QA {label} is not PNG")
        qa_hashes[label] = actual
    _require(isinstance(checks, Mapping), "review checks are required")
    for name in REQUIRED_REVIEW_CHECKS:
        _require(checks.get(name) is True, f"review check was not explicitly confirmed: {name}")
    return {
        "schema": REVIEW_ATTESTATION_SCHEMA,
        "status": "PASSED",
        "video_sha256": video_hash,
        "source_video_receipt_sha256": sha256(source_video_receipt),
        "qa_frame_sha256": qa_hashes,
        "reviewer": reviewer,
        "reviewed_at": reviewed_at,
        "checks": {name: True for name in REQUIRED_REVIEW_CHECKS},
        "qualification": False,
        "real_robot_commands": False,
    }


def seal_full_body_review_attestation(
    *,
    video: Path,
    source_video_receipt: Path,
    reviewer: str,
    reviewed_at: str,
    checks: Mapping[str, bool],
    output: Path,
) -> dict[str, Any]:
    payload = build_full_body_review_attestation(
        video=video,
        source_video_receipt=source_video_receipt,
        reviewer=reviewer,
        reviewed_at=reviewed_at,
        checks=checks,
    )
    _atomic_write(output, payload)
    return payload


def build_canonical_media_receipt(
    *,
    suite_id: str,
    episode_result: Path,
    video: Path,
    source_video_receipt: Path,
    review_attestation: Path,
) -> dict[str, Any]:
    episode_result = episode_result.resolve()
    video = video.resolve()
    source_video_receipt = source_video_receipt.resolve()
    review_attestation = review_attestation.resolve()
    for name, path in (
        ("episode result", episode_result),
        ("video", video),
        ("source video receipt", source_video_receipt),
        ("review attestation", review_attestation),
    ):
        _require(not path.is_symlink() and path.is_file(), f"{name} is missing or a symlink: {path}")
    _require(video.stat().st_size > 0, "canonical video is empty")

    preflight = build_suite(
        [episode_result],
        suite_id="canonical-media-episode-preflight",
        suite_schema=SUITE_SCHEMA_V2,
    )
    attempt = preflight["attempts"][0]
    _require(attempt["gate_status"] == "PASSED", f"episode v2 preflight failed: {attempt['findings']}")
    _require(attempt.get("evidence_schema") == EPISODE_EVIDENCE_SCHEMA_V2, "canonical episode is not episode-evidence/v2")
    state_hash = attempt.get("artifact_sha256_by_role", {}).get("state_command_trace")
    _require(isinstance(state_hash, str) and len(state_hash) == 64, "sealed state-command trace SHA is missing")

    source = load_json(source_video_receipt)
    _require(source.get("schema") == SOURCE_VIDEO_RECEIPT_SCHEMA, "source video receipt schema mismatch")
    _require(source.get("status") == "PASSED", "source video receipt did not pass")
    _require(source.get("qualification") is False, "source video receipt was relabelled")
    _require(source.get("real_robot_commands") is False, "source video receipt is not real_robot_commands=false")
    _require(source.get("source_state_command_trace_sha256") == state_hash, "source video receipt binds a different state trace")
    _require(source.get("source_physics_authority") == PHYSX_AUTHORITY, "source video is not PhysX-authoritative")
    _require(source.get("replay_physics_authority") == "none_state_only_renderer", "source replay advanced another physics authority")
    _require(source.get("renderer_advances_physics") is False, "source renderer advanced physics")

    media = source.get("video")
    _require(isinstance(media, dict), "source video descriptor is missing")
    video_hash = sha256(video)
    _require(media.get("sha256") == video_hash and media.get("size") == video.stat().st_size, "source receipt does not bind the video")
    _require(str(media.get("codec", "")).lower() == "h264", "canonical video is not H.264")
    _require(media.get("fps") == 30.0, "canonical video is not exactly 30 fps")
    _require(isinstance(media.get("width"), int) and media["width"] >= 1280, "canonical video width is below 1280")
    _require(isinstance(media.get("height"), int) and media["height"] >= 720, "canonical video height is below 720")

    overlay = source.get("overlay")
    _require(isinstance(overlay, dict), "source HUD receipt is missing")
    _require(overlay.get("schema") == COMBINATION_VIDEO_OVERLAY_SCHEMA, "source HUD schema mismatch")
    _require(overlay.get("position") == "top_right", "source HUD is not top-right")
    overlay_fields = overlay.get("fields")
    _require(
        isinstance(overlay_fields, list)
        and all(isinstance(item, str) and item for item in overlay_fields)
        and set(REQUIRED_OVERLAY_FIELDS).issubset(set(overlay_fields)),
        "source HUD omits required locomotion fields",
    )
    frame_count = overlay.get("frame_count")
    _require(isinstance(frame_count, int) and not isinstance(frame_count, bool) and frame_count >= 300, "source replay has fewer than 300 HUD frames")

    camera = source.get("camera")
    _require(isinstance(camera, dict), "source camera receipt is missing")
    _require(camera.get("mode") == "robot_follow_full_body_profile_v1", "source camera is not the full-body profile")
    _require(camera.get("left_menu_visible") is False, "source replay includes the left menu")
    offset = _strict_vec3(camera.get("offset_xyz_m"), "camera.offset_xyz_m")
    target = _strict_vec3(camera.get("target_xyz_m"), "camera.target_xyz_m")

    qa_rows = source.get("qa_frames")
    _require(isinstance(qa_rows, list) and len(qa_rows) == 3, "source first/middle/last QA frames are missing")
    validated_qa = []
    expected_labels = ("first", "middle", "last")
    expected_indices = (0, frame_count // 2, frame_count - 1)
    for row, label, frame_index in zip(qa_rows, expected_labels, expected_indices):
        _require(isinstance(row, dict), f"QA {label} descriptor is invalid")
        _require(row.get("label") == label and row.get("frame_index") == frame_index, f"QA {label} frame index mismatch")
        path = _qa_path(source_video_receipt, row.get("path"))
        _require(row.get("sha256") == sha256(path) and row.get("size") == path.stat().st_size, f"QA {label} hash or size mismatch")
        _require(path.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n", f"QA {label} is not PNG")
        validated_qa.append({
            "label": label,
            "frame_index": frame_index,
            "size": path.stat().st_size,
            "sha256": sha256(path),
        })

    attestation = load_json(review_attestation)
    _require(attestation.get("schema") == REVIEW_ATTESTATION_SCHEMA, "review attestation schema mismatch")
    _require(attestation.get("status") == "PASSED", "full-body review did not pass")
    _require(attestation.get("qualification") is False, "review attestation was relabelled")
    _require(attestation.get("real_robot_commands") is False, "review attestation is not real_robot_commands=false")
    _require(attestation.get("video_sha256") == video_hash, "review attestation binds a different video")
    _require(attestation.get("source_video_receipt_sha256") == sha256(source_video_receipt), "review attestation binds a different renderer receipt")
    claimed_qa = attestation.get("qa_frame_sha256")
    _require(
        isinstance(claimed_qa, dict)
        and claimed_qa == {row["label"]: row["sha256"] for row in validated_qa},
        "review attestation binds different QA frames",
    )
    reviewer = attestation.get("reviewer")
    _require(isinstance(reviewer, str) and bool(reviewer.strip()), "reviewer is required")
    reviewed_at = _strict_utc(attestation.get("reviewed_at"))
    checks = attestation.get("checks")
    _require(isinstance(checks, dict), "review checks are missing")
    for name in REQUIRED_REVIEW_CHECKS:
        _require(checks.get(name) is True, f"review check did not pass: {name}")

    return {
        "schema": CANONICAL_MEDIA_RECEIPT_SCHEMA,
        "suite_id": suite_id,
        "status": "PASSED",
        "canonical_episode": {
            "episode_id": attempt["episode_id"],
            "result_sha256": attempt["sha256"],
            "state_command_trace_sha256": state_hash,
        },
        "source_video_receipt_sha256": sha256(source_video_receipt),
        "human_review_attestation_sha256": sha256(review_attestation),
        "authority": {
            "source": PHYSX_AUTHORITY,
            "replay": "none_state_only_renderer",
            "renderer_advances_physics": False,
        },
        "video": {
            "size": video.stat().st_size,
            "sha256": video_hash,
            "codec": "h264",
            "fps": 30.0,
            "width": media["width"],
            "height": media["height"],
            "frame_count": frame_count,
        },
        "overlay": deepcopy(overlay),
        "camera": {
            "mode": "robot_follow_full_body_profile_v1",
            "offset_xyz_m": offset,
            "target_xyz_m": target,
            "left_menu_visible": False,
            "visual_full_body_verification": "PASSED",
        },
        "qa_frames": validated_qa,
        "human_review": {
            "status": "PASSED",
            "method": "human_review_first_middle_last_full_body",
            "reviewer": reviewer,
            "reviewed_at": reviewed_at,
            **{name: True for name in REQUIRED_REVIEW_CHECKS},
        },
        "published_command_topics": [],
        "qualification": False,
        "real_robot_commands": False,
    }


def seal_canonical_media_receipt(
    *,
    suite_id: str,
    episode_result: Path,
    video: Path,
    source_video_receipt: Path,
    review_attestation: Path,
    output: Path,
) -> dict[str, Any]:
    payload = build_canonical_media_receipt(
        suite_id=suite_id,
        episode_result=episode_result,
        video=video,
        source_video_receipt=source_video_receipt,
        review_attestation=review_attestation,
    )
    _atomic_write(output, payload)
    return payload
