#!/usr/bin/env python3
"""Render an immutable X2 PhysX state trace without advancing physics.

The source trace contains the exact local transforms emitted by the
authoritative Genie Sim / Isaac PhysX run.  This process republishes only those
transforms and a free-camera pose to a separate Genie Sim renderer.  It never
publishes velocity, joint, navigation, or robot commands.
"""

from __future__ import annotations

import argparse
import bisect
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any, Mapping, Sequence

import numpy as np


TRACE_SCHEMA = "sceneforge/x2-geniesim-render-state/v1"
TRACE_RECEIPT_SCHEMA = "sceneforge/x2-geniesim-render-state-receipt/v1"
REPLAY_RECEIPT_SCHEMA = "sceneforge/x2-geniesim-render-state-replay/v1"
PHYSICS_SOURCE = "geniesim_isaac_physx"
OVERLAY_SCHEMA = "sceneforge/x2-combination-video-overlay/v1"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, delete=False) as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        temporary = Path(stream.name)
    temporary.chmod(0o644)
    temporary.replace(path)


def finite_vector(value: Any, length: int, name: str) -> tuple[float, ...]:
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must contain {length} numbers") from error
    if len(result) != length or not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain {length} finite numbers")
    return result


def normalise_xyzw(value: Any, name: str) -> tuple[float, float, float, float]:
    x, y, z, w = finite_vector(value, 4, name)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError(f"{name} has zero norm")
    return x / norm, y / norm, z / norm, w / norm


def slerp_xyzw(left: Sequence[float], right: Sequence[float], alpha: float) -> tuple[float, ...]:
    if not 0.0 <= alpha <= 1.0:
        raise ValueError("slerp alpha must be in [0, 1]")
    q0 = np.asarray(normalise_xyzw(left, "left quaternion"), dtype=float)
    q1 = np.asarray(normalise_xyzw(right, "right quaternion"), dtype=float)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1, dot = -q1, -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        result = q0 + alpha * (q1 - q0)
        result /= np.linalg.norm(result)
    else:
        theta = math.acos(dot)
        result = (math.sin((1.0 - alpha) * theta) * q0 + math.sin(alpha * theta) * q1) / math.sin(theta)
    return tuple(float(item) for item in result)


@dataclass(frozen=True)
class LinkTransform:
    parent_path: str
    child_path: str
    translation_xyz_m: tuple[float, float, float]
    rotation_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class ReplaySample:
    stamp_ns: int
    base_position_xyz_m: tuple[float, float, float]
    base_orientation_xyzw: tuple[float, float, float, float]
    transforms: tuple[LinkTransform, ...]
    overlay: Mapping[str, Any] | None = None


@dataclass(frozen=True)
class StateTrace:
    samples: tuple[ReplaySample, ...]
    topology: tuple[str, ...]
    header: Mapping[str, Any]
    summary: Mapping[str, Any]
    source_sha256: str


def _load_transform(record: Mapping[str, Any], line_number: int) -> LinkTransform:
    child = str(record.get("child_path", ""))
    parent = str(record.get("parent_path", ""))
    if not child.startswith("/x2/") or not parent:
        raise ValueError(f"line {line_number}: transform paths are not authoritative X2 USD paths")
    return LinkTransform(
        parent_path=parent,
        child_path=child,
        translation_xyz_m=finite_vector(record.get("translation_xyz_m"), 3, "translation_xyz_m"),
        rotation_xyzw=normalise_xyzw(record.get("rotation_xyzw"), "rotation_xyzw"),
    )


def _load_overlay(value: Any, line_number: int) -> Mapping[str, Any] | None:
    if value is None:
        return None
    if not isinstance(value, dict) or value.get("schema") != OVERLAY_SCHEMA:
        raise ValueError(f"line {line_number}: overlay schema mismatch")
    if value.get("real_robot_commands") is not False:
        raise ValueError(f"line {line_number}: overlay real_robot_commands is not false")
    for name in ("scenario", "phase", "active_gait", "suspension_state", "rtf_method"):
        if not isinstance(value.get(name), str) or not value[name]:
            raise ValueError(f"line {line_number}: overlay {name} is missing")
    for name in (
        "sim_time_s", "cpg_phase_rad", "cpg_period_s", "linear_speed_mps",
        "angular_speed_rps", "suspension_support_ratio", "suspension_force_norm_n",
        "rtf_estimate",
    ):
        number = value.get(name)
        if not isinstance(number, (int, float)) or isinstance(number, bool) or not math.isfinite(float(number)):
            raise ValueError(f"line {line_number}: overlay {name} is not finite")
    for name in (
        "requested_velocity", "effective_velocity_command",
        "measured_body_linear_velocity", "measured_body_angular_velocity",
    ):
        finite_vector(value.get(name), 3, f"overlay {name}")
    for name in ("left_foot_contact", "right_foot_contact"):
        if not isinstance(value.get(name), bool):
            raise ValueError(f"line {line_number}: overlay {name} is not boolean")
    return dict(value)


def load_trace(
    trace_path: Path,
    receipt_path: Path,
    *,
    require_overlay: bool = False,
) -> StateTrace:
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    source_hash = sha256_file(trace_path)
    required_receipt = {
        "schema": TRACE_RECEIPT_SCHEMA,
        "status": "SUCCEEDED",
        "authoritative_physics_source": PHYSICS_SOURCE,
        "read_only": True,
        "real_robot_commands": False,
    }
    for key, expected in required_receipt.items():
        if receipt.get(key) != expected:
            raise ValueError(f"trace receipt {key} must be {expected!r}")
    if receipt.get("published_topics") != []:
        raise ValueError("trace recorder was not read-only")
    if receipt.get("output_sha256") != source_hash:
        raise ValueError("trace SHA-256 differs from its receipt")
    if require_overlay and (
        receipt.get("overlay_schema") != OVERLAY_SCHEMA
        or receipt.get("overlay_sample_count") != receipt.get("sample_count")
    ):
        raise ValueError("trace receipt does not seal a complete overlay")

    records: list[Mapping[str, Any]] = []
    with trace_path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"line {line_number}: invalid JSON") from error
            if not isinstance(record, dict):
                raise ValueError(f"line {line_number}: record must be an object")
            records.append(record)
    if len(records) < 3:
        raise ValueError("trace must contain header, samples, and summary")
    header, summary = records[0], records[-1]
    if header.get("record_type") != "schema" or header.get("schema") != TRACE_SCHEMA:
        raise ValueError("trace header schema mismatch")
    if header.get("authoritative_physics_source") != PHYSICS_SOURCE:
        raise ValueError("trace does not originate from Genie Sim PhysX")
    if header.get("read_only") is not True or header.get("published_topics") != []:
        raise ValueError("trace header does not prove read-only capture")
    if header.get("real_robot_commands") is not False:
        raise ValueError("trace header real_robot_commands is not false")
    if require_overlay and header.get("overlay_schema") != OVERLAY_SCHEMA:
        raise ValueError("trace header does not declare the required overlay schema")
    if summary.get("record_type") != "summary" or summary.get("schema") != TRACE_SCHEMA:
        raise ValueError("trace summary schema mismatch")
    if summary.get("status") != "SUCCEEDED" or summary.get("real_robot_commands") is not False:
        raise ValueError("trace summary is not a safe successful capture")

    samples: list[ReplaySample] = []
    topology: tuple[str, ...] | None = None
    for expected_sequence, record in enumerate(records[1:-1]):
        line_number = expected_sequence + 2
        if record.get("record_type") != "sample" or record.get("schema") != TRACE_SCHEMA:
            raise ValueError(f"line {line_number}: unexpected record type")
        if record.get("sequence") != expected_sequence:
            raise ValueError(f"line {line_number}: non-contiguous sequence")
        stamp_ns = int(record.get("sim_time_ns"))
        if stamp_ns < 0 or (samples and stamp_ns <= samples[-1].stamp_ns):
            raise ValueError(f"line {line_number}: timestamps are not strictly increasing")
        transforms = tuple(_load_transform(item, line_number) for item in record.get("transforms", []))
        paths = tuple(item.child_path for item in transforms)
        if not paths or len(paths) != len(set(paths)):
            raise ValueError(f"line {line_number}: transform topology is empty or duplicated")
        if topology is None:
            topology = paths
        elif paths != topology:
            raise ValueError(f"line {line_number}: transform topology changed")
        overlay = _load_overlay(record.get("overlay"), line_number)
        if require_overlay and overlay is None:
            raise ValueError(f"line {line_number}: required overlay is missing")
        samples.append(ReplaySample(
            stamp_ns=stamp_ns,
            base_position_xyz_m=finite_vector(record.get("base_position_xyz_m"), 3, "base_position_xyz_m"),
            base_orientation_xyzw=normalise_xyzw(record.get("base_orientation_xyzw"), "base_orientation_xyzw"),
            transforms=transforms,
            overlay=overlay,
        ))
    if summary.get("sample_count") != len(samples) or receipt.get("sample_count") != len(samples):
        raise ValueError("trace sample count differs from summary or receipt")
    return StateTrace(tuple(samples), topology or (), header, summary, source_hash)


def interpolate_sample(left: ReplaySample, right: ReplaySample, stamp_ns: int) -> ReplaySample:
    if left.stamp_ns == right.stamp_ns:
        return left
    alpha = (stamp_ns - left.stamp_ns) / (right.stamp_ns - left.stamp_ns)
    if not 0.0 <= alpha <= 1.0:
        raise ValueError("interpolation stamp is outside bracket")
    translation = tuple(
        tuple(a + alpha * (b - a) for a, b in zip(first.translation_xyz_m, second.translation_xyz_m))
        for first, second in zip(left.transforms, right.transforms)
    )
    transforms = tuple(
        LinkTransform(
            parent_path=first.parent_path,
            child_path=first.child_path,
            translation_xyz_m=translation[index],
            rotation_xyzw=slerp_xyzw(first.rotation_xyzw, second.rotation_xyzw, alpha),
        )
        for index, (first, second) in enumerate(zip(left.transforms, right.transforms))
    )
    source_overlay = left.overlay if alpha < 0.5 else right.overlay
    overlay = dict(source_overlay) if source_overlay is not None else None
    if overlay is not None:
        overlay["sim_time_s"] = stamp_ns / 1e9
    return ReplaySample(
        stamp_ns=stamp_ns,
        base_position_xyz_m=tuple(a + alpha * (b - a) for a, b in zip(left.base_position_xyz_m, right.base_position_xyz_m)),
        base_orientation_xyzw=slerp_xyzw(left.base_orientation_xyzw, right.base_orientation_xyzw, alpha),
        transforms=transforms,
        overlay=overlay,
    )


def resample_trace(
    samples: Sequence[ReplaySample], fps: float, start_s: float | None, end_s: float | None
) -> tuple[ReplaySample, ...]:
    if fps <= 0.0 or len(samples) < 2:
        raise ValueError("resampling requires positive fps and at least two samples")
    source_start = samples[0].stamp_ns / 1e9
    source_end = samples[-1].stamp_ns / 1e9
    start = source_start if start_s is None else max(source_start, start_s)
    end = source_end if end_s is None else min(source_end, end_s)
    if end <= start:
        raise ValueError("selected replay window is empty")
    frame_count = int(math.floor((end - start) * fps)) + 1
    source_stamps = [sample.stamp_ns for sample in samples]
    result: list[ReplaySample] = []
    for index in range(frame_count):
        stamp_ns = int(round((start + index / fps) * 1e9))
        right_index = bisect.bisect_left(source_stamps, stamp_ns)
        if right_index <= 0:
            result.append(samples[0])
        elif right_index >= len(samples):
            result.append(samples[-1])
        elif source_stamps[right_index] == stamp_ns:
            result.append(samples[right_index])
        else:
            result.append(interpolate_sample(samples[right_index - 1], samples[right_index], stamp_ns))
    return tuple(result)


def image_to_rgb(message: Any) -> bytes:
    encoding = str(message.encoding).lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    width, height, step = int(message.width), int(message.height), int(message.step)
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if width <= 0 or height <= 0 or step < width * channels or raw.size < height * step:
        raise ValueError("invalid image dimensions, step, or data length")
    pixels = raw[: height * step].reshape(height, step)[:, : width * channels].reshape(height, width, channels)
    if encoding in ("bgr8", "bgra8"):
        pixels = pixels[:, :, [2, 1, 0] + ([3] if channels == 4 else [])]
    return np.ascontiguousarray(pixels[:, :, :3]).tobytes()


def overlay_lines(overlay: Mapping[str, Any]) -> tuple[str, ...]:
    """Format an ASCII-only qualification HUD with no font asset dependency."""

    requested = finite_vector(overlay["requested_velocity"], 3, "requested_velocity")
    effective = finite_vector(
        overlay["effective_velocity_command"], 3, "effective_velocity_command"
    )
    measured_linear = finite_vector(
        overlay["measured_body_linear_velocity"], 3,
        "measured_body_linear_velocity",
    )
    measured_angular = finite_vector(
        overlay["measured_body_angular_velocity"], 3,
        "measured_body_angular_velocity",
    )
    left = "CONTACT" if overlay["left_foot_contact"] else "SWING"
    right = "CONTACT" if overlay["right_foot_contact"] else "SWING"
    return (
        f"CASE {overlay['scenario'].upper()} | PHASE {overlay['phase']} | GAIT {overlay['active_gait']}",
        f"REQ  vx {requested[0]:+.2f} vy {requested[1]:+.2f} wz {requested[2]:+.2f}",
        f"CTRL vx {effective[0]:+.2f} vy {effective[1]:+.2f} wz {effective[2]:+.2f}",
        f"ACT  vx {measured_linear[0]:+.2f} vy {measured_linear[1]:+.2f} wz {measured_angular[2]:+.2f}",
        f"FEET L {left} | R {right} | CPG {float(overlay['cpg_phase_rad']):+.2f} rad",
        f"BAND {overlay['suspension_state']} ratio {float(overlay['suspension_support_ratio']):.2f} force {float(overlay['suspension_force_norm_n']):.1f} N",
        f"SIM {float(overlay['sim_time_s']):.2f} s | RTF~{float(overlay['rtf_estimate']):.2f}x ({overlay['rtf_method']})",
    )


def render_overlay_rgb(
    rgb: bytes,
    width: int,
    height: int,
    overlay: Mapping[str, Any],
) -> bytes:
    """Burn the qualification HUD into the top-right of one RGB frame."""

    # Validate every value again at the presentation boundary.  Missing HUD
    # evidence must fail the replay rather than silently produce an unlabeled
    # qualification video.
    _load_overlay(overlay, 0)
    if width < 640 or height < 360:
        raise ValueError("overlay video must be at least 640x360")
    frame = np.frombuffer(rgb, dtype=np.uint8).reshape(height, width, 3).copy()
    lines = overlay_lines(overlay)
    try:
        import cv2

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.44, min(0.62, width / 2200.0))
        thickness = 1 if width < 1500 else 2
        line_height = max(22, int(31 * scale / 0.55))
        padding = 14
        text_width = max(
            cv2.getTextSize(line, font, scale, thickness)[0][0] for line in lines
        )
        panel_width = min(width - 24, text_width + 2 * padding)
        panel_height = line_height * len(lines) + 2 * padding
        x0, y0 = width - panel_width - 12, 12
        x1, y1 = width - 12, min(height - 12, y0 + panel_height)
        panel = frame[y0:y1, x0:x1]
        black = np.zeros_like(panel)
        cv2.addWeighted(black, 0.68, panel, 0.32, 0.0, panel)
        for index, line in enumerate(lines):
            colour = (115, 255, 230) if index == 0 else (240, 240, 240)
            cv2.putText(
                frame,
                line,
                (x0 + padding, y0 + padding + (index + 1) * line_height - 6),
                font,
                scale,
                colour,
                thickness,
                cv2.LINE_AA,
            )
        return np.ascontiguousarray(frame).tobytes()
    except ImportError:
        try:
            from PIL import Image, ImageDraw, ImageFont
        except ImportError as exc:
            raise RuntimeError(
                "OpenCV or Pillow is required for fail-closed HUD rendering"
            ) from exc
        image = Image.fromarray(frame, mode="RGB")
        draw = ImageDraw.Draw(image, mode="RGBA")
        try:
            font = ImageFont.truetype("DejaVuSansMono.ttf", 16)
        except OSError:
            font = ImageFont.load_default()
        boxes = [draw.textbbox((0, 0), line, font=font) for line in lines]
        line_height = max(box[3] - box[1] for box in boxes) + 7
        padding = 12
        panel_width = min(width - 24, max(box[2] - box[0] for box in boxes) + 2 * padding)
        panel_height = line_height * len(lines) + 2 * padding
        x0, y0 = width - panel_width - 12, 12
        draw.rectangle(
            (x0, y0, width - 12, min(height - 12, y0 + panel_height)),
            fill=(0, 0, 0, 180),
        )
        for index, line in enumerate(lines):
            colour = (115, 255, 230, 255) if index == 0 else (240, 240, 240, 255)
            draw.text(
                (x0 + padding, y0 + padding + index * line_height),
                line,
                font=font,
                fill=colour,
            )
        return np.ascontiguousarray(np.asarray(image)).tobytes()


def look_at_quaternion_xyzw(eye: np.ndarray, target: np.ndarray) -> tuple[float, ...]:
    forward = target - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.asarray((0.0, 0.0, 1.0)))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    rotation = np.column_stack((right, up, -forward))
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * scale, (rotation[2, 1] - rotation[1, 2]) / scale, (rotation[0, 2] - rotation[2, 0]) / scale, (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w, x, y, z = (rotation[2, 1] - rotation[1, 2]) / scale, 0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale, (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w, x, y, z = (rotation[0, 2] - rotation[2, 0]) / scale, (rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale, (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w, x, y, z = (rotation[1, 0] - rotation[0, 1]) / scale, (rotation[0, 2] + rotation[2, 0]) / scale, (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale
    return normalise_xyzw((x, y, z, w), "camera quaternion")


def camera_pose(sample: ReplaySample, offset: Sequence[float], target_offset: Sequence[float]) -> tuple[np.ndarray, tuple[float, ...]]:
    x, y, z, w = sample.base_orientation_xyzw
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    eye = np.asarray((
        sample.base_position_xyz_m[0] + cosine * offset[0] - sine * offset[1],
        sample.base_position_xyz_m[1] + sine * offset[0] + cosine * offset[1],
        offset[2],
    ))
    target = np.asarray((
        sample.base_position_xyz_m[0] + cosine * target_offset[0] - sine * target_offset[1],
        sample.base_position_xyz_m[1] + sine * target_offset[0] + cosine * target_offset[1],
        target_offset[2],
    ))
    return eye, look_at_quaternion_xyzw(eye, target)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--trace-receipt", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--minimum-frames", type=int, default=60)
    parser.add_argument("--source-start-s", type=float)
    parser.add_argument("--source-end-s", type=float)
    parser.add_argument("--tf-topic", default="/tf_render")
    parser.add_argument("--camera-topic", default="/genie_sim_render/free_cam_pose")
    parser.add_argument("--image-topic", default="/genie_sim/free_camera_rgb/image_raw")
    parser.add_argument("--discovery-timeout-s", type=float, default=60.0)
    parser.add_argument("--warmup-timeout-s", type=float, default=120.0)
    parser.add_argument("--frame-timeout-s", type=float, default=10.0)
    parser.add_argument("--wall-timeout-s", type=float, default=1200.0)
    parser.add_argument("--publish-retry-s", type=float, default=0.05)
    parser.add_argument("--warmup-frames", type=int, default=15)
    parser.add_argument(
        "--require-overlay",
        action="store_true",
        help="require and burn the X2 combination qualification HUD into every frame",
    )
    parser.add_argument("--offset-x", type=float, default=-1.6)
    parser.add_argument("--offset-y", type=float, default=-1.6)
    parser.add_argument("--offset-z", type=float, default=1.35)
    parser.add_argument("--target-x", type=float, default=0.2)
    parser.add_argument("--target-y", type=float, default=0.0)
    parser.add_argument("--target-z", type=float, default=0.65)
    return parser.parse_args(argv)


def base_receipt(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "schema": REPLAY_RECEIPT_SCHEMA,
        "status": "STARTING",
        "artifact_kind": "offline_authoritative_physx_link_state_replay",
        "source_authoritative_physics_source": PHYSICS_SOURCE,
        "replay_physics_authority": "none_state_only_renderer",
        "renderer_advances_physics": False,
        "offline_replay": True,
        "mujoco_state_source_used": False,
        "real_robot_commands": False,
        "published_robot_command_topics": [],
        "published_state_only_topics": [args.tf_topic, args.camera_topic],
        "subscribed_topics": [args.image_topic],
        "transform_semantics": "recorded_local_transform_applied_to_exact_absolute_nested_x2_usd_prim_path",
        "output_fps": args.fps,
        "overlay_required": bool(args.require_overlay),
        "overlay_schema": OVERLAY_SCHEMA if args.require_overlay else None,
        "overlay_position": "top_right" if args.require_overlay else None,
        "overlay_frame_count": 0,
        "frame_count": 0,
        "missing_samples": [],
        "errors": [],
        "limitations": [
            "Offline state-only rendering; this process does not advance PhysX.",
            "The video is evidence presentation for the referenced immutable run, not a new control or stability run.",
            "Frames are interpolated to 30 fps from the read-only PhysX transform trace.",
        ],
        "started_at": datetime.now(timezone.utc).isoformat(),
    }


def run_ros_replay(args: argparse.Namespace, samples: Sequence[ReplaySample], partial: Path, receipt: dict[str, Any]) -> None:
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from tf2_msgs.msg import TFMessage

    class ReplayNode(Node):
        def __init__(self) -> None:
            super().__init__("sceneforge_x2_offline_render_state_replay")
            camera_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            image_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
            self.tf_publisher = self.create_publisher(TFMessage, args.tf_topic, qos_profile_sensor_data)
            self.camera_publisher = self.create_publisher(PoseStamped, args.camera_topic, camera_qos)
            self.image_subscription = self.create_subscription(Image, args.image_topic, self.on_image, image_qos)
            self.target_sequence = 0
            self.image_count = 0
            self.rgb: bytes | None = None
            self.width: int | None = None
            self.height: int | None = None

        def on_image(self, message: Image) -> None:
            self.image_count += 1
            if self.image_count <= self.target_sequence or self.rgb is not None:
                return
            self.rgb = image_to_rgb(message)
            self.width, self.height = int(message.width), int(message.height)

        @staticmethod
        def stamp(stamp_ns: int) -> Time:
            result = Time()
            result.sec, result.nanosec = divmod(stamp_ns, 1_000_000_000)
            return result

        def publish(self, sample: ReplaySample) -> None:
            stamp = self.stamp(sample.stamp_ns)
            message = TFMessage()
            for transform in sample.transforms:
                item = TransformStamped()
                item.header.stamp = stamp
                item.header.frame_id = transform.parent_path
                item.child_frame_id = transform.child_path
                item.transform.translation.x, item.transform.translation.y, item.transform.translation.z = transform.translation_xyz_m
                item.transform.rotation.x, item.transform.rotation.y, item.transform.rotation.z, item.transform.rotation.w = transform.rotation_xyzw
                message.transforms.append(item)
            self.tf_publisher.publish(message)
            eye, orientation = camera_pose(
                sample,
                (args.offset_x, args.offset_y, args.offset_z),
                (args.target_x, args.target_y, args.target_z),
            )
            camera = PoseStamped()
            camera.header.stamp = stamp
            camera.header.frame_id = "world"
            camera.pose.position.x, camera.pose.position.y, camera.pose.position.z = map(float, eye)
            camera.pose.orientation.x, camera.pose.orientation.y, camera.pose.orientation.z, camera.pose.orientation.w = orientation
            self.camera_publisher.publish(camera)

    rclpy.init()
    node = ReplayNode()
    ffmpeg: subprocess.Popen[bytes] | None = None
    cv_writer: Any | None = None
    encoded_size: tuple[int, int] | None = None
    primary_error: BaseException | None = None
    try:
        deadline = time.monotonic() + args.discovery_timeout_s
        while time.monotonic() < deadline:
            if node.tf_publisher.get_subscription_count() and node.camera_publisher.get_subscription_count() and node.count_publishers(args.image_topic):
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        else:
            raise RuntimeError("renderer topic discovery timed out")
        receipt["status"] = "REPLAYING"
        receipt["topic_discovery"] = {
            "tf_subscribers": node.tf_publisher.get_subscription_count(),
            "camera_subscribers": node.camera_publisher.get_subscription_count(),
            "image_publishers": node.count_publishers(args.image_topic),
        }
        atomic_json(args.receipt, receipt)
        replay_deadline = time.monotonic() + args.wall_timeout_s
        # Prime both the FreeCam and all nested X2 link transforms, then discard
        # the renderer's shader/camera settling frames.  Otherwise the first
        # encoded frame can be the neutral grey buffer observed during OVRTX
        # warmup even though every later frame is correct.
        warmup_start = node.image_count
        warmup_deadline = min(
            replay_deadline,
            time.monotonic() + args.warmup_timeout_s,
        )
        next_publish = 0.0
        while node.image_count - warmup_start < args.warmup_frames and time.monotonic() < warmup_deadline:
            now = time.monotonic()
            if now >= next_publish:
                node.publish(samples[0])
                next_publish = now + args.publish_retry_s
            rclpy.spin_once(node, timeout_sec=0.02)
        receipt["discarded_warmup_frame_count"] = node.image_count - warmup_start
        if receipt["discarded_warmup_frame_count"] < args.warmup_frames:
            raise RuntimeError("renderer did not complete the requested warmup frames")
        for index, sample in enumerate(samples):
            if time.monotonic() >= replay_deadline:
                raise RuntimeError("wall timeout expired during replay")
            node.target_sequence = node.image_count
            node.rgb = None
            frame_deadline = min(replay_deadline, time.monotonic() + args.frame_timeout_s)
            next_publish = 0.0
            while node.rgb is None and time.monotonic() < frame_deadline:
                now = time.monotonic()
                if now >= next_publish:
                    node.publish(sample)
                    next_publish = now + args.publish_retry_s
                rclpy.spin_once(node, timeout_sec=min(0.02, max(0.0, frame_deadline - now)))
            if node.rgb is None:
                receipt["missing_samples"].append({"index": index, "stamp_ns": sample.stamp_ns})
                continue
            size = (int(node.width), int(node.height))
            if ffmpeg is None:
                if cv_writer is None:
                    encoded_size = size
                    receipt["video_width"] = size[0]
                    receipt["video_height"] = size[1]
                    if shutil.which("ffmpeg"):
                        command = [
                            "ffmpeg", "-hide_banner", "-loglevel", "warning", "-y",
                            "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", f"{size[0]}x{size[1]}",
                            "-framerate", str(args.fps), "-i", "-", "-an", "-c:v", "libx264",
                            "-preset", "veryfast", "-crf", "18", "-pix_fmt", "yuv420p", "-r", str(args.fps),
                            "-movflags", "+faststart", str(partial),
                        ]
                        ffmpeg = subprocess.Popen(command, stdin=subprocess.PIPE)
                        receipt["encoder_backend"] = "ffmpeg_libx264"
                    else:
                        import cv2
                        cv_writer = cv2.VideoWriter(
                            str(partial), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, size
                        )
                        if not cv_writer.isOpened():
                            raise RuntimeError("OpenCV mp4v encoder could not be opened")
                        receipt["encoder_backend"] = "opencv_ffmpeg_mp4v"
            if size != encoded_size:
                raise RuntimeError("renderer image dimensions changed")
            frame_rgb = node.rgb
            if args.require_overlay:
                if sample.overlay is None:
                    raise RuntimeError("required per-frame qualification overlay is missing")
                frame_rgb = render_overlay_rgb(
                    frame_rgb, size[0], size[1], sample.overlay
                )
                receipt["overlay_frame_count"] += 1
            if ffmpeg is not None:
                assert ffmpeg.stdin is not None
                ffmpeg.stdin.write(frame_rgb)
            else:
                assert cv_writer is not None
                rgb = np.frombuffer(frame_rgb, dtype=np.uint8).reshape(size[1], size[0], 3)
                cv_writer.write(np.ascontiguousarray(rgb[:, :, ::-1]))
            receipt["frame_count"] += 1
        receipt["image_message_count"] = node.image_count
    except BaseException as error:
        primary_error = error
        raise
    finally:
        ffmpeg_error: Exception | None = None
        if cv_writer is not None:
            cv_writer.release()
        if ffmpeg is not None:
            assert ffmpeg.stdin is not None
            try:
                ffmpeg.stdin.close()
            except (BrokenPipeError, OSError) as error:
                ffmpeg_error = RuntimeError(f"ffmpeg stdin close failed: {error}")
            try:
                return_code = ffmpeg.wait(timeout=60)
            except subprocess.TimeoutExpired:
                ffmpeg.terminate()
                try:
                    return_code = ffmpeg.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    ffmpeg.kill(); return_code = ffmpeg.wait(timeout=5)
                ffmpeg_error = RuntimeError("ffmpeg required terminate/kill cleanup")
            if return_code != 0 and ffmpeg_error is None:
                ffmpeg_error = RuntimeError(f"ffmpeg exited with {return_code}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if ffmpeg_error is not None:
            receipt["errors"].append(str(ffmpeg_error))
            if primary_error is None:
                raise ffmpeg_error
    if receipt["missing_samples"]:
        raise RuntimeError(f"{len(receipt['missing_samples'])} replay frames were not rendered")
    if receipt["frame_count"] != len(samples) or not partial.is_file() or partial.stat().st_size <= 0:
        raise RuntimeError("encoded video is incomplete")


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    trace_path, trace_receipt = args.trace.resolve(), args.trace_receipt.resolve()
    output, receipt_path = args.output.resolve(), args.receipt.resolve()
    partial = output.with_suffix(output.suffix + ".partial.mp4")
    if (
        args.fps <= 0
        or args.minimum_frames < 1
        or args.warmup_frames < 1
        or args.warmup_timeout_s <= 0
        or args.frame_timeout_s <= 0
    ):
        raise SystemExit("invalid replay limits")
    if output.exists() or receipt_path.exists() or partial.exists():
        raise SystemExit("refusing to overwrite replay evidence")
    receipt = base_receipt(args)
    try:
        trace = load_trace(
            trace_path,
            trace_receipt,
            require_overlay=args.require_overlay,
        )
        samples = resample_trace(trace.samples, args.fps, args.source_start_s, args.source_end_s)
        if len(samples) < args.minimum_frames:
            raise ValueError("selected window has fewer than minimum frames")
        if args.require_overlay and any(sample.overlay is None for sample in samples):
            raise ValueError("resampled replay contains an unlabeled frame")
        topology_payload = "\n".join(trace.topology).encode("utf-8")
        receipt.update({
            "source_trace": {"path": str(trace_path), "size": trace_path.stat().st_size, "sha256": trace.source_sha256},
            "source_trace_receipt": {"path": str(trace_receipt), "sha256": sha256_file(trace_receipt)},
            "source_sample_count": len(trace.samples),
            "source_window_s": [samples[0].stamp_ns / 1e9, samples[-1].stamp_ns / 1e9],
            "replay_frame_count": len(samples),
            "link_count": len(trace.topology),
            "topology_sha256": hashlib.sha256(topology_payload).hexdigest(),
        })
        atomic_json(receipt_path, receipt)
        run_ros_replay(args, samples, partial, receipt)
        if args.require_overlay and receipt["overlay_frame_count"] != len(samples):
            raise RuntimeError("not every encoded frame received the qualification HUD")
        partial.replace(output)
        output.chmod(0o644)
        receipt.update({
            "status": "SUCCEEDED",
            "output": str(output),
            "output_bytes": output.stat().st_size,
            "output_sha256": sha256_file(output),
            "finished_at": datetime.now(timezone.utc).isoformat(),
        })
        atomic_json(receipt_path, receipt)
        return 0
    except Exception as error:
        receipt["status"] = "FAILED"
        receipt["errors"].append(f"{type(error).__name__}: {error}")
        receipt["finished_at"] = datetime.now(timezone.utc).isoformat()
        atomic_json(receipt_path, receipt)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
