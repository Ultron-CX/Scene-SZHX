"""Engine-neutral X2 locomotion metric definitions.

Raw contact slip deliberately remains distinct from steady-support slip.  The
latter excludes lightly loaded/speculative contact and the touchdown/liftoff
edges of each support interval, which otherwise dominate the percentile.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence


def percentile(values: Sequence[float], quantile: float) -> float | None:
    finite = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not finite:
        return None
    if not 0.0 <= quantile <= 1.0:
        raise ValueError("quantile must be within [0, 1]")
    index = quantile * (len(finite) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return finite[lower]
    fraction = index - lower
    return finite[lower] * (1.0 - fraction) + finite[upper] * fraction


def summarize_support_slip(
    samples: Sequence[Mapping[str, object]],
    *,
    load_threshold_n: float = 20.0,
    edge_trim_s: float = 0.10,
) -> dict[str, object]:
    """Summarize raw-contact and stable loaded-support tangential slip.

    Each sample must contain ``sim_time``, ``contact``, ``normal_force_n`` and
    ``tangential_slip_speed_mps``.  Contiguous loaded-support intervals are
    split whenever a sample is not in contact, is below the force threshold,
    or time is non-monotonic.  ``edge_trim_s`` is removed from both ends.
    """

    load_threshold_n = float(load_threshold_n)
    edge_trim_s = float(edge_trim_s)
    if not math.isfinite(load_threshold_n) or load_threshold_n < 0.0:
        raise ValueError("load threshold must be finite and non-negative")
    if not math.isfinite(edge_trim_s) or edge_trim_s < 0.0:
        raise ValueError("edge trim must be finite and non-negative")

    normalized: list[tuple[float, bool, float, float]] = []
    previous_time: float | None = None
    for sample in samples:
        now = float(sample["sim_time"])
        force = abs(float(sample["normal_force_n"]))
        slip = float(sample["tangential_slip_speed_mps"])
        if not all(math.isfinite(value) for value in (now, force, slip)):
            raise ValueError("support-slip sample contains NaN or Inf")
        if previous_time is not None and now <= previous_time:
            raise ValueError("support-slip sample time must strictly increase")
        normalized.append((now, bool(sample["contact"]), force, slip))
        previous_time = now

    raw = [slip for _, contact, _, slip in normalized if contact]
    segments: list[list[tuple[float, float]]] = []
    current: list[tuple[float, float]] = []
    for now, contact, force, slip in normalized:
        if contact and force >= load_threshold_n:
            current.append((now, slip))
        elif current:
            segments.append(current)
            current = []
    if current:
        segments.append(current)

    loaded = [slip for segment in segments for _, slip in segment]
    steady: list[float] = []
    for segment in segments:
        start = segment[0][0] + edge_trim_s
        end = segment[-1][0] - edge_trim_s
        if end < start:
            continue
        steady.extend(slip for now, slip in segment if start <= now <= end)

    return {
        "load_threshold_n": load_threshold_n,
        "edge_trim_s": edge_trim_s,
        "raw_contact_sample_count": len(raw),
        "loaded_support_sample_count": len(loaded),
        "steady_support_sample_count": len(steady),
        "loaded_support_segment_count": len(segments),
        "raw_contact_slip_p50_mps": percentile(raw, 0.50),
        "raw_contact_slip_p95_mps": percentile(raw, 0.95),
        "steady_support_slip_p50_mps": percentile(steady, 0.50),
        "steady_support_slip_p95_mps": percentile(steady, 0.95),
    }


def pose_derived_forward_speed(
    samples: Sequence[Mapping[str, object]],
    *,
    phase: str = "GAIT1_FORWARD",
    trim_start_s: float = 2.0,
    trim_end_s: float = 1.0,
) -> dict[str, object]:
    """Estimate steady body-forward speed from authoritative pose and sim time.

    This common definition avoids engine-specific twist APIs.  Each selected
    world-frame displacement is projected onto the midpoint base heading and
    divided by the covered simulation-time interval.
    """

    rows = [row for row in samples if row["phase"] == phase]
    if len(rows) < 2:
        raise ValueError("pose-derived speed requires at least two phase samples")
    start_limit = float(rows[0]["sim_time"]) + float(trim_start_s)
    end_limit = float(rows[-1]["sim_time"]) - float(trim_end_s)
    selected = [
        row for row in rows
        if start_limit <= float(row["sim_time"]) <= end_limit
    ]
    if len(selected) < 2:
        raise ValueError("pose-derived steady window has insufficient samples")
    forward_distance = 0.0
    for previous, current in zip(selected, selected[1:]):
        previous_position = tuple(float(value) for value in previous["position_m"])
        current_position = tuple(float(value) for value in current["position_m"])
        previous_yaw = float(previous["yaw_rad"])
        current_yaw = float(current["yaw_rad"])
        values = (*previous_position, *current_position, previous_yaw, current_yaw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("pose-derived speed sample contains NaN or Inf")
        yaw_delta = math.atan2(
            math.sin(current_yaw - previous_yaw),
            math.cos(current_yaw - previous_yaw),
        )
        midpoint_yaw = previous_yaw + 0.5 * yaw_delta
        dx = current_position[0] - previous_position[0]
        dy = current_position[1] - previous_position[1]
        forward_distance += dx * math.cos(midpoint_yaw) + dy * math.sin(midpoint_yaw)
    duration = float(selected[-1]["sim_time"]) - float(selected[0]["sim_time"])
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("pose-derived speed duration must be positive")
    return {
        "method": "authoritative_pose_delta_projected_to_midpoint_heading",
        "phase": phase,
        "trim_start_s": float(trim_start_s),
        "trim_end_s": float(trim_end_s),
        "sample_count": len(selected),
        "duration_s": duration,
        "forward_distance_m": forward_distance,
        "mean_forward_speed_mps": forward_distance / duration,
    }


def pose_derived_local_displacement(
    samples: Sequence[Mapping[str, object]],
    *,
    phase: str = "GAIT1_FORWARD",
    trim_start_s: float = 0.0,
    trim_end_s: float = 0.0,
) -> dict[str, object]:
    """Measure locomotion in the frame fixed at walk entry.

    World-X/Y displacement is not a valid forward/lateral metric when the
    robot starts with a non-zero yaw or executes a turn.  This definition
    reports both the net displacement in the walk-entry frame and the
    accumulated body-forward/path distance used by curved trajectories.
    """

    rows = [row for row in samples if row["phase"] == phase]
    if len(rows) < 2:
        raise ValueError("pose-derived displacement requires at least two phase samples")
    start_limit = float(rows[0]["sim_time"]) + float(trim_start_s)
    end_limit = float(rows[-1]["sim_time"]) - float(trim_end_s)
    selected = [
        row for row in rows
        if start_limit <= float(row["sim_time"]) <= end_limit
    ]
    if len(selected) < 2:
        raise ValueError("pose-derived displacement window has insufficient samples")

    start_position = tuple(float(value) for value in selected[0]["position_m"])
    end_position = tuple(float(value) for value in selected[-1]["position_m"])
    entry_yaw = float(selected[0]["yaw_rad"])
    finite = (*start_position, *end_position, entry_yaw)
    if not all(math.isfinite(value) for value in finite):
        raise ValueError("pose-derived displacement sample contains NaN or Inf")

    dx = end_position[0] - start_position[0]
    dy = end_position[1] - start_position[1]
    local_forward = dx * math.cos(entry_yaw) + dy * math.sin(entry_yaw)
    local_cross_track = -dx * math.sin(entry_yaw) + dy * math.cos(entry_yaw)
    body_forward_distance = 0.0
    planar_path_length = 0.0
    for previous, current in zip(selected, selected[1:]):
        previous_position = tuple(float(value) for value in previous["position_m"])
        current_position = tuple(float(value) for value in current["position_m"])
        previous_yaw = float(previous["yaw_rad"])
        current_yaw = float(current["yaw_rad"])
        values = (*previous_position, *current_position, previous_yaw, current_yaw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("pose-derived displacement sample contains NaN or Inf")
        step_dx = current_position[0] - previous_position[0]
        step_dy = current_position[1] - previous_position[1]
        yaw_delta = math.atan2(
            math.sin(current_yaw - previous_yaw),
            math.cos(current_yaw - previous_yaw),
        )
        midpoint_yaw = previous_yaw + 0.5 * yaw_delta
        body_forward_distance += (
            step_dx * math.cos(midpoint_yaw) + step_dy * math.sin(midpoint_yaw)
        )
        planar_path_length += math.hypot(step_dx, step_dy)

    duration = float(selected[-1]["sim_time"]) - float(selected[0]["sim_time"])
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("pose-derived displacement duration must be positive")
    return {
        "method": "walk_entry_frame_net_and_midpoint_heading_path_projection",
        "phase": phase,
        "trim_start_s": float(trim_start_s),
        "trim_end_s": float(trim_end_s),
        "sample_count": len(selected),
        "duration_s": duration,
        "entry_yaw_rad": entry_yaw,
        "local_forward_displacement_m": local_forward,
        "local_cross_track_m": local_cross_track,
        "body_forward_distance_m": body_forward_distance,
        "planar_path_length_m": planar_path_length,
        "local_cross_track_per_forward_m": (
            abs(local_cross_track) / max(abs(local_forward), 1e-9)
        ),
    }


def pose_derived_turn_rate(
    samples: Sequence[Mapping[str, object]],
    *,
    phase: str = "GAIT1_FORWARD",
    trim_start_s: float = 2.0,
    trim_end_s: float = 1.0,
) -> dict[str, object]:
    """Estimate unwrapped yaw change/rate from authoritative pose and sim time."""

    rows = [row for row in samples if row["phase"] == phase]
    if len(rows) < 2:
        raise ValueError("pose-derived turn rate requires at least two phase samples")
    start_limit = float(rows[0]["sim_time"]) + float(trim_start_s)
    end_limit = float(rows[-1]["sim_time"]) - float(trim_end_s)
    selected = [
        row for row in rows
        if start_limit <= float(row["sim_time"]) <= end_limit
    ]
    if len(selected) < 2:
        raise ValueError("pose-derived steady turn window has insufficient samples")
    yaw_change = 0.0
    for previous, current in zip(selected, selected[1:]):
        previous_yaw = float(previous["yaw_rad"])
        current_yaw = float(current["yaw_rad"])
        if not all(math.isfinite(value) for value in (previous_yaw, current_yaw)):
            raise ValueError("pose-derived turn sample contains NaN or Inf")
        yaw_change += math.atan2(
            math.sin(current_yaw - previous_yaw),
            math.cos(current_yaw - previous_yaw),
        )
    duration = float(selected[-1]["sim_time"]) - float(selected[0]["sim_time"])
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("pose-derived turn duration must be positive")
    return {
        "method": "authoritative_pose_unwrapped_yaw_delta",
        "phase": phase,
        "trim_start_s": float(trim_start_s),
        "trim_end_s": float(trim_end_s),
        "sample_count": len(selected),
        "duration_s": duration,
        "yaw_change_rad": yaw_change,
        "mean_yaw_rate_rps": yaw_change / duration,
    }


def summarize_stable_stop(
    samples: Sequence[Mapping[str, object]],
    *,
    phase: str = "STOP_STAND",
    window_s: float = 2.0,
) -> dict[str, object]:
    """Check a sustained final stop window instead of one terminal sample."""

    rows = [row for row in samples if row["phase"] == phase]
    if len(rows) < 2:
        raise ValueError("stable-stop summary requires at least two phase samples")
    end_time = float(rows[-1]["sim_time"])
    selected = [row for row in rows if float(row["sim_time"]) >= end_time - window_s]
    if len(selected) < 2:
        raise ValueError("stable-stop window has insufficient samples")
    linear = [float(row["linear_speed_mps"]) for row in selected]
    angular = [float(row["angular_speed_rps"]) for row in selected]
    positions = [tuple(float(value) for value in row["position_m"]) for row in selected]
    finite = [*linear, *angular, *(value for position in positions for value in position)]
    if not all(math.isfinite(value) for value in finite):
        raise ValueError("stable-stop sample contains NaN or Inf")
    drift = math.dist(positions[0][:2], positions[-1][:2])
    return {
        "phase": phase,
        "window_s": float(window_s),
        "sample_count": len(selected),
        "observed_duration_s": float(selected[-1]["sim_time"]) - float(selected[0]["sim_time"]),
        "linear_speed_p95_mps": percentile(linear, 0.95),
        "angular_speed_p95_rps": percentile(angular, 0.95),
        "maximum_linear_speed_mps": max(linear),
        "maximum_angular_speed_rps": max(angular),
        "planar_drift_m": drift,
    }
