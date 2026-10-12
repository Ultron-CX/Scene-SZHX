"""Engine-neutral timing and metric helpers for X2 MuJoCo motion baselines."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence
import xml.etree.ElementTree as ET


# URDF velocity is a nominal actuator limit, not a hard state constraint in
# either MuJoCo or PhysX.  Contact impulses and discrete sampling are retained
# as strict exceedance evidence; only a short <=5% numerical band is allowed
# for development A/B, and sustained exceedance is rejected separately.
X2_JOINT_VELOCITY_NUMERIC_TOLERANCE_RATIO = 1.05


@dataclass(frozen=True)
class X2MujocoMotionProfile:
    supported_stand_s: float = 3.0
    band_ramp_s: float = 5.0
    unsupported_stand_s: float = 3.0
    walk_s: float = 8.0
    deceleration_s: float = 2.0
    cpg_stand_handoff_s: float = 1.2
    stop_s: float = 8.0
    command_x_mps: float = 0.3
    command_y_mps: float = 0.0
    command_yaw_rps: float = 0.0

    def validate(self) -> None:
        durations = (
            self.supported_stand_s,
            self.band_ramp_s,
            self.unsupported_stand_s,
            self.walk_s,
            self.deceleration_s,
            self.cpg_stand_handoff_s,
            self.stop_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in durations):
            raise ValueError("motion-profile durations must be finite and positive")
        command = (self.command_x_mps, self.command_y_mps, self.command_yaw_rps)
        if not all(math.isfinite(value) for value in command):
            raise ValueError("motion command contains NaN or Inf")
        if math.sqrt(sum(value * value for value in command)) <= 0.10:
            raise ValueError("motion command must cross the CPG active threshold")

    @property
    def release_time_s(self) -> float:
        return self.supported_stand_s + self.band_ramp_s

    @property
    def walk_start_s(self) -> float:
        return self.release_time_s + self.unsupported_stand_s

    @property
    def stop_start_s(self) -> float:
        return self.cpg_stand_handoff_start_s + self.cpg_stand_handoff_s

    @property
    def deceleration_start_s(self) -> float:
        return self.walk_start_s + self.walk_s

    @property
    def cpg_stand_handoff_start_s(self) -> float:
        return self.deceleration_start_s + self.deceleration_s

    @property
    def total_duration_s(self) -> float:
        return self.stop_start_s + self.stop_s

    def phase(self, sim_time_s: float) -> str:
        value = float(sim_time_s)
        if value < 0.0 or not math.isfinite(value):
            raise ValueError("sim time must be finite and non-negative")
        if value < self.supported_stand_s:
            return "SUPPORTED_STAND"
        if value < self.release_time_s:
            return "BAND_RAMP_DOWN"
        if value < self.walk_start_s:
            return "UNSUPPORTED_STAND"
        if value < self.deceleration_start_s:
            return "GAIT1_FORWARD"
        if value < self.cpg_stand_handoff_start_s:
            return "DECELERATE"
        if value < self.stop_start_s:
            return "CPG_STAND_HANDOFF"
        if value < self.total_duration_s:
            return "STOP_STAND"
        return "COMPLETE"

    def band_scale(self, sim_time_s: float) -> float:
        phase = self.phase(sim_time_s)
        if phase == "SUPPORTED_STAND":
            return 1.0
        if phase == "BAND_RAMP_DOWN":
            elapsed = float(sim_time_s) - self.supported_stand_s
            return max(0.0, min(1.0, 1.0 - elapsed / self.band_ramp_s))
        return 0.0

    def velocity_command(self, sim_time_s: float) -> tuple[float, float, float]:
        phase = self.phase(sim_time_s)
        if phase == "GAIT1_FORWARD":
            return (self.command_x_mps, self.command_y_mps, self.command_yaw_rps)
        if phase == "DECELERATE":
            progress = max(
                0.0,
                min(1.0, (float(sim_time_s) - self.deceleration_start_s) / self.deceleration_s),
            )
            scale = 1.0 - progress
            return (
                self.command_x_mps * scale,
                self.command_y_mps * scale,
                self.command_yaw_rps * scale,
            )
        return (0.0, 0.0, 0.0)


def load_urdf_joint_limits(
    path: str | Path,
    required_names: Sequence[str],
) -> dict[str, tuple[float, float]]:
    """Load finite limits for an exact set of revolute/prismatic joints."""

    root = ET.parse(Path(path)).getroot()
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name not in required_names:
            continue
        if joint.get("type") not in {"revolute", "prismatic"}:
            raise ValueError(f"required joint {name} does not have finite URDF limits")
        limit = joint.find("limit")
        if limit is None or limit.get("lower") is None or limit.get("upper") is None:
            raise ValueError(f"required joint {name} lacks a URDF limit")
        lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        if not all(math.isfinite(value) for value in (lower, upper)) or lower >= upper:
            raise ValueError(f"required joint {name} has an invalid URDF limit")
        limits[str(name)] = (lower, upper)
    missing = sorted(set(required_names) - set(limits))
    if missing:
        raise ValueError(f"URDF joint-limit map is incomplete: {missing}")
    return {name: limits[name] for name in required_names}


def load_urdf_joint_velocity_limits(
    path: str | Path,
    required_names: Sequence[str],
) -> dict[str, float]:
    """Load positive finite velocity limits for the exact joint set."""

    root = ET.parse(Path(path)).getroot()
    limits: dict[str, float] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name not in required_names:
            continue
        limit = joint.find("limit")
        if limit is None or limit.get("velocity") is None:
            raise ValueError(f"required joint {name} lacks a URDF velocity limit")
        velocity = float(limit.get("velocity"))
        if not math.isfinite(velocity) or velocity <= 0.0:
            raise ValueError(f"required joint {name} has an invalid URDF velocity limit")
        limits[str(name)] = velocity
    missing = sorted(set(required_names) - set(limits))
    if missing:
        raise ValueError(f"URDF joint-velocity map is incomplete: {missing}")
    return {name: limits[name] for name in required_names}


def normalize_quaternion_wxyz(values: Sequence[float]) -> tuple[float, float, float, float]:
    result = tuple(float(value) for value in values)
    if len(result) != 4 or not all(math.isfinite(value) for value in result):
        raise ValueError("quaternion must contain four finite values")
    norm = math.sqrt(sum(value * value for value in result))
    if norm <= 1e-12:
        raise ValueError("quaternion has zero norm")
    return tuple(value / norm for value in result)  # type: ignore[return-value]


def quaternion_rpy_wxyz(values: Sequence[float]) -> tuple[float, float, float]:
    w, x, y, z = normalize_quaternion_wxyz(values)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_term)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def percentile(values: Sequence[float], quantile: float) -> float | None:
    if not values:
        return None
    if not 0.0 <= quantile <= 1.0:
        raise ValueError("quantile must be within [0, 1]")
    ordered = sorted(float(value) for value in values)
    position = quantile * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction
