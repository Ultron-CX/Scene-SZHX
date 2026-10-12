#!/usr/bin/env python3
"""Classify whether the vendor X2 controller advances while sim time is frozen.

This process is deliberately read-only.  The authoritative PhysX engine owns
the bounded freeze request/status handshake; this probe only timestamps ROS
messages and produces an immutable receipt after the freeze has completed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import signal
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List


SCHEMA = "sceneforge/x2-controller-clock-freeze-probe/v1"
STATUS_SCHEMA = "sceneforge/physx-clock-freeze-status/v1"
LANES = ("leg", "waist", "arm", "head")


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
        os.chmod(path, 0o644)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def load_completed_status(path: Path) -> dict | None:
    if not path.is_file():
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema") != STATUS_SCHEMA:
        raise ValueError(f"unexpected freeze status schema: {payload.get('schema')!r}")
    if payload.get("real_robot_commands") is not False:
        raise ValueError("freeze status must contain real_robot_commands=false")
    return payload if payload.get("status") == "COMPLETED" else None


def count_interval(events: Iterable[dict], started_at: float, finished_at: float) -> List[dict]:
    return [event for event in events if started_at <= float(event["wall_time"]) <= finished_at]


def classify_freeze(
    status: dict,
    clock_events: List[dict],
    state_events: List[dict],
    command_events: Dict[str, List[dict]],
) -> tuple[str, dict]:
    started_at = float(status["started_at_unix"])
    finished_at = float(status["finished_at_unix"])
    sim_before = float(status["sim_time_before_s"])
    sim_after = float(status["sim_time_after_s"])
    in_clock = count_interval(clock_events, started_at, finished_at)
    in_state = count_interval(state_events, started_at, finished_at)
    in_commands = {
        lane: count_interval(command_events.get(lane, []), started_at, finished_at)
        for lane in LANES
    }
    counts = {lane: len(events) for lane, events in in_commands.items()}
    unique_stamps = {
        lane: len({int(event["stamp_ns"]) for event in events})
        for lane, events in in_commands.items()
    }
    fingerprint_changes = {}
    for lane, events in in_commands.items():
        fingerprints = [event["fingerprint"] for event in events]
        fingerprint_changes[lane] = sum(
            before != after for before, after in zip(fingerprints, fingerprints[1:])
        )

    engine_frozen = (
        math.isclose(sim_before, sim_after, rel_tol=0.0, abs_tol=1e-12)
        and len(in_clock) <= 1
        and len(in_state) <= 1
    )
    # A single queued callback at either edge is tolerated.  Continuous output
    # from every lane during a two-second freeze proves a wall-clock loop.
    if not engine_frozen:
        classification = "FREEZE_NOT_PROVEN"
    elif all(count <= 1 for count in counts.values()):
        classification = "SIM_TIME_BOUND"
    elif all(count >= 5 for count in counts.values()):
        classification = "CONTROLLER_WALL_CLOCK_BOUND"
    else:
        classification = "AMBIGUOUS"

    return classification, {
        "engine_frozen": engine_frozen,
        "clock_messages_during_freeze": len(in_clock),
        "physics_state_messages_during_freeze": len(in_state),
        "command_messages_during_freeze": counts,
        "unique_command_stamps_during_freeze": unique_stamps,
        "command_fingerprint_changes_during_freeze": fingerprint_changes,
    }


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def fingerprint(values: Iterable[float]) -> str:
    encoded = ",".join(f"{float(value):.12g}" for value in values).encode("ascii")
    return hashlib.sha256(encoded).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--freeze-status", required=True)
    parser.add_argument("--receipt", required=True)
    parser.add_argument("--wall-timeout-s", type=float, default=90.0)
    parser.add_argument("--post-freeze-grace-s", type=float, default=0.25)
    args = parser.parse_args()
    if args.wall_timeout_s <= 0 or args.post_freeze_grace_s < 0:
        parser.error("timeouts must be non-negative and wall timeout must be positive")

    import rclpy
    from my_ros2_proto.msg import JointCommand
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    class Probe(Node):
        def __init__(self) -> None:
            super().__init__("x2_controller_clock_freeze_probe")
            self.clock_events: List[dict] = []
            self.state_events: List[dict] = []
            self.command_events: Dict[str, List[dict]] = {lane: [] for lane in LANES}
            self.create_subscription(Clock, "/clock", self.on_clock, qos)
            self.create_subscription(JointState, "/joint_states", self.on_state, qos)
            for lane in LANES:
                self.create_subscription(
                    JointCommand,
                    f"/joint_cmd/{lane}",
                    lambda message, selected=lane: self.on_command(selected, message),
                    qos,
                )

        def on_clock(self, message: Clock) -> None:
            self.clock_events.append({"wall_time": time.time(), "stamp_ns": stamp_ns(message.clock)})

        def on_state(self, message: JointState) -> None:
            self.state_events.append({"wall_time": time.time(), "stamp_ns": stamp_ns(message.header.stamp)})

        def on_command(self, lane: str, message: JointCommand) -> None:
            values = list(message.position) + list(message.velocity) + list(message.effort)
            self.command_events[lane].append({
                "wall_time": time.time(),
                "stamp_ns": stamp_ns(message.header.stamp),
                "fingerprint": fingerprint(values),
            })

    status_path = Path(args.freeze_status)
    receipt_path = Path(args.receipt)
    if receipt_path.exists():
        raise FileExistsError(f"refusing to overwrite receipt: {receipt_path}")

    rclpy.init()
    node = Probe()
    stopped = False
    started_at = time.time()

    def request_stop(_signum, _frame) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    status = None
    error = None
    try:
        while rclpy.ok() and not stopped and time.time() - started_at < args.wall_timeout_s:
            rclpy.spin_once(node, timeout_sec=0.05)
            status = load_completed_status(status_path)
            if status is not None and time.time() >= float(status["finished_at_unix"]) + args.post_freeze_grace_s:
                break
        if status is None:
            error = "freeze status did not reach COMPLETED before timeout or stop"
            classification = "PROBE_INCOMPLETE"
            metrics = {}
        else:
            classification, metrics = classify_freeze(
                status, node.clock_events, node.state_events, node.command_events
            )
    except Exception as exc:
        error = str(exc)
        classification = "PROBE_ERROR"
        metrics = {}
    finally:
        receipt = {
            "schema": SCHEMA,
            "status": "PASSED" if classification in ("SIM_TIME_BOUND", "CONTROLLER_WALL_CLOCK_BOUND") else "FAILED",
            "classification": classification,
            "error": error,
            "started_at_unix": started_at,
            "finished_at_unix": time.time(),
            "freeze_status": status,
            "metrics": metrics,
            "observed_totals": {
                "clock_messages": len(node.clock_events),
                "physics_state_messages": len(node.state_events),
                "command_messages": {lane: len(node.command_events[lane]) for lane in LANES},
            },
            "read_only": True,
            "subscribed_topics": [
                "/clock", "/joint_states", *(f"/joint_cmd/{lane}" for lane in LANES)
            ],
            "published_topics": [],
            "physics_authority": "genie_sim_isaac_physx",
            "mujoco_state_source_used": False,
            "real_robot_commands": False,
        }
        atomic_json(receipt_path, receipt)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if classification == "SIM_TIME_BOUND" else 3


if __name__ == "__main__":
    raise SystemExit(main())
