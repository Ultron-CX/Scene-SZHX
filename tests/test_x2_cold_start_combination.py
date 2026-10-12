from dataclasses import dataclass
from pathlib import Path
import tempfile
import unittest

from sceneforge.x2_cold_start_combination import (
    SCENARIOS,
    X2ColdStartCombinationCoordinator,
    X2CombinationCasePlan,
    X2ContinuationMotionProfile,
    perform_sim_time_freeze_probe,
)
from sceneforge.x2_mujoco_motion import X2MujocoMotionProfile


@dataclass(frozen=True)
class Hopf:
    mode: int = 1
    gait: int = 1
    walk_initialized: bool = True
    period: float = 0.87
    obs4: tuple[float, float, float, float] = (0.0, 1.0, 0.0, -1.0)
    filtered_velocity: tuple[float, float, float] = (0.3, 0.0, 0.0)
    filtered_command: tuple[float, float] = (0.3, 0.0)


class SnapshotObject:
    def __init__(self, value):
        self.value = value

    def snapshot(self):
        return self.value


class WalkSession:
    def __init__(self):
        self.hopf = SnapshotObject(Hopf())

    def state_digest(self):
        return "a" * 64


class FakeCapture:
    def __init__(self, scenario, *, fail=False):
        self.scenario = scenario
        self.fail = fail
        self.completed = False
        self.status = "RUNNING"
        self.freeze_probe = {"status": "PASSED", "resume_completed": False}
        self.first_sim_time = 0.0
        self.physics_sequence = 3
        self.command_sequence = 3
        self.walk_sequence = 3
        self.walk_policy = type("Policy", (), {"elapsed_ms": [1.0, 1.0]})()
        self.walk_session = WalkSession()
        self.velocity_filter = SnapshotObject({"x": 0.3})
        self.heading_hold = SnapshotObject({"yaw": 0.0})
        self.cross_track_hold = SnapshotObject({"y": 0.0})
        self.position_target_safety = SnapshotObject({"bounded": True})
        self.next_walk_time = 1.0
        self.current_target = {"joint": 0.0}
        self.current_kp = {"joint": 1.0}
        self.current_kd = {"joint": 0.1}
        self.last_policy_action = (0.0,)
        self.trace_records = []
        self.base_records = []
        self.contact_records = []
        self.last_effective_command = (0.3, 0.0, 0.0)
        self.last_cross_track_m = 0.0
        self.last_cross_track_command_y_mps = 0.0
        self.last_heading_hold_command_yaw_rps = 0.0
        self.profile = X2MujocoMotionProfile()
        self.stop_count = 0
        self.step_times = []

    def step(self, sim_time):
        self.step_times.append(sim_time)
        self.completed = True
        self.status = "FAILED" if self.fail else "PASSED"

    def request_authoritative_stop(self):
        self.stop_count += 1


class X2ColdStartCombinationTests(unittest.TestCase):
    def plans(self, root):
        return tuple(
            X2CombinationCasePlan(
                scenario=scenario,
                profile=(
                    X2MujocoMotionProfile()
                    if index == 0
                    else X2ContinuationMotionProfile()
                ),
                receipt=root / scenario / "physx-result.json",
                trace=root / scenario / "trace.json",
                performs_suspension_transition=index == 0,
            )
            for index, scenario in enumerate(SCENARIOS)
        )

    def test_continuation_profile_never_claims_band_transition(self):
        profile = X2ContinuationMotionProfile(unsupported_stand_s=2.0)
        self.assertEqual(profile.phase(0.0), "UNSUPPORTED_STAND")
        self.assertEqual(profile.phase(1.999), "UNSUPPORTED_STAND")
        self.assertEqual(profile.phase(2.0), "GAIT1_FORWARD")
        self.assertNotIn(
            "BAND_RAMP_DOWN",
            {profile.phase(value / 10.0) for value in range(0, 500)},
        )

    def test_three_cases_share_coordinator_and_only_final_requests_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            captures = []

            def factory(plan):
                capture = FakeCapture(plan.scenario)
                captures.append(capture)
                return capture

            coordinator = X2ColdStartCombinationCoordinator(
                self.plans(root), factory
            )
            coordinator.step(1.0)
        self.assertTrue(coordinator.completed)
        self.assertEqual(coordinator.status, "PASSED")
        self.assertEqual([item["scenario"] for item in coordinator.completed_cases], list(SCENARIOS))
        self.assertEqual([capture.stop_count for capture in captures], [0, 0, 1])
        self.assertEqual([capture.step_times for capture in captures], [[1.0], [1.0], [1.0]])

    def test_failure_stops_same_lifecycle_without_starting_later_cases(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            captures = []

            def factory(plan):
                capture = FakeCapture(plan.scenario, fail=plan.scenario == "turn_left")
                captures.append(capture)
                return capture

            coordinator = X2ColdStartCombinationCoordinator(
                self.plans(root), factory
            )
            coordinator.step(1.0)
        self.assertTrue(coordinator.completed)
        self.assertEqual(coordinator.status, "FAILED")
        self.assertEqual([capture.scenario for capture in captures], ["straight", "turn_left"])
        self.assertEqual([capture.stop_count for capture in captures], [0, 1])

    def test_freeze_probe_measures_all_lanes_without_advancement(self):
        capture = FakeCapture("straight")
        capture.freeze_probe = None
        counters = iter((100.0, 102.1))
        probe = perform_sim_time_freeze_probe(
            capture,
            sim_time=7.0,
            requested_wall_duration_s=2.0,
            sleep=lambda _seconds: None,
            perf_counter=lambda: next(counters),
        )
        self.assertEqual(probe["status"], "PASSED")
        for name in (
            "sim_time_advance_s",
            "clock_update_count",
            "physics_state_update_count",
            "controller_update_count",
            "policy_update_count",
            "command_update_count",
            "cpg_phase_advance_rad",
        ):
            self.assertEqual(probe[name], 0)
        self.assertEqual(
            probe["controller_digest_before"],
            probe["controller_digest_after"],
        )


if __name__ == "__main__":
    unittest.main()
