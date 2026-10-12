import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import sys


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "admin/spark/compare-x2-locomotion-ab.py"
SPEC = importlib.util.spec_from_file_location("compare_x2_locomotion_ab", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

FULL_JOINT_NAMES = MODULE.LEG_JOINT_NAMES + tuple(
    f"test_aux_joint_{index}" for index in range(
        MODULE.EXPECTED_JOINT_COUNT - len(MODULE.LEG_JOINT_NAMES)
    )
)


def sample(
    authority: str,
    *,
    yaw: float = 0.0,
    body_forward: float | None = None,
    local_forward: float | None = None,
    local_cross_track: float = 0.02,
    slip: float = 0.01,
    turn_angle: float | None = None,
    steady_yaw_rate: float | None = None,
    stop_distance: float = 0.10,
    stop_p95: float = 0.01,
    stop_maximum: float = 0.02,
    stop_drift: float = 0.01,
    target_saturation_count: int = 0,
    target_saturation_ratio: float = 0.0,
    target_maximum_excess: float = 0.0,
) -> dict:
    command_x = 0.2 if abs(yaw) > 1e-9 else 0.3
    expected_forward = command_x * 8.0
    body_forward = expected_forward if body_forward is None else body_forward
    local_forward = body_forward if local_forward is None else local_forward
    turn_angle = yaw * 8.0 if turn_angle is None else turn_angle
    steady_yaw_rate = yaw if steady_yaw_rate is None else steady_yaw_rate
    profile = {
        "supported_stand_s": 3.0,
        "band_ramp_s": 5.0,
        "unsupported_stand_s": 3.0,
        "walk_s": 8.0,
        "deceleration_s": 2.0,
        "cpg_stand_handoff_s": 1.2,
        "stop_s": 8.0,
        "command_x_mps": command_x,
        "command_y_mps": 0.0,
        "command_yaw_rps": yaw,
    }
    checks = {name: True for name in MODULE.SEALED_INPUT_CHECKS}
    result = {
        "status": "PASSED",
        "qualification": False,
        "physics_authority": authority,
        "real_robot_commands": False,
        "published_command_topics": [],
        "profile": profile,
        "velocity_command_calibration": {
            "forward_scale": 0.735,
            "lateral_scale": 1.0,
            "yaw_scale": 1.0,
        },
        "position_target_safety_envelope": {
            "schema": "sceneforge/x2-position-target-safety-envelope/v1",
            "maximum_saturation_ratio": 0.01,
            "maximum_raw_limit_excess_rad": 0.02,
        },
        "checks": checks,
        "metrics": {
            # Deliberately unrelated world-frame values.  The comparator must
            # use the walk-entry/body-frame metrics below instead.
            "forward_displacement_m": -100.0,
            "lateral_displacement_m": 100.0,
            "walk_body_forward_distance_m": body_forward,
            "walk_local_forward_displacement_m": local_forward,
            "walk_local_cross_track_m": local_cross_track,
            "walk_local_cross_track_per_forward_m": (
                local_cross_track / max(abs(local_forward), 1e-9)
            ),
            "minimum_base_height_m": 0.63,
            "maximum_tilt_rad": 0.12,
            "left_support_slip_p95_mps": slip,
            "right_support_slip_p95_mps": slip,
            "left_steady_support_slip_p95_mps": slip,
            "right_steady_support_slip_p95_mps": slip,
            "walk_yaw_drift_rad_per_forward_m": 0.01,
            "steady_forward_speed_mps": command_x,
            "steady_forward_speed_relative_error": 0.0,
            "walk_turn_angle_rad": turn_angle,
            "steady_yaw_rate_rps": steady_yaw_rate,
            "steady_yaw_rate_relative_error": 0.0,
            "stop_distance_xy_m": stop_distance,
            "stable_stop": {
                "linear_speed_p95_mps": stop_p95,
                "angular_speed_p95_rps": stop_p95,
                "maximum_linear_speed_mps": stop_maximum,
                "maximum_angular_speed_rps": stop_maximum,
                "planar_drift_m": stop_drift,
                "observed_duration_s": 2.0,
                "sample_count": 200 if authority == "google_mujoco_3_3_5" else 1000,
            },
            "effort_saturation_ratio": 0.0,
            "final_linear_speed_mps": 0.0,
            "final_angular_speed_rps": 0.0,
            "joint_limit_violation_count": 0,
            "maximum_joint_limit_excess_rad": 0.0,
            "position_target_limit_violation_count": 0,
            "maximum_position_target_limit_excess_rad": 0.0,
            "maximum_joint_velocity_ratio": 1.0,
            "maximum_joint_velocity_exceedance_duration_s": 0.0,
            "raw_policy_position_target_sample_count": 1000,
            "raw_policy_position_target_saturation_count": target_saturation_count,
            "raw_policy_position_target_saturation_ratio": target_saturation_ratio,
            "maximum_raw_policy_position_target_limit_excess_rad": (
                target_maximum_excess
            ),
        },
        "joint_names": list(FULL_JOINT_NAMES),
    }
    if authority == "google_mujoco_3_3_5":
        result["runtime"] = {
            "model_sha256": "1" * 64,
            "stand_model_sha256": "2" * 64,
            "walk_model_sha256": "3" * 64,
        }
    else:
        result["model"] = {
            "stand_sha256": "2" * 64,
            "walk_sha256": "3" * 64,
        }
    result["parameters"] = {
        "joint_limit_source_sha256": "4" * 64,
        "joint_position_limits_rad": {
            name: [-3.14, 3.14] for name in FULL_JOINT_NAMES
        },
        "joint_velocity_limits_rps": {
            name: 10.0 for name in FULL_JOINT_NAMES
        },
    }
    if authority == "genie_sim_isaac_physx":
        result["parameters"]["articulation_joint_count"] = (
            MODULE.EXPECTED_JOINT_COUNT
        )
    return result


def write_phase_traces(root: Path) -> None:
    rows = []
    for index in range(72):
        phase = -math.pi + (index + 0.5) * 2.0 * math.pi / 72.0
        rows.append((2.01 + 0.05 * index, phase))
    rows = [(0.0, rows[0][1]), *rows, (8.0, rows[-1][1])]

    mujoco_samples = []
    physx_states = []
    physx_bases = []
    for sequence, (sim_time, phase) in enumerate(rows, start=1):
        mujoco_samples.append({
            "sim_time": sim_time,
            "phase": "GAIT1_FORWARD",
            "cpg_phase_rad": phase,
            "joint_position": [0.0] * len(FULL_JOINT_NAMES),
        })
        physx_states.append({
            "physics_sequence": sequence,
            "sim_time": sim_time,
            "phase": "GAIT1_FORWARD",
            "position": [0.01] * len(FULL_JOINT_NAMES),
        })
        physx_bases.append({
            "physics_sequence": sequence,
            "sim_time": sim_time,
            "phase": "GAIT1_FORWARD",
            "cpg_phase_rad": phase,
        })

    (root / "trace.json").write_text(json.dumps({
        "physics_authority": "google_mujoco_3_3_5",
        "samples": mujoco_samples,
        "real_robot_commands": False,
    }), encoding="utf-8")
    (root / "gait1-trace.json").write_text(json.dumps({
        "physics_authority": "genie_sim_isaac_physx",
        "joint_names": list(FULL_JOINT_NAMES),
        "physics_samples": physx_states,
        "base_samples": physx_bases,
        "real_robot_commands": False,
    }), encoding="utf-8")


def write_results(root: Path, mujoco: dict, physx: dict) -> tuple[Path, Path]:
    mujoco_path = root / "m.json"
    physx_path = root / "p.json"
    mujoco_path.write_text(json.dumps(mujoco), encoding="utf-8")
    physx_path.write_text(
        json.dumps({
            "status": "PASSED",
            "qualification": False,
            "published_command_topics": [],
            "real_robot_commands": False,
            "capture": physx,
        }),
        encoding="utf-8",
    )
    return mujoco_path, physx_path


def write_direct_physx_result(root: Path, physx: dict, *, trace_hash: str) -> Path:
    path = root / "physx-result.json"
    physx["trace"] = {
        "path": "/project/runs/cases/turn/state-command-trace.json",
        "sha256": trace_hash,
    }
    path.write_text(json.dumps(physx), encoding="utf-8")
    return path


class CompareX2LocomotionAbTests(unittest.TestCase):
    def test_direct_physx_result_uses_bound_state_command_sibling(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            state_trace = root / "state-command-trace.json"
            state_trace.write_bytes((root / "gait1-trace.json").read_bytes())
            expected_trace_hash = MODULE.sha256(state_trace)
            # A direct result must not consult this legacy wrapper filename.
            (root / "gait1-trace.json").write_text("not-json", encoding="utf-8")
            mujoco_path = root / "m.json"
            mujoco_path.write_text(
                json.dumps(sample("google_mujoco_3_3_5", yaw=0.25)),
                encoding="utf-8",
            )
            physx_path = write_direct_physx_result(
                root,
                sample("genie_sim_isaac_physx", yaw=0.25),
                trace_hash=expected_trace_hash,
            )
            report = MODULE.build_report(mujoco_path, physx_path)

        trace_input = report["phase_aligned_leg_joint_comparison"][
            "trace_inputs"
        ]["physx"]
        self.assertTrue(report["phase_aligned_leg_joint_comparison"]["available"])
        self.assertTrue(trace_input["path"].endswith("state-command-trace.json"))
        self.assertEqual(trace_input["sha256"], expected_trace_hash)

    def test_direct_physx_result_missing_state_command_sibling_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            mujoco_path = root / "m.json"
            mujoco_path.write_text(
                json.dumps(sample("google_mujoco_3_3_5", yaw=0.25)),
                encoding="utf-8",
            )
            physx_path = write_direct_physx_result(
                root,
                sample("genie_sim_isaac_physx", yaw=0.25),
                trace_hash="5" * 64,
            )
            report = MODULE.build_report(mujoco_path, physx_path)

        phase = report["phase_aligned_leg_joint_comparison"]
        self.assertEqual(report["status"], "FAILED")
        self.assertFalse(phase["available"])
        self.assertEqual(phase["reason"], "trace_missing")
        self.assertTrue(
            phase["trace_inputs"]["physx"]["expected_path"].endswith(
                "state-command-trace.json"
            )
        )

    def test_direct_physx_result_rejects_sibling_hash_mismatch(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            state_trace = root / "state-command-trace.json"
            state_trace.write_bytes((root / "gait1-trace.json").read_bytes())
            mujoco_path = root / "m.json"
            mujoco_path.write_text(
                json.dumps(sample("google_mujoco_3_3_5", yaw=0.25)),
                encoding="utf-8",
            )
            physx_path = write_direct_physx_result(
                root,
                sample("genie_sim_isaac_physx", yaw=0.25),
                trace_hash="6" * 64,
            )
            with self.assertRaisesRegex(ValueError, "sibling trace SHA-256 mismatch"):
                MODULE.build_report(mujoco_path, physx_path)

    def test_report_distinguishes_execution_from_equivalence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            m, p = write_results(
                root,
                sample("google_mujoco_3_3_5", slip=0.16),
                sample("genie_sim_isaac_physx", slip=0.70),
            )
            report = MODULE.build_report(m, p)
        self.assertEqual(report["execution_status"], "COMPLETED")
        self.assertEqual(report["gate_status"], "FAILED")
        self.assertEqual(report["status"], "FAILED")
        self.assertFalse(report["functional_equivalence"])
        self.assertEqual(report["decision"], "dynamics_alignment_required")
        self.assertFalse(report["checks"]["physx_steady_support_slip"])
        self.assertTrue(report["checks"]["real_robot_commands_false"])

    def test_missing_joint_trace_is_not_relabelled_as_equivalence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            m, p = write_results(
                root,
                sample("google_mujoco_3_3_5"),
                sample("genie_sim_isaac_physx"),
            )
            report = MODULE.build_report(m, p)
        self.assertFalse(report["common_metric_subset_passed"])
        self.assertFalse(report["functional_equivalence"])
        self.assertEqual(report["decision"], "dynamics_alignment_required")
        self.assertFalse(report["checks"]["phase_aligned_leg_joint_rms"])
        self.assertIn(
            "phase_aligned_leg_joint_rms_and_peak",
            report["remaining_required_metrics"],
        )

    def test_straight_uses_local_metrics_instead_of_world_xy(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mujoco = sample("google_mujoco_3_3_5", local_cross_track=0.02)
            physx = sample("genie_sim_isaac_physx", local_cross_track=0.03)
            write_phase_traces(root)
            m, p = write_results(root, mujoco, physx)
            report = MODULE.build_report(m, p)
        self.assertEqual(report["scenario"], "straight")
        self.assertTrue(report["case_gate_passed"])
        self.assertTrue(report["checks"]["mujoco_local_cross_track"])
        self.assertNotIn("mean_speed_proxy_between_engines", report["checks"])
        self.assertNotIn("forward_displacement_m", report["mujoco"])
        self.assertEqual(report["gate_status"], "PASSED")

    def test_bounded_raw_target_saturation_is_explicit_and_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            m, p = write_results(
                root,
                sample("google_mujoco_3_3_5"),
                sample(
                    "genie_sim_isaac_physx",
                    target_saturation_count=1,
                    target_saturation_ratio=0.001,
                    target_maximum_excess=0.008,
                ),
            )
            report = MODULE.build_report(m, p)
        self.assertTrue(report["case_gate_passed"])
        self.assertTrue(
            report["checks"]["physx_policy_position_target_saturation_bounded"]
        )
        self.assertEqual(
            report["delta"][
                "raw_policy_position_target_saturation_count_physx_minus_mujoco"
            ],
            1.0,
        )

    def test_report_carries_calibration_and_frozen_input_hash_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            m, p = write_results(
                root,
                sample("google_mujoco_3_3_5"),
                sample("genie_sim_isaac_physx"),
            )
            report = MODULE.build_report(m, p)
        self.assertEqual(
            report["velocity_command_calibration"],
            {"forward_scale": 0.735, "lateral_scale": 1.0, "yaw_scale": 1.0},
        )
        self.assertEqual(report["frozen_input_hashes"]["mujoco_model_sha256"], "1" * 64)
        self.assertEqual(report["frozen_input_hashes"]["physx_walk_policy_sha256"], "3" * 64)
        self.assertEqual(
            report["frozen_input_hashes"]["physx_joint_limit_source_sha256"],
            "4" * 64,
        )
        self.assertTrue(report["checks"]["same_stand_policy_hash"])
        self.assertTrue(report["checks"]["same_walk_policy_hash"])
        self.assertTrue(report["checks"]["same_joint_limit_source_hash"])

    def test_report_seals_full_joint_limits_targets_and_stop_window(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            m, p = write_results(
                root,
                sample("google_mujoco_3_3_5"),
                sample("genie_sim_isaac_physx"),
            )
            report = MODULE.build_report(m, p)

        for side_name, expected_samples in (("mujoco", 200), ("physx", 1000)):
            facts = report[side_name]["sealed_facts"]
            for count_name in (
                "joint_state_count",
                "joint_target_count",
                "applied_position_target_count",
                "joint_position_limit_count",
                "joint_velocity_limit_count",
            ):
                self.assertEqual(facts[count_name], MODULE.EXPECTED_JOINT_COUNT)
            self.assertEqual(facts["joint_position_limit_violation_count"], 0)
            self.assertEqual(
                facts["maximum_joint_position_limit_excess_rad"], 0.0
            )
            self.assertEqual(
                facts["applied_position_target_limit_violation_count"], 0
            )
            self.assertEqual(
                facts["maximum_applied_position_target_limit_excess_rad"], 0.0
            )
            self.assertEqual(facts["stable_stop_window_duration_s"], 2.0)
            self.assertEqual(facts["stable_stop_sample_count"], expected_samples)
        self.assertTrue(report["checks"]["mujoco_full_31_joint_coverage"])
        self.assertTrue(report["checks"]["physx_full_31_joint_coverage"])
        self.assertTrue(report["checks"]["mujoco_stable_stop_window_valid"])
        self.assertTrue(report["checks"]["physx_stable_stop_window_valid"])

    def test_joint_coverage_below_31_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            mujoco = sample("google_mujoco_3_3_5")
            removed = mujoco["joint_names"].pop()
            del mujoco["parameters"]["joint_position_limits_rad"][removed]
            del mujoco["parameters"]["joint_velocity_limits_rps"][removed]
            m, p = write_results(
                root, mujoco, sample("genie_sim_isaac_physx")
            )
            report = MODULE.build_report(m, p)

        self.assertEqual(
            report["mujoco"]["sealed_facts"]["joint_state_count"], 30
        )
        self.assertFalse(report["checks"]["mujoco_full_31_joint_coverage"])
        self.assertEqual(report["gate_status"], "FAILED")

    def test_applied_target_limit_violation_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            physx = sample("genie_sim_isaac_physx")
            physx["metrics"]["position_target_limit_violation_count"] = 1
            physx["metrics"]["maximum_position_target_limit_excess_rad"] = 0.01
            m, p = write_results(
                root, sample("google_mujoco_3_3_5"), physx
            )
            report = MODULE.build_report(m, p)

        facts = report["physx"]["sealed_facts"]
        self.assertEqual(facts["applied_position_target_limit_violation_count"], 1)
        self.assertEqual(
            facts["maximum_applied_position_target_limit_excess_rad"], 0.01
        )
        self.assertFalse(
            report["checks"]["physx_full_joint_state_and_target_within_limits"]
        )
        self.assertEqual(report["gate_status"], "FAILED")

    def test_short_or_undersampled_stable_stop_window_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            mujoco = sample("google_mujoco_3_3_5")
            mujoco["metrics"]["stable_stop"]["observed_duration_s"] = 1.5
            physx = sample("genie_sim_isaac_physx")
            physx["metrics"]["stable_stop"]["sample_count"] = 99
            m, p = write_results(root, mujoco, physx)
            report = MODULE.build_report(m, p)

        self.assertFalse(report["checks"]["mujoco_stable_stop_window_valid"])
        self.assertFalse(report["checks"]["physx_stable_stop_window_valid"])
        self.assertEqual(report["gate_status"], "FAILED")

    def test_policy_hash_mismatch_fails_ab_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            mujoco = sample("google_mujoco_3_3_5")
            physx = sample("genie_sim_isaac_physx")
            physx["model"]["walk_sha256"] = "9" * 64
            m, p = write_results(root, mujoco, physx)
            report = MODULE.build_report(m, p)
        self.assertEqual(report["status"], "FAILED")
        self.assertFalse(report["checks"]["same_walk_policy_hash"])

    def test_cli_preserves_failed_receipt_and_returns_nonzero(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            mujoco = sample("google_mujoco_3_3_5")
            physx = sample("genie_sim_isaac_physx")
            physx["model"]["walk_sha256"] = "9" * 64
            m, p = write_results(root, mujoco, physx)
            output = root / "ab" / "result.json"
            with mock.patch.object(
                sys,
                "argv",
                [str(SCRIPT), "--mujoco", str(m), "--physx", str(p),
                 "--output", str(output)],
            ):
                return_code = MODULE.main()
            persisted = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(return_code, 2)
        self.assertEqual(persisted["execution_status"], "COMPLETED")
        self.assertEqual(persisted["status"], "FAILED")

    def test_left_turn_uses_full_angle_rate_and_signed_direction(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            m, p = write_results(
                root,
                sample(
                    "google_mujoco_3_3_5", yaw=0.25,
                    turn_angle=2.00, steady_yaw_rate=0.25,
                    local_cross_track=1.2,
                ),
                sample(
                    "genie_sim_isaac_physx", yaw=0.25,
                    turn_angle=1.95, steady_yaw_rate=0.245,
                    local_cross_track=1.1,
                ),
            )
            report = MODULE.build_report(m, p)
        self.assertEqual(report["scenario"], "turn_left")
        self.assertTrue(report["case_gate_passed"])
        self.assertTrue(report["checks"]["physx_full_walk_turn_angle"])
        self.assertTrue(report["checks"]["steady_yaw_rate_between_engines"])
        self.assertNotIn("physx_local_cross_track", report["checks"])

    def test_right_turn_rejects_wrong_signed_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            m, p = write_results(
                root,
                sample(
                    "google_mujoco_3_3_5", yaw=-0.25,
                    turn_angle=-2.0, steady_yaw_rate=-0.25,
                ),
                sample(
                    "genie_sim_isaac_physx", yaw=-0.25,
                    turn_angle=2.0, steady_yaw_rate=0.25,
                ),
            )
            report = MODULE.build_report(m, p)
        self.assertEqual(report["scenario"], "turn_right")
        self.assertFalse(report["case_gate_passed"])
        self.assertFalse(report["checks"]["physx_turn_direction"])

    def test_turn_rejects_full_angle_outside_tolerance(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            m, p = write_results(
                root,
                sample("google_mujoco_3_3_5", yaw=0.25, turn_angle=2.0),
                sample("genie_sim_isaac_physx", yaw=0.25, turn_angle=1.70),
            )
            report = MODULE.build_report(m, p)
        self.assertFalse(report["case_gate_passed"])
        self.assertFalse(report["checks"]["physx_full_walk_turn_angle"])

    def test_stable_stop_requires_p95_maximum_and_drift(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_phase_traces(root)
            m, p = write_results(
                root,
                sample("google_mujoco_3_3_5"),
                sample(
                    "genie_sim_isaac_physx",
                    stop_p95=0.01,
                    stop_maximum=0.08,
                    stop_drift=0.01,
                ),
            )
            report = MODULE.build_report(m, p)
        self.assertFalse(report["case_gate_passed"])
        self.assertTrue(report["checks"]["physx_stable_stop_p95"])
        self.assertFalse(report["checks"]["physx_stable_stop_maximum"])

    def test_rejects_missing_or_failed_sealed_input_check(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bad = sample("google_mujoco_3_3_5")
            del bad["checks"]["no_fall"]
            path = root / "bad.json"
            path.write_text(json.dumps(bad), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "lacks sealed checks"):
                MODULE.load_result(path, "google_mujoco_3_3_5")

            bad = sample("google_mujoco_3_3_5")
            bad["checks"]["no_nan_or_inf"] = False
            path = root / "failed.json"
            path.write_text(json.dumps(bad), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "failed sealed checks"):
                MODULE.load_result(path, "google_mujoco_3_3_5")

    def test_rejects_relabelled_real_robot_or_nonfinite_input(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            payload = sample("google_mujoco_3_3_5")
            payload["real_robot_commands"] = True
            path = root / "real.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "real_robot_commands"):
                MODULE.load_result(path, "google_mujoco_3_3_5")

            payload = sample("google_mujoco_3_3_5")
            payload["metrics"]["steady_forward_speed_mps"] = float("nan")
            path = root / "nan.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "non-finite JSON"):
                MODULE.load_result(path, "google_mujoco_3_3_5")

    def test_rejects_unsafe_physx_wrapper_even_when_capture_is_safe(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture = sample("genie_sim_isaac_physx")
            path = root / "unsafe-wrapper.json"
            path.write_text(json.dumps({
                "status": "PASSED",
                "qualification": False,
                "real_robot_commands": True,
                "published_command_topics": [],
                "capture": capture,
            }), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "envelope real_robot_commands"):
                MODULE.load_result(path, "genie_sim_isaac_physx")

    def test_atomic_output_forbids_nan_and_removes_temporary_file(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "new" / "result.json"
            with self.assertRaises(ValueError):
                MODULE.atomic_write(output, {"bad": float("nan")})
            self.assertFalse(output.exists())
            self.assertFalse((output.parent / ".result.json.tmp").exists())


if __name__ == "__main__":
    unittest.main()
