import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import sys


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "admin/spark/summarize-x2-turn-symmetry.py"
SPEC = importlib.util.spec_from_file_location("summarize_x2_turn_symmetry", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

AB_SCRIPT = ROOT / "admin/spark/compare-x2-locomotion-ab.py"
AB_SPEC = importlib.util.spec_from_file_location("compare_x2_locomotion_ab_for_pair", AB_SCRIPT)
AB_MODULE = importlib.util.module_from_spec(AB_SPEC)
assert AB_SPEC.loader is not None
AB_SPEC.loader.exec_module(AB_MODULE)

AB_FIXTURE_SCRIPT = ROOT / "tests/test_compare_x2_locomotion_ab.py"
AB_FIXTURE_SPEC = importlib.util.spec_from_file_location(
    "compare_x2_locomotion_ab_fixtures", AB_FIXTURE_SCRIPT
)
AB_FIXTURES = importlib.util.module_from_spec(AB_FIXTURE_SPEC)
assert AB_FIXTURE_SPEC.loader is not None
AB_FIXTURE_SPEC.loader.exec_module(AB_FIXTURES)


HASHES = {
    "mujoco_model_sha256": "1" * 64,
    "mujoco_stand_policy_sha256": "2" * 64,
    "mujoco_walk_policy_sha256": "3" * 64,
    "mujoco_joint_limit_source_sha256": "4" * 64,
    "physx_stand_policy_sha256": "2" * 64,
    "physx_walk_policy_sha256": "3" * 64,
    "physx_joint_limit_source_sha256": "4" * 64,
}
CALIBRATION = {
    "forward_scale": 0.735,
    "lateral_scale": 1.0,
    "yaw_scale": 1.0,
}
SAFETY = {
    "schema": "sceneforge/x2-position-target-safety-envelope/v1",
    "maximum_saturation_ratio": 0.01,
    "maximum_raw_limit_excess_rad": 0.02,
}


def receipt(
    scenario: str,
    *,
    yaw: float,
    mujoco_angle: float,
    physx_angle: float,
    mujoco_yaw: float | None = None,
    physx_yaw: float | None = None,
    mujoco_speed: float = 0.20,
    physx_speed: float = 0.20,
    mujoco_stop: float = 0.10,
    physx_stop: float = 0.10,
    hashes: dict | None = HASHES,
    calibration: dict | None = CALIBRATION,
) -> dict:
    mujoco_yaw = yaw if mujoco_yaw is None else mujoco_yaw
    physx_yaw = yaw if physx_yaw is None else physx_yaw

    def side(angle: float, rate: float, speed: float, stop: float) -> dict:
        return {
            "walk_turn_angle_rad": angle,
            "steady_yaw_rate_rps": rate,
            "steady_forward_speed_mps": speed,
            "stop_distance_xy_m": stop,
            "position_target_safety_contract": dict(SAFETY),
        }

    result = {
        "schema": MODULE.INPUT_SCHEMA,
        "execution_status": "COMPLETED",
        "gate_status": "PASSED",
        "status": "PASSED",
        "scenario": scenario,
        "case_gate_passed": True,
        "common_metric_subset_passed": True,
        "functional_equivalence": False,
        "qualification": False,
        "real_robot_commands": False,
        "checks": {
            "same_velocity_command_calibration": True,
            "same_position_target_safety_envelope": True,
            "real_robot_commands_false": True,
            "turn_direction": True,
        },
        "command": {
            "x_mps": 0.20,
            "y_mps": 0.0,
            "yaw_rps": yaw,
            "walk_s": 8.0,
        },
        "mujoco": side(mujoco_angle, mujoco_yaw, mujoco_speed, mujoco_stop),
        "physx": side(physx_angle, physx_yaw, physx_speed, physx_stop),
        "inputs": {
            "mujoco": {"path": "/evidence/m.json", "sha256": "a" * 64},
            "physx": {"path": "/evidence/p.json", "sha256": "b" * 64},
        },
        "published_command_topics": [],
    }
    if hashes is not None:
        result["frozen_input_hashes"] = dict(hashes)
    if calibration is not None:
        result["velocity_command_calibration"] = dict(calibration)
    return result


def pair(**overrides) -> tuple[dict, dict]:
    left_options = overrides.pop("left", {})
    right_options = overrides.pop("right", {})
    if overrides:
        raise AssertionError(f"unexpected overrides: {overrides}")
    left = receipt(
        "turn_left", yaw=0.25, mujoco_angle=2.00, physx_angle=1.96,
        **left_options,
    )
    right = receipt(
        "turn_right", yaw=-0.25, mujoco_angle=-1.96, physx_angle=-1.92,
        **right_options,
    )
    return left, right


def write_pair(root: Path, left: dict, right: dict) -> tuple[Path, Path]:
    left_path = root / "left-ab.json"
    right_path = root / "right-ab.json"
    left_path.write_text(json.dumps(left), encoding="utf-8")
    right_path.write_text(json.dumps(right), encoding="utf-8")
    return left_path, right_path


class SummarizeX2TurnSymmetryTests(unittest.TestCase):
    def test_consumes_exact_reports_emitted_by_ab_v2_comparator(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ab_paths = []
            for name, yaw in (("left", 0.25), ("right", -0.25)):
                case_root = root / name
                case_root.mkdir()
                AB_FIXTURES.write_phase_traces(case_root)
                mujoco_path, physx_path = AB_FIXTURES.write_results(
                    case_root,
                    AB_FIXTURES.sample(
                        "google_mujoco_3_3_5", yaw=yaw,
                        turn_angle=yaw * 8.0, steady_yaw_rate=yaw,
                    ),
                    AB_FIXTURES.sample(
                        "genie_sim_isaac_physx", yaw=yaw,
                        turn_angle=yaw * 8.0, steady_yaw_rate=yaw,
                    ),
                )
                ab_report = AB_MODULE.build_report(mujoco_path, physx_path)
                self.assertEqual(ab_report["status"], "PASSED")
                ab_path = root / f"{name}-ab.json"
                ab_path.write_text(json.dumps(ab_report), encoding="utf-8")
                ab_paths.append(ab_path)

            report = MODULE.build_report(*ab_paths)

        self.assertEqual(report["status"], "PASSED")
        self.assertTrue(report["checks"]["frozen_input_hashes_available"])
        self.assertTrue(
            report["checks"]["velocity_command_calibration_available"]
        )

    def test_passing_pair_reports_both_engine_differences(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left_path, right_path = write_pair(root, *pair())
            report = MODULE.build_report(left_path, right_path)

        self.assertEqual(report["execution_status"], "COMPLETED")
        self.assertEqual(report["gate_status"], "PASSED")
        self.assertEqual(report["status"], "PASSED")
        self.assertTrue(report["symmetry_gate_passed"])
        self.assertFalse(report["qualification"])
        self.assertFalse(report["real_robot_commands"])
        self.assertEqual(
            report["engines"]["physx"]["left"]["absolute_turn_angle_rad"],
            1.96,
        )
        self.assertAlmostEqual(
            report["engines"]["mujoco"]["difference"][
                "absolute_turn_angle_rad"
            ],
            0.04,
        )
        self.assertTrue(report["checks"]["physx_turn_angle_relative_difference"])
        self.assertTrue(
            report["shared_contracts"]["frozen_input_hashes_available"]
        )

    def test_rejects_an_input_whose_case_gate_did_not_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left, right = pair()
            right["case_gate_passed"] = False
            right["gate_status"] = "FAILED"
            right["status"] = "FAILED"
            left_path, right_path = write_pair(root, left, right)
            with self.assertRaisesRegex(ValueError, "gate_status"):
                MODULE.build_report(left_path, right_path)

    def test_rejects_non_opposite_command_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left, right = pair()
            right["command"]["yaw_rps"] = -0.20
            left_path, right_path = write_pair(root, left, right)
            report = MODULE.build_report(left_path, right_path)

        self.assertEqual(report["execution_status"], "COMPLETED")
        self.assertEqual(report["gate_status"], "FAILED")
        self.assertFalse(report["checks"]["opposite_yaw_commands"])

    def test_turn_angle_asymmetry_fails_only_the_metric_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left, right = pair()
            right["physx"]["walk_turn_angle_rad"] = -1.20
            left_path, right_path = write_pair(root, left, right)
            report = MODULE.build_report(left_path, right_path)

        self.assertEqual(report["status"], "FAILED")
        self.assertFalse(report["symmetry_gate_passed"])
        self.assertFalse(report["checks"]["physx_turn_angle_relative_difference"])
        self.assertTrue(report["checks"]["mujoco_turn_angle_relative_difference"])

    def test_stop_distance_uses_absolute_or_twenty_percent_tolerance(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left, right = pair()
            left["physx"]["stop_distance_xy_m"] = 1.00
            right["physx"]["stop_distance_xy_m"] = 1.19
            left_path, right_path = write_pair(root, left, right)
            report = MODULE.build_report(left_path, right_path)
        self.assertTrue(report["checks"]["physx_stop_distance_difference"])

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left, right = pair()
            left["physx"]["stop_distance_xy_m"] = 0.10
            right["physx"]["stop_distance_xy_m"] = 0.21
            left_path, right_path = write_pair(root, left, right)
            report = MODULE.build_report(left_path, right_path)
        self.assertFalse(report["checks"]["physx_stop_distance_difference"])

    def test_calibration_safety_or_hash_mismatch_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left, right = pair()
            right["velocity_command_calibration"]["yaw_scale"] = 0.9
            right["frozen_input_hashes"]["physx_walk_policy_sha256"] = "9" * 64
            right["mujoco"]["position_target_safety_contract"][
                "maximum_saturation_ratio"
            ] = 0.02
            right["physx"]["position_target_safety_contract"][
                "maximum_saturation_ratio"
            ] = 0.02
            left_path, right_path = write_pair(root, left, right)
            report = MODULE.build_report(left_path, right_path)

        self.assertEqual(report["status"], "FAILED")
        self.assertFalse(
            report["checks"]["same_position_target_safety_contract"]
        )
        self.assertFalse(
            report["checks"]["available_velocity_command_calibration_matches"]
        )
        self.assertFalse(report["checks"]["available_frozen_input_hashes_match"])

    def test_missing_frozen_identity_fails_closed_without_inventing_it(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left, right = pair(
                left={"hashes": None, "calibration": None},
                right={"hashes": None, "calibration": None},
            )
            left_path, right_path = write_pair(root, left, right)
            report = MODULE.build_report(left_path, right_path)

        self.assertEqual(report["status"], "FAILED")
        self.assertFalse(report["symmetry_gate_passed"])
        self.assertFalse(report["checks"]["frozen_input_hashes_available"])
        self.assertFalse(
            report["checks"]["velocity_command_calibration_available"]
        )
        self.assertFalse(
            report["shared_contracts"]["frozen_input_hashes_available"]
        )
        self.assertIsNone(report["shared_contracts"]["frozen_input_hashes"])
        self.assertFalse(
            report["shared_contracts"]["velocity_command_calibration_available"]
        )

    def test_rejects_nonfinite_and_real_robot_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left, right = pair()
            left["mujoco"]["walk_turn_angle_rad"] = float("nan")
            left_path, right_path = write_pair(root, left, right)
            with self.assertRaisesRegex(ValueError, "non-finite"):
                MODULE.build_report(left_path, right_path)

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left, right = pair()
            right["real_robot_commands"] = True
            left_path, right_path = write_pair(root, left, right)
            with self.assertRaisesRegex(ValueError, "real_robot_commands"):
                MODULE.build_report(left_path, right_path)

    def test_atomic_write_is_finite_and_never_overwrites(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "evidence" / "symmetry.json"
            MODULE.atomic_write(output, {"status": "PASSED"})
            original = output.read_bytes()
            with self.assertRaises(FileExistsError):
                MODULE.atomic_write(output, {"status": "FAILED"})
            self.assertEqual(output.read_bytes(), original)

            bad = root / "other" / "symmetry.json"
            with self.assertRaises(ValueError):
                MODULE.atomic_write(bad, {"metric": float("nan")})
            self.assertFalse(bad.exists())
            self.assertEqual(list(bad.parent.glob(".*.tmp")), [])

    def test_cli_preserves_failed_receipt_and_returns_nonzero(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            left, right = pair()
            right["physx"]["walk_turn_angle_rad"] = -1.0
            left_path, right_path = write_pair(root, left, right)
            output = root / "summary" / "result.json"
            with mock.patch.object(
                sys,
                "argv",
                [str(SCRIPT), "--left", str(left_path), "--right", str(right_path),
                 "--output", str(output)],
            ):
                return_code = MODULE.main()
            persisted = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(return_code, 2)
        self.assertEqual(persisted["execution_status"], "COMPLETED")
        self.assertEqual(persisted["status"], "FAILED")


if __name__ == "__main__":
    unittest.main()
