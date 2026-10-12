import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from sceneforge.x2_locomotion_qualification import (
    EPISODE_EVIDENCE_SCHEMA,
    REQUIRED_ARTIFACT_ROLES,
    atomic_write,
    build_suite,
)


def digest(label: str) -> str:
    return hashlib.sha256(label.encode("utf-8")).hexdigest()


def file_descriptor(path: Path, role: str, root: Path) -> dict:
    data = path.read_bytes()
    return {
        "role": role,
        "path": path.relative_to(root).as_posix(),
        "sha256": hashlib.sha256(data).hexdigest(),
        "size": len(data),
    }


def case_payload(scenario: str, *, direct_physx: bool = False) -> dict:
    reference = digest(f"mujoco-{scenario}")
    if not direct_physx:
        return {
            "schema": "sceneforge/x2-locomotion-ab/v2",
            "status": "PASSED",
            "qualification": False,
            "case_gate_passed": True,
            "functional_equivalence": False,
            "scenario": scenario,
            "inputs": {"mujoco": {"sha256": reference}},
            "real_robot_commands": False,
        }
    yaw = 0.25 if scenario == "turn_left" else -0.25 if scenario == "turn_right" else 0.0
    checks = {
        "completed_profile": True,
        "no_fall": True,
        "no_nan_or_inf": True,
        "zero_joint_limit_violations": True,
        "full_joint_state_and_target_within_limits": True,
        "sustained_stable_stop": True,
        "sim_time_freeze_probe": True,
        "one_command_per_physics_state": True,
        "bounded_command_execution_age_sim_time": True,
        "walk_policy_strict_sim_time": True,
    }
    return {
        "schema": "sceneforge/x2-geniesim-independent-motion/v2",
        "status": "PASSED",
        "qualification": False,
        "physics_authority": "genie_sim_isaac_physx",
        "real_robot_commands": False,
        "mujoco_reference_sha256": reference,
        "profile": {"command_yaw_rps": yaw},
        "checks": checks,
    }


def write_episode(
    root: Path,
    index: int,
    *,
    direct_physx: bool = False,
) -> Path:
    episode = root / f"episode-{index}"
    episode.mkdir()
    artifacts = []
    scenario_roles = {
        "straight": "straight_result",
        "turn_left": "turn_left_result",
        "turn_right": "turn_right_result",
    }
    for scenario, role in scenario_roles.items():
        path = episode / f"{scenario}-result.json"
        path.write_text(
            json.dumps(case_payload(scenario, direct_physx=direct_physx)),
            encoding="utf-8",
        )
        artifacts.append(file_descriptor(path, role, episode))
    for role in REQUIRED_ARTIFACT_ROLES:
        if role in scenario_roles.values():
            continue
        suffix = ".mp4" if role == "video" else ".json" if role.endswith("receipt") or role.endswith("manifest") or role == "controlled_stop_proof" else ".log"
        path = episode / f"{role}{suffix}"
        path.write_bytes(f"sealed-{role}-{index}\n".encode("utf-8"))
        artifacts.append(file_descriptor(path, role, episode))

    frozen = {
        name: digest(name)
        for name in (
            "profile_sha256",
            "thresholds_sha256",
            "controller_source_sha256",
            "stand_policy_sha256",
            "walk_policy_sha256",
            "asset_manifest_sha256",
            "joint_map_sha256",
            "dynamics_config_sha256",
        )
    }
    process = {
        "mode": "in_process",
        "pid": 1000 + index,
        "identity": f"container-{index}:engine-{index}:start-100",
        "started_at_unix": 1000.0 + index,
        "stopped_at_unix": 1100.0 + index,
        "exit_code": 0,
    }
    evidence = {
        "schema": EPISODE_EVIDENCE_SCHEMA,
        "episode_id": f"cold-start-{index}",
        "physics_authority": "genie_sim_isaac_physx",
        "published_command_topics": [],
        "real_robot_commands": False,
        "frozen_inputs": frozen,
        "safety": {
            "one_physics_authority": True,
            "no_fallback": True,
            "no_fall": True,
            "no_nan_or_inf": True,
            "full_joint_state_and_target_within_limits": True,
            "suspension_released_before_locomotion": True,
            "suspension_force_zero_during_locomotion": True,
            "controlled_stop_complete": True,
            "real_robot_commands": False,
            "root_pinned": False,
            "gravity_modified": False,
            "kinematic_teleportation_used": False,
            "fall_count": 0,
            "nonfinite_value_count": 0,
            "joint_limit_violation_count": 0,
            "nonfoot_contact_count": 0,
            "unexpected_fail_safe_count": 0,
            "joint_state_count": 31,
            "joint_target_count": 31,
        },
        "lifecycle": {
            "cold_start": True,
            "container_recreated": True,
            "started_cleanly": True,
            "controlled_stop_complete": True,
            "teardown_complete": True,
            "container_id": f"container-{index}",
            "container_image_digest": f"sha256:{digest('image')}",
            "physics_authority_process_count": 1,
            "residual_process_count": 0,
            "target_processes_before_start": 0,
            "target_processes_after_stop": 0,
            "container_exit_code": 0,
            "container_started_at_unix": 999.0 + index,
            "container_removed_at_unix": 1101.0 + index,
            "engine_process": dict(process),
            "controller_process": dict(process),
        },
        "reset": {
            "status": "PASSED",
            "reset_id": f"reset-{index}",
            "physics_reset_complete": True,
            "controller_history_reset": True,
            "cpg_state_reset": True,
            "command_sequence_reset": True,
            "initial_state_finite": True,
            "initial_pose_within_tolerance": True,
            "initial_state_sha256": digest(f"initial-state-{index}"),
        },
        "sim_time_freeze": {
            "status": "PASSED",
            "observed_wall_duration_s": 2.05,
            "sim_time_advance_s": 0.0,
            "clock_update_count": 0,
            "physics_state_update_count": 0,
            "controller_update_count": 0,
            "policy_update_count": 0,
            "command_update_count": 0,
            "cpg_phase_advance_rad": 0.0,
            "resume_completed": True,
        },
        "cases": {
            scenario: {
                "status": "PASSED",
                "gate_passed": True,
                "stable_stop_passed": True,
                "mujoco_alignment_passed": True,
                "mujoco_reference_sha256": digest(f"mujoco-{scenario}"),
                "artifact_role": role,
            }
            for scenario, role in scenario_roles.items()
        },
        "stable_stop": {
            "all_cases_sustained": True,
            "window_duration_s": 2.0,
            "maximum_linear_speed_mps": 0.02,
            "maximum_angular_speed_rps": 0.02,
            "linear_speed_p95_mps": 0.01,
            "angular_speed_p95_rps": 0.01,
            "maximum_planar_drift_m": 0.01,
        },
        "artifacts": artifacts,
    }
    result = {
        "schema": "sceneforge/x2-geniesim-cold-start-combination/v1",
        "status": "PASSED",
        "qualification": False,
        "real_robot_commands": False,
        "qualification_evidence": evidence,
    }
    path = episode / "result.json"
    path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return path


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def rewrite(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


class X2LocomotionQualificationTests(unittest.TestCase):
    def test_five_complete_immutable_attempts_qualify(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = [write_episode(root, index) for index in range(1, 6)]
            before = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in episodes}
            report = build_suite(episodes, suite_id="qualification-001")
            after = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in episodes}
        self.assertEqual(report["gate_status"], "PASSED")
        self.assertTrue(report["qualification"])
        self.assertTrue(report["functional_equivalence"])
        self.assertEqual(report["observed_attempt_count"], 5)
        self.assertEqual(report["findings"], [])
        self.assertEqual(before, after)

    def test_direct_physx_case_results_are_supported(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = [
                write_episode(root, index, direct_physx=True)
                for index in range(1, 6)
            ]
            report = build_suite(episodes, suite_id="qualification-physx")
        self.assertEqual(report["gate_status"], "PASSED")

    def test_four_attempts_are_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = [write_episode(root, index) for index in range(1, 5)]
            report = build_suite(episodes, suite_id="qualification-002")
        self.assertEqual(report["gate_status"], "INCOMPLETE")
        self.assertFalse(report["qualification"])
        self.assertIn("ATTEMPT_COUNT_INCOMPLETE", {x["code"] for x in report["findings"]})

    def test_missing_sealed_evidence_is_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = [write_episode(root, index) for index in range(1, 6)]
            payload = read(episodes[2])
            del payload["qualification_evidence"]["sim_time_freeze"]
            rewrite(episodes[2], payload)
            report = build_suite(episodes, suite_id="qualification-003")
        self.assertEqual(report["gate_status"], "INCOMPLETE")
        self.assertIn("MISSING_OBJECT", {x["code"] for x in report["findings"]})

    def test_explicit_safety_or_freeze_failure_fails_suite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = [write_episode(root, index) for index in range(1, 6)]
            payload = read(episodes[0])
            payload["qualification_evidence"]["safety"]["no_fall"] = False
            payload["qualification_evidence"]["sim_time_freeze"]["policy_update_count"] = 1
            rewrite(episodes[0], payload)
            report = build_suite(episodes, suite_id="qualification-004")
        self.assertEqual(report["gate_status"], "FAILED")
        codes = {x["code"] for x in report["findings"]}
        self.assertIn("GATE_FAILED", codes)
        self.assertIn("SIM_TIME_FREEZE_ADVANCED", codes)

    def test_real_robot_flag_fails_suite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = [write_episode(root, index) for index in range(1, 6)]
            payload = read(episodes[1])
            payload["real_robot_commands"] = True
            rewrite(episodes[1], payload)
            report = build_suite(episodes, suite_id="qualification-005")
        self.assertEqual(report["gate_status"], "FAILED")
        self.assertFalse(report["checks"]["real_robot_commands_false"])

    def test_duplicate_lifecycle_identity_fails_suite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = [write_episode(root, index) for index in range(1, 6)]
            first = read(episodes[0])["qualification_evidence"]["lifecycle"]
            payload = read(episodes[1])
            lifecycle = payload["qualification_evidence"]["lifecycle"]
            lifecycle["container_id"] = first["container_id"]
            lifecycle["engine_process"]["identity"] = first["engine_process"]["identity"]
            lifecycle["controller_process"]["identity"] = first["controller_process"]["identity"]
            rewrite(episodes[1], payload)
            report = build_suite(episodes, suite_id="qualification-006")
        self.assertEqual(report["gate_status"], "FAILED")
        codes = {x["code"] for x in report["findings"]}
        self.assertIn("DUPLICATE_CONTAINER_ID", codes)
        self.assertIn("DUPLICATE_ENGINE_PROCESS", codes)

    def test_artifact_hash_mismatch_fails_suite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = [write_episode(root, index) for index in range(1, 6)]
            (episodes[3].parent / "state_command_trace.log").write_text(
                "tampered\n", encoding="utf-8"
            )
            report = build_suite(episodes, suite_id="qualification-007")
        self.assertEqual(report["gate_status"], "FAILED")
        self.assertIn("ARTIFACT_HASH_MISMATCH", {x["code"] for x in report["findings"]})

    def test_missing_artifact_is_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = [write_episode(root, index) for index in range(1, 6)]
            (episodes[4].parent / "contact_trace.log").unlink()
            report = build_suite(episodes, suite_id="qualification-008")
        self.assertEqual(report["gate_status"], "INCOMPLETE")
        self.assertIn("ARTIFACT_FILE_MISSING", {x["code"] for x in report["findings"]})

    def test_changed_frozen_inputs_fail_suite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = [write_episode(root, index) for index in range(1, 6)]
            payload = read(episodes[4])
            payload["qualification_evidence"]["frozen_inputs"]["profile_sha256"] = digest("changed")
            rewrite(episodes[4], payload)
            report = build_suite(episodes, suite_id="qualification-009")
        self.assertEqual(report["gate_status"], "FAILED")
        self.assertIn("FROZEN_INPUT_MISMATCH", {x["code"] for x in report["findings"]})

    def test_nonfinite_json_fails_suite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            episodes = [write_episode(root, index) for index in range(1, 6)]
            text = episodes[0].read_text(encoding="utf-8")
            episodes[0].write_text(text[:-1] + ', "bad": NaN}', encoding="utf-8")
            report = build_suite(episodes, suite_id="qualification-010")
        self.assertEqual(report["gate_status"], "FAILED")
        self.assertIn("EPISODE_INVALID_JSON", {x["code"] for x in report["findings"]})

    def test_atomic_output_refuses_overwrite_and_nan(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "suite" / "result.json"
            with self.assertRaises(ValueError):
                atomic_write(output, {"bad": float("nan")})
            self.assertFalse(output.exists())
            atomic_write(output, {"ok": True})
            with self.assertRaises(FileExistsError):
                atomic_write(output, {"ok": True})


if __name__ == "__main__":
    unittest.main()
