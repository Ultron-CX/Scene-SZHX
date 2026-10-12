import copy
import hashlib
import json
import os
from pathlib import Path
import stat
import tempfile
import unittest

from sceneforge.x2_locomotion_qualification import (
    DIRECT_PHYSX_CASE_CHECKS,
    EPISODE_EVIDENCE_SCHEMA_V2,
    REQUIRED_ARTIFACT_ROLES,
    SUITE_SCHEMA_V2,
    REQUIRED_FROZEN_INPUT_HASHES,
    build_suite,
    sha256,
)
from sceneforge.x2_qualification_sealing import (
    EpisodeSealingError,
    SEALING_INPUT_SCHEMA,
    SEALING_INPUT_SCHEMA_V2,
    build_episode_payload,
    seal_episode,
)


def digest(label: str) -> str:
    return hashlib.sha256(label.encode("utf-8")).hexdigest()


def case_payload(scenario: str) -> dict:
    yaw = 0.25 if scenario == "turn_left" else -0.25 if scenario == "turn_right" else 0.0
    return {
        "schema": "sceneforge/x2-geniesim-independent-motion/v2",
        "status": "PASSED",
        "qualification": False,
        "physics_authority": "genie_sim_isaac_physx",
        "real_robot_commands": False,
        "mujoco_reference_sha256": digest(f"mujoco-{scenario}"),
        "scenario": scenario,
        "profile": {"command_yaw_rps": yaw},
        "checks": {name: True for name in DIRECT_PHYSX_CASE_CHECKS},
    }


def make_spec(root: Path, index: int) -> dict:
    artifacts: dict[str, str] = {}
    scenario_roles = {
        "straight": "straight_result",
        "turn_left": "turn_left_result",
        "turn_right": "turn_right_result",
    }
    for scenario, role in scenario_roles.items():
        relative = Path("cases") / scenario / "result.json"
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(case_payload(scenario)), encoding="utf-8")
        artifacts[role] = relative.as_posix()
    for role in REQUIRED_ARTIFACT_ROLES:
        if role in scenario_roles.values():
            continue
        relative = Path("artifacts") / f"{role}.dat"
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(f"episode-{index}:{role}\n".encode("utf-8"))
        artifacts[role] = relative.as_posix()

    process = {
        "mode": "in_process",
        "pid": 2000 + index,
        "identity": f"container-{index}:engine-{index}:start-100",
        "started_at_unix": 1000.0 + index,
        "stopped_at_unix": 1100.0 + index,
        "exit_code": 0,
    }
    return {
        "schema": SEALING_INPUT_SCHEMA,
        "episode_id": f"cold-start-{index}",
        "physics_authority": "genie_sim_isaac_physx",
        "published_command_topics": [],
        "real_robot_commands": False,
        "frozen_inputs": {
            name: digest(name) for name in REQUIRED_FROZEN_INPUT_HASHES
        },
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


class X2QualificationSealingTests(unittest.TestCase):
    def test_v2_seals_episode_without_per_episode_media(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = make_spec(root, 1)
            spec["schema"] = SEALING_INPUT_SCHEMA_V2
            for role in ("video", "video_receipt"):
                relative = spec["artifacts"].pop(role)
                (root / relative).unlink()
            receipt = seal_episode(episode_root=root, sealing_input=spec)
            output = Path(receipt["output"])
            payload = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(
                EPISODE_EVIDENCE_SCHEMA_V2,
                payload["qualification_evidence"]["schema"],
            )
            roles = {
                row["role"] for row in payload["qualification_evidence"]["artifacts"]
            }
            self.assertFalse({"video", "video_receipt"} & roles)
            report = build_suite(
                [output],
                suite_id="v2-single-preflight",
                suite_schema=SUITE_SCHEMA_V2,
            )
            self.assertEqual("PASSED", report["attempts"][0]["gate_status"])
            self.assertEqual("INCOMPLETE", report["gate_status"])

    def test_frozen_episode_seals_inside_reserved_derived_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = make_spec(root, 1)
            raw_hashes = {
                path.relative_to(root).as_posix(): sha256(path)
                for path in root.rglob("*")
                if path.is_file()
            }
            derived = root / "derived"
            derived.mkdir(mode=0o755)
            for path in root.rglob("*"):
                if path.is_file() and "derived" not in path.relative_to(root).parts:
                    path.chmod(0o444)
            for path in sorted(
                (item for item in root.rglob("*") if item.is_dir() and item != derived),
                key=lambda item: len(item.parts),
                reverse=True,
            ):
                path.chmod(0o555)
            root.chmod(0o555)
            derived.chmod(0o755)
            try:
                receipt = seal_episode(episode_root=root, sealing_input=spec)
                output = derived / "result.json"
                self.assertTrue(receipt["sealed"])
                self.assertEqual(Path(receipt["output"]), output)
                payload = json.loads(output.read_text())
                self.assertEqual(
                    payload["qualification_evidence"]["artifact_root"], ".."
                )
                self.assertEqual(
                    build_suite([output], suite_id="derived-seal-test")["gate_status"],
                    "INCOMPLETE",
                )
                # One valid episode is expected to fail only the five-attempt
                # suite count, while its individual attempt remains sealed.
                attempt = build_suite(
                    [output], suite_id="derived-seal-attempt-test"
                )["attempts"][0]
                self.assertEqual(attempt["gate_status"], "PASSED")
                for relative, expected in raw_hashes.items():
                    self.assertEqual(sha256(root / relative), expected)
                self.assertEqual(
                    output.stat().st_mode & (stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH),
                    0,
                )
            finally:
                root.chmod(0o755)
                for path in root.rglob("*"):
                    try:
                        path.chmod(0o755 if path.is_dir() else 0o644)
                    except OSError:
                        pass

    def test_artifact_root_marker_is_rejected_outside_reserved_derived_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = make_spec(root, 1)
            receipt = seal_episode(episode_root=root, sealing_input=spec)
            output = Path(receipt["output"])
            output.chmod(0o644)
            payload = json.loads(output.read_text())
            payload["qualification_evidence"]["artifact_root"] = ".."
            output.write_text(json.dumps(payload), encoding="utf-8")
            attempt = build_suite(
                [output], suite_id="invalid-artifact-root-test"
            )["attempts"][0]
            self.assertEqual(attempt["gate_status"], "FAILED")
            self.assertIn(
                "ARTIFACT_ROOT_INVALID",
                {item["code"] for item in attempt["findings"]},
            )

    def test_five_sealed_episodes_are_accepted_by_suite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            outputs = []
            for index in range(1, 6):
                episode = root / f"episode-{index}"
                episode.mkdir()
                receipt = seal_episode(
                    episode_root=episode,
                    sealing_input=make_spec(episode, index),
                )
                self.assertTrue(receipt["sealed"])
                output = episode / "result.json"
                outputs.append(output)
                self.assertEqual(output.stat().st_mode & stat.S_IWUSR, 0)
            suite = build_suite(outputs, suite_id="sealed-cold-starts")
            self.assertEqual(suite["gate_status"], "PASSED")
            self.assertTrue(suite["qualification"])
            for output in outputs:
                os.chmod(output, 0o600)

    def test_builder_does_not_mutate_input_and_sorts_artifacts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = make_spec(root, 1)
            before = copy.deepcopy(spec)
            payload, output = build_episode_payload(
                episode_root=root,
                sealing_input=spec,
            )
        self.assertEqual(spec, before)
        self.assertEqual(output.name, "result.json")
        roles = [
            item["role"]
            for item in payload["qualification_evidence"]["artifacts"]
        ]
        self.assertEqual(roles, sorted(roles))
        self.assertFalse(payload["qualification"])
        self.assertFalse(payload["real_robot_commands"])

    def test_missing_artifact_role_is_incomplete_and_leaves_no_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = make_spec(root, 1)
            del spec["artifacts"]["contact_trace"]
            with self.assertRaises(EpisodeSealingError) as caught:
                seal_episode(episode_root=root, sealing_input=spec)
            self.assertEqual(caught.exception.gate_status, "INCOMPLETE")
            self.assertFalse((root / "result.json").exists())

    def test_explicit_safety_failure_is_failed_and_leaves_no_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = make_spec(root, 1)
            spec["safety"]["no_fall"] = False
            with self.assertRaises(EpisodeSealingError) as caught:
                seal_episode(episode_root=root, sealing_input=spec)
            self.assertEqual(caught.exception.gate_status, "FAILED")
            self.assertIn(
                "GATE_FAILED",
                {item["code"] for item in caught.exception.findings},
            )
            self.assertFalse((root / "result.json").exists())

    def test_path_escape_is_failed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = make_spec(root, 1)
            spec["artifacts"]["engine_log"] = "../engine.log"
            with self.assertRaises(EpisodeSealingError) as caught:
                seal_episode(episode_root=root, sealing_input=spec)
            self.assertEqual(caught.exception.gate_status, "FAILED")
            self.assertEqual(caught.exception.findings[0]["code"], "ARTIFACT_PATH_ESCAPE")

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "result.json"
            output.write_text("original\n", encoding="utf-8")
            spec = make_spec(root, 1)
            with self.assertRaises(EpisodeSealingError) as caught:
                seal_episode(episode_root=root, sealing_input=spec)
            self.assertEqual(caught.exception.gate_status, "FAILED")
            self.assertEqual(output.read_text(encoding="utf-8"), "original\n")

    def test_case_reference_mismatch_fails_preflight(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = make_spec(root, 1)
            spec["cases"]["turn_left"]["mujoco_reference_sha256"] = digest(
                "wrong-reference"
            )
            with self.assertRaises(EpisodeSealingError) as caught:
                seal_episode(episode_root=root, sealing_input=spec)
            self.assertEqual(caught.exception.gate_status, "FAILED")
            self.assertIn(
                "MUJOCO_REFERENCE_MISMATCH",
                {item["code"] for item in caught.exception.findings},
            )
            self.assertFalse((root / "result.json").exists())

    def test_nonfinite_fact_fails_without_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            spec = make_spec(root, 1)
            spec["stable_stop"]["maximum_linear_speed_mps"] = float("nan")
            with self.assertRaises(EpisodeSealingError) as caught:
                seal_episode(episode_root=root, sealing_input=spec)
            self.assertEqual(caught.exception.gate_status, "FAILED")
            self.assertEqual(
                caught.exception.findings[0]["code"],
                "SEALING_INPUT_NOT_STRICT_JSON",
            )
            self.assertFalse((root / "result.json").exists())


if __name__ == "__main__":
    unittest.main()
