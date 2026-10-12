import hashlib
import json
from pathlib import Path
import stat
import tempfile
import unittest

from sceneforge.x2_cold_start_finalization import finalize_combination
from sceneforge.x2_locomotion_qualification import REQUIRED_FROZEN_INPUT_HASHES


ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ("straight", "turn_left", "turn_right")
JOINTS = [f"joint_{index:02d}" for index in range(31)]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def digest(label: str) -> str:
    return hashlib.sha256(label.encode()).hexdigest()


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def profile(scenario: str) -> dict:
    yaw = 0.25 if scenario == "turn_left" else -0.25 if scenario == "turn_right" else 0.0
    return {
        "supported_stand_s": 3.0 if scenario == "straight" else 0.001,
        "band_ramp_s": 5.0 if scenario == "straight" else 0.001,
        "unsupported_stand_s": 3.0 if scenario == "straight" else 2.0,
        "walk_s": 8.0,
        "deceleration_s": 2.0,
        "cpg_stand_handoff_s": 1.2,
        "stop_s": 8.0,
        "command_x_mps": 0.3 if scenario == "straight" else 0.2,
        "command_y_mps": 0.0,
        "command_yaw_rps": yaw,
    }


def capture(scenario: str) -> dict:
    return {
        "status": "PASSED",
        "qualification": False,
        "physics_authority": "genie_sim_isaac_physx",
        "published_command_topics": [],
        "real_robot_commands": False,
        "root_pinned": False,
        "gravity_modified": False,
        "last_error": None,
        "joint_names": JOINTS,
        "profile": profile(scenario),
        "velocity_command_calibration": {
            "forward_scale": 0.735,
            "lateral_scale": 1.0,
            "yaw_scale": 1.0,
        },
        "checks": {
            "no_fall": True,
            "no_nan_or_inf": True,
            "zero_joint_limit_violations": True,
            "full_joint_state_and_target_within_limits": True,
            "band_released": True,
            "band_support_zero": True,
            "sustained_stable_stop": True,
        },
        "parameters": {
            "articulation_joint_count": 31,
            "joint_limit_source_sha256": digest("joint-limit-source"),
            "joint_position_limits_rad": {name: [-2.0, 2.0] for name in JOINTS},
            "joint_velocity_limits_rps": {name: 10.0 for name in JOINTS},
        },
        "model": {
            "stand_sha256": digest("stand-policy"),
            "walk_sha256": digest("walk-policy"),
        },
        "metrics": {
            "fall_event_count": 0,
            "nonfinite_sample_count": 0,
            "joint_limit_violation_count": 0,
            "position_target_limit_violation_count": 0,
            "stable_stop": {
                "observed_duration_s": 2.1,
                "window_s": 2.0,
                "sample_count": 1050,
                "linear_speed_p95_mps": 0.01,
                "angular_speed_p95_rps": 0.01,
                "maximum_linear_speed_mps": 0.02,
                "maximum_angular_speed_rps": 0.02,
                "planar_drift_m": 0.01,
            },
        },
    }


def make_runtime(root: Path) -> dict[str, Path]:
    root.mkdir()
    process = {
        "mode": "in_process",
        "identity": "container-1:engine-1:start-1",
        "pid": 101,
        "started_at_unix": 1000.0,
        "stopped_at_unix": 1100.0,
        "exit_code": 0,
    }
    write_json(root / "combination-runtime-result.json", {
        "status": "PASSED", "qualification": False,
        "physics_authority": "genie_sim_isaac_physx",
        "single_physics_lifecycle": True,
        "case_order": list(SCENARIOS), "published_command_topics": [],
        "real_robot_commands": False,
    })
    write_json(root / "lifecycle-receipt.json", {
        "cold_start": True, "container_recreated": True, "started_cleanly": True,
        "controlled_stop_complete": True, "teardown_complete": True,
        "container_id": "container-1", "container_image_digest": f"sha256:{digest('image')}",
        "physics_authority_process_count": 1, "residual_process_count": 0,
        "target_processes_before_start": 0, "target_processes_after_stop": 0,
        "container_exit_code": 0, "container_started_at_unix": 999.0,
        "container_removed_at_unix": 1101.0, "engine_process": process,
        "controller_process": process, "physics_authority": "genie_sim_isaac_physx",
        "published_command_topics": [], "real_robot_commands": False,
    })
    write_json(root / "reset-receipt.json", {
        "status": "PASSED", "reset_id": "reset-1",
        "physics_reset_complete": True, "controller_history_reset": True,
        "cpg_state_reset": True, "command_sequence_reset": True,
        "initial_state_finite": True, "initial_pose_within_tolerance": True,
        "initial_state_sha256": digest("initial-state"),
        "published_command_topics": [], "real_robot_commands": False,
    })
    write_json(root / "sim-time-freeze-receipt.json", {
        "status": "PASSED", "observed_wall_duration_s": 2.05,
        "sim_time_advance_s": 0.0, "clock_update_count": 0,
        "physics_state_update_count": 0, "controller_update_count": 0,
        "policy_update_count": 0, "command_update_count": 0,
        "cpg_phase_advance_rad": 0.0, "resume_completed": True,
        "qualification": False, "real_robot_commands": False,
    })
    write_json(root / "controlled-stop-proof.json", {
        "status": "PASSED", "request_present": True,
        "engine_logged_completion": True, "container_exit_code": 0,
        "real_robot_commands": False,
    })
    write_json(root / "runtime-manifest.json", {
        "engine_image": "x2:test", "engine_image_digest": f"sha256:{digest('image')}",
        "physics_hz": 500.0, "single_physics_lifecycle": True,
        "case_order": list(SCENARIOS), "real_robot_commands": False,
    })
    source_files = []
    for relative in (
        "src/sceneforge/x2_physx_gait1.py",
        "src/sceneforge/x2_cold_start_combination.py",
        "admin/spark/x2-geniesim-cold-start-combination-driver.py",
    ):
        path = ROOT / relative
        source_files.append({"path": relative, "size": path.stat().st_size, "sha256": sha(path)})
    write_json(root / "source-manifest.json", {
        "schema": "sceneforge/source-manifest/v1", "files": source_files,
        "real_robot_commands": False,
    })
    write_json(root / "asset-manifest.json", {
        "schema": "sceneforge/asset-manifest/v1",
        "files": [{"path": "asset.usd", "sha256": digest("asset"), "size": 1}],
        "real_robot_commands": False,
    })
    write_json(root / "controller-manifest.json", {
        "model": {"stand_sha256": digest("stand-policy"), "walk_sha256": digest("walk-policy")},
        "real_robot_commands": False,
    })
    write_json(root / "dynamics-alignment.json", {
        "status": "PASSED", "pre_world_reset": True,
        "physics_authority": "genie_sim_isaac_physx", "scope": "ground_friction_only",
        "ground_path": "/World/Ground", "material_path": "/World/PhysicsMaterials/X2",
        "static_friction": 0.8, "dynamic_friction": 0.7, "restitution": 0.0,
        "friction_combine_mode": "max", "foot_collision_mode": "imported_mesh",
        "foot_collision_geometry_changed": False, "authored_vendor_capsule_count": 0,
        "joint_or_controller_parameters_changed": False,
        "stage_root_layer_sha256": digest("stage"),
        "real_robot_commands": False,
    })
    (root / "engine.log").write_text("CONTROLLED_PHYSICS_STOP_COMPLETE\n", encoding="utf-8")
    (root / "controller.log").write_text("X2 complete\n", encoding="utf-8")

    mujoco = {}
    combined_physics = []
    combined_base = []
    combined_commands = []
    combined_contacts = []
    time_windows = {"straight": (1.0, 1.5), "turn_left": (3.0, 3.5), "turn_right": (5.0, 5.5)}
    for scenario in SCENARIOS:
        case_dir = root / "cases" / scenario
        case_capture = capture(scenario)
        write_json(case_dir / "physx-result.json", {
            "status": "PASSED", "qualification": False,
            "published_command_topics": [], "real_robot_commands": False,
            "capture": case_capture,
        })
        start, end = time_windows[scenario]
        physics = [
            {"physics_sequence": 1, "sim_time": start, "phase": "GAIT1_FORWARD", "position": [0.0] * 31},
            {"physics_sequence": 2, "sim_time": end, "phase": "GAIT1_FORWARD", "position": [0.01] * 31},
        ]
        base = [
            {"physics_sequence": 1, "sim_time": start, "phase": "GAIT1_FORWARD", "cpg_phase_rad": 0.0},
            {"physics_sequence": 2, "sim_time": end, "phase": "GAIT1_FORWARD", "cpg_phase_rad": 1.0},
        ]
        commands = [
            {"sim_time": start, "articulation_position_target": [0.0] * 31},
            {"sim_time": end, "articulation_position_target": [0.01] * 31},
        ]
        contacts = [
            {"sim_time": start, "maximum_non_foot_contact_force_n": 0.0},
            {"sim_time": end, "maximum_non_foot_contact_force_n": 0.0},
        ]
        write_json(case_dir / "state-command-trace.json", {
            "joint_names": JOINTS, "physics_samples": physics, "base_samples": base,
            "commands": commands, "contact_samples": contacts,
            "physics_authority": "genie_sim_isaac_physx", "real_robot_commands": False,
        })
        for rows, target in ((physics, combined_physics), (base, combined_base), (commands, combined_commands), (contacts, combined_contacts)):
            for row in rows:
                target.append({**row, "scenario": scenario})

        m_dir = root.parent / f"mujoco-{scenario}"
        m_dir.mkdir()
        m_result = m_dir / "result.json"
        write_json(m_result, {
            "status": "PASSED", "qualification": False,
            "physics_authority": "google_mujoco_3_3_5",
            "published_command_topics": [], "real_robot_commands": False,
        })
        write_json(m_dir / "trace.json", {
            "physics_authority": "google_mujoco_3_3_5", "samples": [],
            "real_robot_commands": False,
        })
        mujoco[scenario] = m_result

    write_json(root / "state-command-trace.json", {
        "joint_names": JOINTS, "physics_samples": combined_physics,
        "base_samples": combined_base, "commands": combined_commands,
        "physics_authority": "genie_sim_isaac_physx", "real_robot_commands": False,
    })
    write_json(root / "contact-trace.json", {
        "samples": combined_contacts, "physics_authority": "genie_sim_isaac_physx",
        "real_robot_commands": False,
    })
    rows = []
    for value in (0.9, 1.2, 2.9, 3.2, 4.9, 5.2):
        rows.append({
            "sim_time_s": value, "state": "RELEASED", "support_ratio": 0.0,
            "force_norm_n": 0.0, "real_robot_commands": False,
        })
    (root / "elastic-band-telemetry.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
    )
    return mujoco


def fake_comparator(failed: str | None = None):
    def build(mujoco: Path, physx: Path) -> dict:
        payload = json.loads(physx.read_text(encoding="utf-8"))
        scenario_profile = payload["capture"]["profile"]
        yaw = scenario_profile["command_yaw_rps"]
        scenario = "turn_left" if yaw > 0 else "turn_right" if yaw < 0 else "straight"
        passed = scenario != failed
        return {
            "schema": "sceneforge/x2-locomotion-ab/v2",
            "status": "PASSED" if passed else "FAILED",
            "gate_status": "PASSED" if passed else "FAILED",
            "case_gate_passed": passed,
            "qualification": False,
            "scenario": scenario,
            "checks": {
                "physx_stable_stop_p95": True,
                "physx_stable_stop_maximum": True,
                "physx_stable_stop_drift": True,
            },
            "phase_aligned_leg_joint_comparison": {"available": True},
            "inputs": {
                "mujoco": {"path": str(mujoco), "sha256": sha(mujoco)},
                "physx": {"path": str(physx), "sha256": sha(physx)},
            },
            "published_command_topics": [],
            "real_robot_commands": False,
        }
    return build


def make_video(root: Path, run: Path) -> tuple[Path, Path]:
    video = root / "video.mp4"
    video.write_bytes(b"not-a-real-codec-but-nonempty-test-video")
    receipt = root / "video-receipt.json"
    write_json(receipt, {
        "status": "SUCCEEDED", "output_sha256": sha(video),
        "source_runtime_sha256": sha(run / "combination-runtime-result.json"),
        "qualification": False, "real_robot_commands": False,
    })
    return video, receipt


def freeze_runtime_for_derivations(run: Path) -> dict[str, str]:
    derived = run / "derived"
    derived.mkdir(mode=0o755)
    index_name = "runtime-frozen-index.json"
    rows = []
    raw_hashes = {}
    for path in sorted(run.rglob("*"), key=lambda item: item.relative_to(run).as_posix()):
        relative = path.relative_to(run)
        if relative.parts and relative.parts[0] == "derived":
            continue
        if path.is_file() and relative.as_posix() != index_name:
            digest_value = sha(path)
            raw_hashes[relative.as_posix()] = digest_value
            rows.append({
                "path": relative.as_posix(),
                "size": path.stat().st_size,
                "sha256": digest_value,
            })
    write_json(run / index_name, {
        "schema": "sceneforge/x2-frozen-runtime-index/v1",
        "derivation_root": "derived",
        "file_count": len(rows),
        "files": rows,
        "qualification": False,
        "real_robot_commands": False,
    })
    for path in run.rglob("*"):
        if path.is_file() and "derived" not in path.relative_to(run).parts:
            path.chmod(0o444)
    for path in sorted(
        (item for item in run.rglob("*") if item.is_dir() and item != derived),
        key=lambda item: len(item.parts),
        reverse=True,
    ):
        path.chmod(0o555)
    run.chmod(0o555)
    derived.chmod(0o755)
    return raw_hashes


def make_tree_writable(root: Path) -> None:
    if not root.exists():
        return
    root.chmod(0o755)
    for path in root.rglob("*"):
        try:
            path.chmod(0o755 if path.is_dir() else 0o644)
        except OSError:
            pass


class X2ColdStartFinalizationTests(unittest.TestCase):
    def test_v2_uses_authoritative_case_trace_order_when_result_omits_names(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "run"
            mujoco = make_runtime(run)
            for scenario in SCENARIOS:
                result_path = run / f"cases/{scenario}/physx-result.json"
                payload = json.loads(result_path.read_text())
                del payload["capture"]["joint_names"]
                write_json(result_path, payload)
            receipt = finalize_combination(
                run_dir=run,
                mujoco_results=mujoco,
                repo_root=ROOT,
                episode_evidence_version=2,
                comparator=fake_comparator(),
            )
            self.assertEqual("PASSED", receipt["status"])
            manifest = json.loads(
                (run / "qualification-finalization/manifests/joint-map.json").read_text()
            )
            self.assertEqual(JOINTS, manifest["joint_names"])

    def test_v2_no_video_is_sealable_and_defers_media_to_suite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "run"
            mujoco = make_runtime(run)
            receipt = finalize_combination(
                run_dir=run,
                mujoco_results=mujoco,
                repo_root=ROOT,
                episode_evidence_version=2,
                comparator=fake_comparator(),
            )
            self.assertEqual("PASSED", receipt["status"])
            self.assertTrue(receipt["ready_for_episode_sealing"])
            self.assertFalse(receipt["episode_media_required"])
            final_dir = run / "qualification-finalization"
            spec = json.loads((final_dir / "sealing-input.json").read_text())
            self.assertEqual(
                "sceneforge/x2-locomotion-episode-sealing-input/v2",
                spec["schema"],
            )
            self.assertNotIn("video", spec["artifacts"])
            self.assertNotIn("video_receipt", spec["artifacts"])
            thresholds = json.loads(
                (final_dir / "manifests/thresholds.json").read_text()
            )
            self.assertEqual(
                "sceneforge/x2-locomotion-qualification-suite/v2",
                thresholds["qualification_suite_schema"],
            )
            self.assertFalse((final_dir / "sealing-input.pending.json").exists())
            for scenario in SCENARIOS:
                self.assertTrue(
                    (
                        final_dir
                        / f"references/physx/{scenario}/state-command-trace.json"
                    ).is_file()
                )

    def test_frozen_runtime_uses_reserved_derivation_area_without_raw_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "run"
            mujoco = make_runtime(run)
            video, video_receipt = make_video(root, run)
            raw_hashes = freeze_runtime_for_derivations(run)
            try:
                receipt = finalize_combination(
                    run_dir=run,
                    mujoco_results=mujoco,
                    repo_root=ROOT,
                    video=video,
                    video_receipt=video_receipt,
                    comparator=fake_comparator(),
                )
                final_dir = run / "derived/qualification-finalization"
                self.assertEqual(receipt["status"], "PASSED")
                self.assertEqual(receipt["output_directory"], "derived/qualification-finalization")
                self.assertEqual(receipt["frozen_runtime"]["file_count"], len(raw_hashes))
                self.assertTrue((final_dir / "sealing-input.json").is_file())
                self.assertEqual(
                    final_dir.stat().st_mode & (stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH),
                    0,
                )
                self.assertNotEqual((run / "derived").stat().st_mode & stat.S_IWUSR, 0)
                for relative, expected in raw_hashes.items():
                    self.assertEqual(sha(run / relative), expected)
                spec = json.loads((final_dir / "sealing-input.json").read_text())
                self.assertEqual(
                    spec["frozen_inputs"]["runtime_frozen_index_sha256"],
                    sha(run / "runtime-frozen-index.json"),
                )
                self.assertEqual(
                    spec["artifacts"]["runtime_frozen_index"],
                    "runtime-frozen-index.json",
                )
            finally:
                make_tree_writable(run)

    def test_frozen_runtime_hash_change_fails_closed_in_derived_area(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "run"
            mujoco = make_runtime(run)
            freeze_runtime_for_derivations(run)
            engine_log = run / "engine.log"
            engine_log.chmod(0o644)
            altered = bytearray(engine_log.read_bytes())
            altered[0] ^= 1
            engine_log.write_bytes(altered)
            engine_log.chmod(0o444)
            try:
                receipt = finalize_combination(
                    run_dir=run,
                    mujoco_results=mujoco,
                    repo_root=ROOT,
                    comparator=fake_comparator(),
                )
                self.assertEqual(receipt["status"], "FAILED")
                self.assertIn(
                    "FROZEN_RUNTIME_HASH_MISMATCH",
                    {item["code"] for item in receipt["findings"]},
                )
                self.assertTrue(
                    (run / "derived/qualification-finalization/finalization-receipt.json").is_file()
                )
            finally:
                make_tree_writable(run)

    def test_complete_postprocessing_is_sealable_but_not_qualified(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "run"
            mujoco = make_runtime(run)
            video, video_receipt = make_video(root, run)
            receipt = finalize_combination(
                run_dir=run, mujoco_results=mujoco, repo_root=ROOT,
                episode_id="cold-start-1", video=video,
                video_receipt=video_receipt, comparator=fake_comparator(),
            )
            self.assertEqual(receipt["status"], "PASSED")
            self.assertTrue(receipt["ready_for_episode_sealing"])
            self.assertFalse(receipt["qualification"])
            spec = json.loads((run / "qualification-finalization/sealing-input.json").read_text())
            self.assertFalse(spec["qualification"])
            self.assertFalse(spec["real_robot_commands"])
            self.assertEqual(set(spec["frozen_inputs"]), set(REQUIRED_FROZEN_INPUT_HASHES))
            self.assertTrue(all(len(value) == 64 for value in spec["frozen_inputs"].values()))
            self.assertEqual(spec["stable_stop"]["window_duration_s"], 2.0)
            self.assertEqual(spec["stable_stop"]["minimum_observed_span_s"], 2.1)
            self.assertEqual(spec["stable_stop"]["minimum_sample_count"], 1050)
            self.assertEqual(spec["safety"]["nonfoot_contact_count"], 0)
            for scenario in SCENARIOS:
                report = json.loads((run / f"qualification-finalization/cases/{scenario}/result.json").read_text())
                self.assertTrue(report["case_gate_passed"])

    def test_missing_video_is_incomplete_and_only_writes_pending_input(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "run"
            mujoco = make_runtime(run)
            receipt = finalize_combination(
                run_dir=run, mujoco_results=mujoco, repo_root=ROOT,
                comparator=fake_comparator(),
            )
            self.assertEqual(receipt["status"], "INCOMPLETE")
            self.assertFalse(receipt["ready_for_episode_sealing"])
            self.assertTrue((run / "qualification-finalization/sealing-input.pending.json").is_file())
            self.assertFalse((run / "qualification-finalization/sealing-input.json").exists())
            codes = {item["code"] for item in receipt["findings"]}
            self.assertTrue({"VIDEO_MISSING", "VIDEO_RECEIPT_MISSING"}.issubset(codes))
            self.assertIn("ARTIFACT_ROLE_MISSING", codes)

    def test_explicit_ab_failure_is_failed_without_sealing_input(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "run"
            mujoco = make_runtime(run)
            receipt = finalize_combination(
                run_dir=run, mujoco_results=mujoco, repo_root=ROOT,
                comparator=fake_comparator("turn_left"),
            )
            self.assertEqual(receipt["status"], "FAILED")
            self.assertFalse((run / "qualification-finalization/sealing-input.json").exists())
            self.assertIn("AB_CASE_GATE_FAILED", {item["code"] for item in receipt["findings"]})

    def test_video_receipt_must_bind_this_runtime(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "run"
            mujoco = make_runtime(run)
            video, video_receipt = make_video(root, run)
            payload = json.loads(video_receipt.read_text())
            payload["source_runtime_sha256"] = digest("other-run")
            write_json(video_receipt, payload)
            receipt = finalize_combination(
                run_dir=run, mujoco_results=mujoco, repo_root=ROOT,
                video=video, video_receipt=video_receipt,
                comparator=fake_comparator(),
            )
            self.assertEqual(receipt["status"], "FAILED")
            self.assertIn("VIDEO_SOURCE_HASH_MISMATCH", {item["code"] for item in receipt["findings"]})

    def test_native_replay_receipt_chain_can_bind_authoritative_trace(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "run"
            mujoco = make_runtime(run)
            video = root / "video.mp4"
            video.write_bytes(b"video")
            render_trace = root / "render-state.jsonl"
            render_trace.write_text('{"sample":1}\n', encoding="utf-8")
            render_receipt = root / "render-state-receipt.json"
            write_json(render_receipt, {
                "status": "SUCCEEDED", "output_sha256": sha(render_trace),
                "source_trace_sha256": sha(run / "state-command-trace.json"),
                "real_robot_commands": False,
            })
            replay_receipt = root / "replay-receipt.json"
            write_json(replay_receipt, {
                "status": "SUCCEEDED", "output_sha256": sha(video),
                "source_trace": {"path": str(render_trace), "sha256": sha(render_trace)},
                "source_trace_receipt": {"path": str(render_receipt), "sha256": sha(render_receipt)},
                "qualification": False, "real_robot_commands": False,
            })
            receipt = finalize_combination(
                run_dir=run, mujoco_results=mujoco, repo_root=ROOT,
                video=video, video_receipt=replay_receipt,
                comparator=fake_comparator(),
            )
            self.assertEqual(receipt["status"], "PASSED")
            spec = json.loads((run / "qualification-finalization/sealing-input.json").read_text())
            self.assertIn("video_source_trace", spec["artifacts"])
            self.assertIn("video_source_trace_receipt", spec["artifacts"])

    def test_missing_explicit_safety_count_is_incomplete_not_inferred_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "run"
            mujoco = make_runtime(run)
            result_path = run / "cases/turn_right/physx-result.json"
            payload = json.loads(result_path.read_text())
            del payload["capture"]["metrics"]["nonfinite_sample_count"]
            write_json(result_path, payload)
            receipt = finalize_combination(
                run_dir=run, mujoco_results=mujoco, repo_root=ROOT,
                comparator=fake_comparator(),
            )
            self.assertEqual(receipt["status"], "INCOMPLETE")
            self.assertIn("SAFETY_COUNT_MISSING", {item["code"] for item in receipt["findings"]})
            self.assertFalse(receipt["ready_for_episode_sealing"])

    def test_finalization_directory_is_no_clobber(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root / "run"
            mujoco = make_runtime(run)
            finalize_combination(
                run_dir=run, mujoco_results=mujoco, repo_root=ROOT,
                comparator=fake_comparator(),
            )
            with self.assertRaises(FileExistsError):
                finalize_combination(
                    run_dir=run, mujoco_results=mujoco, repo_root=ROOT,
                    comparator=fake_comparator(),
                )


if __name__ == "__main__":
    unittest.main()
