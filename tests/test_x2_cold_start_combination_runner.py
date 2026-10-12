from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "admin/spark/x2-geniesim-cold-start-combination-driver.py"
RUNNER = ROOT / "admin/spark/run-x2-geniesim-cold-start-combination.sh"


class X2ColdStartCombinationRunnerTests(unittest.TestCase):
    def test_driver_uses_one_engine_session_and_one_run_loop(self):
        source = DRIVER.read_text(encoding="utf-8")
        self.assertEqual(source.count("session = EngineSession("), 1)
        self.assertEqual(source.count("session.run("), 1)
        self.assertEqual(source.count("session.startup("), 1)
        self.assertIn("X2ColdStartCombinationCoordinator", source)
        self.assertIn("single-lifecycle X2 straight+turn_left+turn_right", source)

    def test_driver_is_physx_only_and_has_no_command_transport(self):
        source = DRIVER.read_text(encoding="utf-8")
        self.assertIn('physics_engine != "isaac_physx"', source)
        self.assertIn('"physics_authority": "genie_sim_isaac_physx"', source)
        self.assertIn('"published_command_topics": []', source)
        self.assertIn('"real_robot_commands": False', source)
        for forbidden in (
            "rclpy",
            "create_publisher",
            "/cmd_vel",
            "/joint_command",
            "ssh ",
        ):
            self.assertNotIn(forbidden, source)

    def test_driver_records_reset_and_exact_case_order(self):
        source = DRIVER.read_text(encoding="utf-8")
        self.assertIn("reset-receipt.json", source)
        self.assertIn("physics_reset_complete", source)
        self.assertIn("controller_history_reset", source)
        self.assertIn("cpg_state_reset", source)
        self.assertIn("command_sequence_reset", source)
        self.assertIn("initial_state_sha256", source)
        self.assertIn("zip(SCENARIOS, case_profiles)", source)

    def test_shell_creates_one_container_and_never_kills_a_successful_run(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertEqual(source.count("container_id=$(docker create"), 1)
        self.assertEqual(source.count('docker start "$container"'), 1)
        self.assertNotIn('docker kill "$container"', source)
        self.assertIn("authority_process_count", source)
        self.assertIn("target_processes_before_start", source)
        self.assertIn("target_processes_after_stop", source)
        self.assertIn("container_image_digest", source)

    def test_shell_marks_runtime_bundle_nonqualified_and_offline(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertIn("REAL_ROBOT_COMMANDS=false", source)
        self.assertIn('"qualification":False', source)
        self.assertIn("produce_case_ab_reports_and_offline_video_then_seal_episode", source)
        self.assertIn("single_physics_lifecycle", source)
        self.assertIn("sim-time-freeze-receipt.json", source)
        self.assertNotIn("docker pull", source)
        self.assertNotIn("ssh ", source)

    def test_shell_passes_the_same_scene_joint_pose_that_reset_verifies(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertIn("init_joint_pos_json=$(python3", source)
        self.assertIn("len(joints) != 31", source)
        self.assertIn("math.degrees(float(value))", source)
        self.assertIn("-p init_joint_pos_json:='$init_joint_pos_json'", source)
        self.assertNotIn("-p init_joint_pos_json:='{}'", source)

    def test_shell_counts_only_the_authoritative_isaac_python_process(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertIn("/isaac-sim\\/kit\\/python\\/bin\\/python3", source)
        self.assertIn("container_init_pid=$(docker inspect", source)
        self.assertIn('"launcher_process": {', source)

    def test_driver_makes_container_artifacts_host_readable(self):
        source = DRIVER.read_text(encoding="utf-8")
        self.assertIn("def make_run_artifacts_host_readable", source)
        self.assertIn("path.chmod(0o644)", source)
        self.assertIn("make_run_artifacts_host_readable(Path(run_root))", source)

    def test_shell_seals_uid_1234_artifacts_with_a_scoped_nonphysics_container(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertIn("docker run --rm --user 0:0 --entrypoint bash", source)
        self.assertIn('-v "$run_dir:/evidence-run"', source)
        self.assertIn(
            "find /evidence-run -path /evidence-run/derived -prune -o -type f",
            source,
        )
        self.assertIn("-exec chown 0:0 {} + -exec chmod 0444 {} +", source)
        self.assertIn(
            "find /evidence-run -path /evidence-run/derived -prune -o -type d",
            source,
        )
        self.assertIn("-exec chown 0:0 {} + -exec chmod 0555 {} +", source)
        self.assertIn("chmod 0755 /evidence-run/derived", source)
        self.assertNotIn('find "$run_dir" -type f -exec chmod a-w', source)

    def test_shell_reserves_derivations_and_hash_binds_all_raw_files(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertIn('mkdir -m 0755 "$run_dir/derived"', source)
        self.assertIn('index_name = "runtime-frozen-index.json"', source)
        self.assertIn('"schema": "sceneforge/x2-frozen-runtime-index/v1"', source)
        self.assertIn('"derivation_root": "derived"', source)
        self.assertIn('"real_robot_commands": False', source)
        self.assertIn('relative.parts[0] == "derived"', source)
        self.assertIn('"sha256": digest(path)', source)


if __name__ == "__main__":
    unittest.main()
