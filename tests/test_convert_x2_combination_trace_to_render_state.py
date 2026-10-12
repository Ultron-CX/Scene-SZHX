import importlib.util
import hashlib
import json
import os
import stat
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = (
    Path(__file__).parents[1]
    / "admin"
    / "spark"
    / "convert-x2-combination-trace-to-render-state.py"
)
SPEC = importlib.util.spec_from_file_location(
    "convert_x2_combination_trace_to_render_state", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def freeze_run(run: Path) -> None:
    rows = []
    for path in sorted(run.rglob("*"), key=lambda item: item.relative_to(run).as_posix()):
        relative = path.relative_to(run).as_posix()
        if path.is_file() and relative != "runtime-frozen-index.json":
            rows.append({
                "path": relative,
                "size": path.stat().st_size,
                "sha256": sha(path),
            })
    write_json(run / "runtime-frozen-index.json", {
        "schema": MODULE.FROZEN_INDEX_SCHEMA,
        "derivation_root": "derived",
        "file_count": len(rows),
        "files": rows,
        "qualification": False,
        "real_robot_commands": False,
    })


def make_urdf(path: Path) -> list[str]:
    names = [f"joint_{index:02d}" for index in range(MODULE.JOINT_COUNT)]
    rows = ["<robot name='x2'>", "  <link name='base_link'/>"]
    parent = "base_link"
    for index, name in enumerate(names):
        child = f"link_{index:02d}"
        rows.extend((
            f"  <link name='{child}'/>",
            f"  <joint name='{name}' type='revolute'>",
            f"    <parent link='{parent}'/>",
            f"    <child link='{child}'/>",
            "    <origin xyz='0 0 0.01' rpy='0 0 0'/>",
            "    <axis xyz='0 1 0'/>",
            "    <limit lower='-1' upper='1' effort='10' velocity='10'/>",
            "  </joint>",
        ))
        parent = child
    rows.append("</robot>")
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    return names


def make_run(root: Path, *, scenario_order=None, unsafe_band=False) -> tuple[Path, Path, Path]:
    run = root / "run"
    run.mkdir()
    urdf = root / "x2.urdf"
    names = make_urdf(urdf)
    manifest = root / "manifest.json"
    write_json(manifest, {
        "robot_root_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "theta": 0.0}
    })
    scenarios = scenario_order or list(MODULE.SCENARIOS)
    write_json(run / "combination-runtime-result.json", {
        "schema": "sceneforge/x2-physx-cold-start-combination-runtime/v1",
        "status": "PASSED",
        "qualification": False,
        "physics_authority": MODULE.PHYSICS_AUTHORITY,
        "single_physics_lifecycle": True,
        "case_order": list(MODULE.SCENARIOS),
        "completed_case_count": 3,
        "cases": [
            {"scenario": item, "status": "PASSED"} for item in MODULE.SCENARIOS
        ],
        "real_robot_commands": False,
    })
    write_json(run / "lifecycle-receipt.json", {
        "schema": "sceneforge/x2-physx-container-lifecycle-receipt/v1",
        "physics_authority_process_count": 1,
        "container_exit_code": 0,
        "teardown_complete": True,
        "engine_process": {
            "started_at_unix": 100.0,
            "stopped_at_unix": 102.0,
        },
        "real_robot_commands": False,
    })
    write_json(run / "runtime-manifest.json", {
        "schema": "sceneforge/x2-runtime-manifest/v1",
        "physics_hz": 500.0,
        "single_physics_lifecycle": True,
        "case_order": list(MODULE.SCENARIOS),
        "real_robot_commands": False,
    })

    physics, bases, commands, contacts, band = [], [], [], [], []
    sim_time = 0.0
    for scenario in scenarios:
        requested = {
            "straight": [0.3, 0.0, 0.0],
            "turn_left": [0.2, 0.0, 0.25],
            "turn_right": [0.2, 0.0, -0.25],
        }[scenario]
        for phase in ("GAIT1_FORWARD", "STOP_STAND"):
            physics.append({
                "scenario": scenario,
                "sim_time": sim_time,
                "position": [0.01] * MODULE.JOINT_COUNT,
                "velocity": [0.0] * MODULE.JOINT_COUNT,
            })
            bases.append({
                "scenario": scenario,
                "sim_time": sim_time,
                "phase": phase,
                "position_m": [sim_time, 0.0, 0.65],
                "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                "base_linear_velocity_body_mps": requested[:2] + [0.0],
                "base_angular_velocity_body_rps": [0.0, 0.0, requested[2]],
                "linear_speed_mps": abs(requested[0]) if phase == "GAIT1_FORWARD" else 0.0,
                "angular_speed_rad_s": abs(requested[2]) if phase == "GAIT1_FORWARD" else 0.0,
                "cpg_phase_rad": sim_time,
                "cpg_period_s": 0.7,
                "active_gait": "gait1" if phase == "GAIT1_FORWARD" else "stand",
                "requested_velocity": requested if phase == "GAIT1_FORWARD" else [0.0, 0.0, 0.0],
                "effective_velocity_command": requested if phase == "GAIT1_FORWARD" else [0.0, 0.0, 0.0],
            })
            commands.append({
                "scenario": scenario,
                "sim_time": sim_time,
                "articulation_position_target": [0.01] * MODULE.JOINT_COUNT,
            })
            contacts.append({
                "scenario": scenario,
                "sim_time": sim_time,
                "feet": {
                    "left_ankle_roll_link": {"contact": phase == "STOP_STAND"},
                    "right_ankle_roll_link": {"contact": True},
                },
            })
            band.append({
                "schema": "sceneforge/physx-elastic-band-status/v1",
                "record_type": "sample",
                "sim_time_s": sim_time,
                "state": "RELEASED",
                "support_ratio": 0.0,
                "force_norm_n": 0.0,
                "real_robot_commands": unsafe_band,
            })
            sim_time += 0.4
    write_json(run / "state-command-trace.json", {
        "schema": "sceneforge/x2-physx-cold-start-state-command-trace/v1",
        "joint_names": names,
        "physics_samples": physics,
        "base_samples": bases,
        "commands": commands,
        "physics_authority": MODULE.PHYSICS_AUTHORITY,
        "real_robot_commands": False,
    })
    write_json(run / "contact-trace.json", {
        "schema": "sceneforge/x2-physx-cold-start-contact-trace/v1",
        "samples": contacts,
        "physics_authority": MODULE.PHYSICS_AUTHORITY,
        "real_robot_commands": False,
    })
    (run / "elastic-band-telemetry.jsonl").write_text(
        "".join(json.dumps(row, allow_nan=False) + "\n" for row in band),
        encoding="utf-8",
    )
    freeze_run(run)
    return run, urdf, manifest


class X2CombinationRenderStateTests(unittest.TestCase):
    def test_builds_exact_30fps_overlay_trace_and_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run, urdf, manifest = make_run(root)
            output, receipt_path = root / "render.jsonl", root / "receipt.json"
            receipt = MODULE.build_render_state(
                run_dir=run,
                urdf=urdf,
                manifest_path=manifest,
                output=output,
                receipt_path=receipt_path,
                fps=30.0,
            )
            self.assertEqual("SUCCEEDED", receipt["status"])
            self.assertEqual(list(MODULE.SCENARIOS), receipt["case_order"])
            self.assertEqual(MODULE.OVERLAY_SCHEMA, receipt["overlay_schema"])
            self.assertEqual(30.0, receipt["target_sample_rate_hz"])
            self.assertEqual(receipt["sample_count"], receipt["overlay_sample_count"])
            self.assertEqual(MODULE.sha256(output), receipt["output_sha256"])
            self.assertEqual(
                MODULE.sha256(run / "runtime-frozen-index.json"),
                receipt["source_frozen_index_sha256"],
            )
            self.assertFalse(receipt["real_robot_commands"])
            self.assertEqual("global_sim_span_divided_by_engine_process_wall_lifetime", receipt["rtf"]["method"])
            records = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
            self.assertEqual(MODULE.OVERLAY_SCHEMA, records[0]["overlay_schema"])
            self.assertEqual("sample", records[1]["record_type"])
            self.assertEqual(MODULE.OVERLAY_SCHEMA, records[1]["overlay"]["schema"])
            self.assertIn("left_foot_contact", records[1]["overlay"])
            self.assertIn("cpg_phase_rad", records[1]["overlay"])
            self.assertEqual(receipt["sample_count"], records[-1]["overlay_sample_count"])
            if os.name != "nt":
                self.assertEqual(0, output.stat().st_mode & stat.S_IWUSR)

    def test_rejects_wrong_case_order(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run, urdf, manifest = make_run(
                root, scenario_order=["turn_left", "straight", "turn_right"]
            )
            with self.assertRaisesRegex(ValueError, "contiguous"):
                MODULE.build_render_state(
                    run_dir=run, urdf=urdf, manifest_path=manifest,
                    output=root / "out.jsonl", receipt_path=root / "receipt.json",
                )

    def test_rejects_non_30fps_or_unsafe_band_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run, urdf, manifest = make_run(root, unsafe_band=True)
            with self.assertRaisesRegex(ValueError, "elastic-band.*real_robot_commands"):
                MODULE.build_render_state(
                    run_dir=run, urdf=urdf, manifest_path=manifest,
                    output=root / "out.jsonl", receipt_path=root / "receipt.json",
                )
            with self.assertRaisesRegex(ValueError, "exactly 30 fps"):
                MODULE.build_render_state(
                    run_dir=run, urdf=urdf, manifest_path=manifest,
                    output=root / "other.jsonl", receipt_path=root / "other.json",
                    fps=29.97,
                )

    def test_rejects_runtime_modified_after_frozen_index(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run, urdf, manifest = make_run(root)
            with (run / "state-command-trace.json").open("a", encoding="utf-8") as stream:
                stream.write(" ")
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                MODULE.build_render_state(
                    run_dir=run, urdf=urdf, manifest_path=manifest,
                    output=root / "out.jsonl", receipt_path=root / "receipt.json",
                )

    def test_source_has_no_remote_or_command_transport(self):
        source = MODULE_PATH.read_text(encoding="utf-8").lower()
        for forbidden in ("ssh ", "/cmd_vel", "/joint_command", "203.0.113."):
            self.assertNotIn(forbidden, source)
        self.assertIn('"published_topics": []', source)
        self.assertIn('"real_robot_commands": false', source)


if __name__ == "__main__":
    unittest.main()
