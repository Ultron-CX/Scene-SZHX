import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).parents[1] / "admin" / "spark" / "replay-x2-geniesim-render-state.py"
WRAPPER_PATH = Path(__file__).parents[1] / "admin" / "spark" / "render-x2-geniesim-state-replay.sh"
SPEC = importlib.util.spec_from_file_location("replay_x2_render_state", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def transform(x):
    return {
        "parent_path": "map",
        "child_path": "/x2/Geometry/base_link",
        "translation_xyz_m": [x, 0.0, 0.6],
        "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def overlay(*, sequence: int, stamp: int):
    return {
        "schema": MODULE.OVERLAY_SCHEMA,
        "scenario": "straight",
        "phase": "GAIT1_FORWARD",
        "active_gait": "gait1",
        "requested_velocity": [0.3, 0.0, 0.0],
        "effective_velocity_command": [0.3, 0.0, 0.0],
        "measured_body_linear_velocity": [0.28, 0.01, 0.0],
        "measured_body_angular_velocity": [0.0, 0.0, 0.01],
        "linear_speed_mps": 0.2802,
        "angular_speed_rps": 0.01,
        "left_foot_contact": sequence % 2 == 0,
        "right_foot_contact": sequence % 2 != 0,
        "cpg_phase_rad": float(sequence),
        "cpg_period_s": 0.7,
        "suspension_state": "RELEASED",
        "suspension_support_ratio": 0.0,
        "suspension_force_norm_n": 0.0,
        "sim_time_s": stamp / 1e9,
        "rtf_estimate": 0.4,
        "rtf_method": "global_lifecycle",
        "real_robot_commands": False,
    }


def write_trace(directory: Path, *, unsafe=False, with_overlay=False):
    directory.mkdir(parents=True, exist_ok=True)
    trace = directory / "state.jsonl"
    receipt = directory / "receipt.json"
    records = [{
        "record_type": "schema",
        "schema": MODULE.TRACE_SCHEMA,
        "authoritative_physics_source": MODULE.PHYSICS_SOURCE,
        "read_only": True,
        "published_topics": [],
        "real_robot_commands": unsafe,
    }]
    if with_overlay:
        records[0]["overlay_schema"] = MODULE.OVERLAY_SCHEMA
    for sequence, stamp in enumerate((1_000_000_000, 2_000_000_000)):
        record = {
            "record_type": "sample",
            "schema": MODULE.TRACE_SCHEMA,
            "sequence": sequence,
            "sim_time_ns": stamp,
            "base_position_xyz_m": [float(sequence), 0.0, 0.0],
            "base_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "transforms": [transform(float(sequence))],
        }
        if with_overlay:
            record["overlay"] = overlay(sequence=sequence, stamp=stamp)
        records.append(record)
    records.append({
        "record_type": "summary",
        "schema": MODULE.TRACE_SCHEMA,
        "status": "SUCCEEDED",
        "sample_count": 2,
        "real_robot_commands": False,
    })
    trace.write_text("".join(json.dumps(record) + "\n" for record in records), encoding="utf-8")
    receipt_payload = {
        "schema": MODULE.TRACE_RECEIPT_SCHEMA,
        "status": "SUCCEEDED",
        "authoritative_physics_source": MODULE.PHYSICS_SOURCE,
        "read_only": True,
        "published_topics": [],
        "real_robot_commands": False,
        "sample_count": 2,
        "output_sha256": MODULE.sha256_file(trace),
    }
    if with_overlay:
        receipt_payload["overlay_schema"] = MODULE.OVERLAY_SCHEMA
        receipt_payload["overlay_sample_count"] = 2
    receipt.write_text(json.dumps(receipt_payload), encoding="utf-8")
    return trace, receipt


class X2RenderStateReplayTests(unittest.TestCase):
    def test_loads_authoritative_read_only_trace(self):
        with tempfile.TemporaryDirectory() as directory:
            trace_path, receipt_path = write_trace(Path(directory))
            trace = MODULE.load_trace(trace_path, receipt_path)
            self.assertEqual(("/x2/Geometry/base_link",), trace.topology)
            self.assertEqual(2, len(trace.samples))

    def test_rejects_trace_that_claims_real_robot_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            trace_path, receipt_path = write_trace(Path(directory), unsafe=True)
            with self.assertRaisesRegex(ValueError, "real_robot_commands"):
                MODULE.load_trace(trace_path, receipt_path)

    def test_resampling_interpolates_recorded_local_transform(self):
        with tempfile.TemporaryDirectory() as directory:
            trace_path, receipt_path = write_trace(Path(directory))
            trace = MODULE.load_trace(trace_path, receipt_path)
            samples = MODULE.resample_trace(trace.samples, 2.0, None, None)
            self.assertEqual(3, len(samples))
            self.assertAlmostEqual(0.5, samples[1].transforms[0].translation_xyz_m[0])
            self.assertEqual("/x2/Geometry/base_link", samples[1].transforms[0].child_path)

    def test_overlay_is_required_fail_closed_and_survives_resampling(self):
        with tempfile.TemporaryDirectory() as directory:
            trace_path, receipt_path = write_trace(
                Path(directory), with_overlay=True
            )
            trace = MODULE.load_trace(
                trace_path, receipt_path, require_overlay=True
            )
            samples = MODULE.resample_trace(trace.samples, 2.0, None, None)
            self.assertEqual(MODULE.OVERLAY_SCHEMA, samples[1].overlay["schema"])
            self.assertAlmostEqual(1.5, samples[1].overlay["sim_time_s"])
            self.assertEqual("straight", samples[1].overlay["scenario"])

            plain_trace, plain_receipt = write_trace(
                Path(directory) / "plain"
            )
            with self.assertRaisesRegex(ValueError, "overlay"):
                MODULE.load_trace(
                    plain_trace, plain_receipt, require_overlay=True
                )

    def test_hud_is_burned_into_top_right_frame(self):
        width, height = 1280, 720
        source = np.zeros((height, width, 3), dtype=np.uint8).tobytes()
        rendered = MODULE.render_overlay_rgb(
            source,
            width,
            height,
            overlay(sequence=0, stamp=1_000_000_000),
        )
        self.assertEqual(len(source), len(rendered))
        pixels = np.frombuffer(rendered, dtype=np.uint8).reshape(height, width, 3)
        self.assertEqual(0, int(pixels[:, : width // 3].max()))
        self.assertGreater(int(pixels[: height // 2, width // 2 :].max()), 0)

    def test_rejects_incomplete_overlay(self):
        value = overlay(sequence=0, stamp=1_000_000_000)
        del value["left_foot_contact"]
        with self.assertRaisesRegex(ValueError, "left_foot_contact"):
            MODULE._load_overlay(value, 2)

    def test_source_contains_no_robot_command_publishers(self):
        source = MODULE_PATH.read_text(encoding="utf-8").lower()
        for forbidden in ("/cmd_vel", "/joint_command", "/stand_mode", "203.0.113."):
            self.assertNotIn(forbidden, source)
        self.assertIn('"published_robot_command_topics": []', source)
        self.assertIn('"real_robot_commands": false', source)
        self.assertIn('"opencv_ffmpeg_mp4v"', source)
        self.assertIn('"discarded_warmup_frame_count"', source)

    def test_wrapper_launches_renderer_but_not_physics(self):
        source = WRAPPER_PATH.read_text(encoding="utf-8")
        self.assertIn("genie_sim_render_node", source)
        self.assertNotIn("genie_sim_engine_node", source)
        self.assertNotIn("run-x2-geniesim-fixed-controller-gate.sh", source)
        self.assertIn("replay_physics_authority", source)
        self.assertIn('REAL_ROBOT_COMMANDS:-false', source)
        self.assertIn('domain=$((80 + 10#${stamp:9:4} % 100))', source)
        self.assertIn('/opt/ros/jazzy/lib/aarch64-linux-gnu', source)
        self.assertIn("docker exec", source)
        self.assertIn(
            "LD_LIBRARY_PATH=/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib:/isaac-sim",
            source,
        )

    def test_combination_wrapper_is_state_only_h264_hud_replay(self):
        wrapper = (
            Path(__file__).parents[1]
            / "admin"
            / "spark"
            / "render-x2-geniesim-combination-replay.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("convert-x2-combination-trace-to-render-state.py", wrapper)
        self.assertIn("genie_sim_render_node", wrapper)
        self.assertNotIn("genie_sim_engine_node", wrapper)
        self.assertIn("--require-overlay", wrapper)
        self.assertIn('"encoder_backend": "opencv_ffmpeg_mp4v"', wrapper)
        self.assertIn('"left_menu_visible": False', wrapper)
        self.assertIn('"real_robot_commands": False', wrapper)
        self.assertIn('"visual_full_body_verification": "pending_user_review"', wrapper)
        self.assertIn('runs/x2-geniesim-cold-combination-', wrapper)
        self.assertIn('evidence/x2-geniesim-cold-combination-', wrapper)
        self.assertIn('third_party/ovrtx-python-0.3/ovrtx/bin/cache', wrapper)
        self.assertIn('ovrtx/bin/cache:rw', wrapper)
        self.assertIn('.sceneforge-ovrtx-render.lock', wrapper)
        self.assertIn('.sceneforge-write-probe-', wrapper)
        self.assertIn('--warmup-timeout-s 300', wrapper)
        self.assertIn('--frame-timeout-s 15', wrapper)
        self.assertIn('roboease/x2-video-recorder-humble-arm64:20260818', wrapper)
        self.assertIn('-c:v libx264', wrapper)
        self.assertIn('-pix_fmt yuv420p', wrapper)
        self.assertIn('-vsync vfr', wrapper)
        self.assertIn('ffprobe -v error -count_frames', wrapper)
        self.assertIn('video-probe.json', wrapper)
        self.assertIn('chown -R 0:0 /evidence', wrapper)
        self.assertIn('find /evidence -type f -exec chmod 0444', wrapper)
        self.assertIn("qa-first.png", wrapper)
        self.assertIn("qa-middle.png", wrapper)
        self.assertIn("qa-last.png", wrapper)
        self.assertIn('"qa_frames": qa_frames', wrapper)
        for forbidden in ("/cmd_vel", "/joint_command", "203.0.113."):
            self.assertNotIn(forbidden, wrapper)


if __name__ == "__main__":
    unittest.main()
