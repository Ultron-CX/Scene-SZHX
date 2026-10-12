import json
import math
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


PATCH_ROOT = Path(__file__).resolve().parents[1] / "third_party" / "genie_sim_engine_patch" / "scripts"
os.sys.path.insert(0, str(PATCH_ROOT))

from kit.elastic_band import ElasticBandConfig, ElasticBandController  # noqa: E402


class ElasticBandTests(unittest.TestCase):
    def make_controller(self, root: Path, state: str = "ENGAGED") -> ElasticBandController:
        command = root / "command.json"
        command.write_text(
            json.dumps(
                {
                    "schema": "sceneforge/physx-elastic-band-command/v1",
                    "sequence": 1,
                    "state": state,
                    "support_ratio": 1.0,
                }
            ),
            encoding="utf-8",
        )
        return ElasticBandController(
            ElasticBandConfig(
                enabled=True,
                body_name="torso_link",
                anchor=(0.0, 0.0, 2.5),
                stiffness_n_per_m=200.0,
                damping_n_s_per_m=100.0,
                rest_length_m=0.0,
                max_force_n=800.0,
                command_file=command,
                status_file=root / "status.json",
                telemetry_file=root / "telemetry.jsonl",
            )
        )

    def test_vendor_parameter_force_points_toward_anchor(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.make_controller(Path(directory))
            force = controller.step((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.002)
            self.assertEqual(force[0:2], (0.0, 0.0))
            self.assertAlmostEqual(force[2], 300.0)

    def test_damping_opposes_motion_toward_anchor(self):
        with tempfile.TemporaryDirectory() as directory:
            controller = self.make_controller(Path(directory))
            still = controller.step((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.002)
            moving = controller.step((0.0, 0.0, 1.0), (0.0, 0.0, 1.0), 0.002)
            self.assertLess(moving[2], still[2])

    def test_ramp_down_reaches_exact_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            controller = self.make_controller(root)
            controller.step((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.1)
            (root / "command.json").write_text(
                json.dumps(
                    {
                        "schema": "sceneforge/physx-elastic-band-command/v1",
                        "sequence": 2,
                        "state": "RAMP_DOWN",
                        "ramp_duration_s": 1.0,
                    }
                ),
                encoding="utf-8",
            )
            for _ in range(11):
                force = controller.step((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.1)
            self.assertEqual(controller.state, "RELEASED")
            self.assertEqual(controller.support_ratio, 0.0)
            self.assertEqual(force, (0.0, 0.0, 0.0))
            status = json.loads((root / "status.json").read_text(encoding="utf-8"))
            self.assertTrue(status["force_is_exactly_zero"])
            self.assertFalse(status["root_pinned"])
            self.assertFalse(status["gravity_modified"])
            self.assertFalse(status["real_robot_commands"])
            telemetry = [json.loads(line) for line in (root / "telemetry.jsonl").read_text().splitlines()]
            self.assertGreater(len(telemetry), 2)
            self.assertEqual(telemetry[-1]["force_norm_n"], 0.0)
            self.assertEqual(telemetry[-1]["record_type"], "sample")

    def test_command_sequence_must_increase(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            controller = self.make_controller(root)
            (root / "command.json").write_text(
                json.dumps(
                    {
                        "schema": "sceneforge/physx-elastic-band-command/v1",
                        "sequence": 1,
                        "state": "RELEASED",
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "not increasing"):
                controller.step((0.0, 0.0, 1.0), (0.0, 0.0, 0.0), 0.1)

    def test_environment_is_fail_closed(self):
        with patch.dict(
            os.environ,
            {
                "ROBOEASE_PHYSX_ELASTIC_BAND_ENABLED": "true",
                "ROBOEASE_PHYSX_ELASTIC_BAND_COMMAND_FILE": "relative.json",
                "ROBOEASE_PHYSX_ELASTIC_BAND_STATUS_FILE": "status.json",
                "ROBOEASE_PHYSX_ELASTIC_BAND_TELEMETRY_FILE": "telemetry.jsonl",
            },
            clear=True,
        ):
            with self.assertRaisesRegex(ValueError, "absolute command-file"):
                ElasticBandConfig.from_environment()


if __name__ == "__main__":
    unittest.main()
