import json
import os
import tempfile
import unittest
from pathlib import Path


PATCH_ROOT = Path(__file__).resolve().parents[1] / "third_party" / "genie_sim_engine_patch" / "scripts"
os.sys.path.insert(0, str(PATCH_ROOT))

from common.loop import clock_freeze_request, controlled_stop_requested  # noqa: E402


class ControlledStopTests(unittest.TestCase):
    def test_missing_path_is_not_a_request(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertFalse(controlled_stop_requested(str(Path(directory) / "missing.json")))

    def test_valid_request(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stop.json"
            path.write_text(
                json.dumps(
                    {
                        "schema": "sceneforge/physx-controlled-stop-request/v1",
                        "request": "STOP",
                        "real_robot_commands": False,
                    }
                ),
                encoding="utf-8",
            )
            self.assertTrue(controlled_stop_requested(str(path)))

    def test_request_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stop.json"
            path.write_text(
                json.dumps(
                    {
                        "schema": "sceneforge/physx-controlled-stop-request/v1",
                        "request": "STOP",
                        "real_robot_commands": True,
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "real_robot_commands=false"):
                controlled_stop_requested(str(path))

    def test_valid_clock_freeze_request_is_consumed_once(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "freeze.json"
            path.write_text(
                json.dumps(
                    {
                        "schema": "sceneforge/physx-clock-freeze-request/v1",
                        "request": "FREEZE",
                        "sequence": 3,
                        "wall_duration_s": 2.0,
                        "real_robot_commands": False,
                    }
                ),
                encoding="utf-8",
            )
            self.assertEqual(3, clock_freeze_request(str(path), 2)["sequence"])
            self.assertIsNone(clock_freeze_request(str(path), 3))

    def test_clock_freeze_is_bounded_and_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "freeze.json"
            payload = {
                "schema": "sceneforge/physx-clock-freeze-request/v1",
                "request": "FREEZE",
                "sequence": 1,
                "wall_duration_s": 11.0,
                "real_robot_commands": False,
            }
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "within"):
                clock_freeze_request(str(path), 0)
            payload["wall_duration_s"] = 2.0
            payload["real_robot_commands"] = True
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "real_robot_commands=false"):
                clock_freeze_request(str(path), 0)


if __name__ == "__main__":
    unittest.main()
