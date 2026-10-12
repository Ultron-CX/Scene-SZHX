import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "admin" / "spark" / "seal-x2-clock-freeze-evidence.py"
SPEC = importlib.util.spec_from_file_location("seal_x2_clock", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class SealClockEvidenceTests(unittest.TestCase):
    def test_digest_is_sha256(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "value"
            path.write_bytes(b"abc")
            self.assertEqual(
                MODULE.digest(path),
                "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
            )

    def test_atomic_json_sets_read_only_result(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "result.json"
            MODULE.atomic_json(path, {"real_robot_commands": False})
            self.assertFalse(path.stat().st_mode & 0o222)
            self.assertFalse(json.loads(path.read_text())["real_robot_commands"])


if __name__ == "__main__":
    unittest.main()
