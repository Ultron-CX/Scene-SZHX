import json
import unittest
from pathlib import Path

from sceneforge.canonical import canonical_json_bytes, fingerprint


ROOT = Path(__file__).resolve().parents[1]


class CanonicalTests(unittest.TestCase):
    def test_key_order_does_not_change_fingerprint(self) -> None:
        self.assertEqual(fingerprint({"a": 1, "b": 2}), fingerprint({"b": 2, "a": 1}))

    def test_example_round_trip_is_canonical(self) -> None:
        data = json.loads((ROOT / "examples/hospital/building-seed-1101.json").read_text())
        encoded = canonical_json_bytes(data)
        self.assertEqual(json.loads(encoded), data)


if __name__ == "__main__":
    unittest.main()
