"""Synthetic sample preparation checks; no real scene or live store."""
import json
import tempfile
import unittest
from pathlib import Path
from prepare_r1_sample import ROOT, prepare


class SampleTests(unittest.TestCase):
    def test_empty_unique_and_source_unchanged(self):
        base = ROOT / '.tmp' / 'r1-sample-tests'
        base.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=base) as directory:
            parent = Path(directory)
            source = parent / 'scene.json'
            raw = json.dumps(dict(scene_id='synthetic-r1', source_hashes={'test': 'a'*64},
                                 coordinate_frame='TEST', T_lcc_from_las=[])).encode()
            source.write_bytes(raw)
            first, second = prepare(source, parent), prepare(source, parent)
            self.assertNotEqual(first, second)
            manifest = json.loads((first / 'manifest.json').read_text())
            self.assertEqual(manifest['initial_regions'], 0)
            self.assertFalse(manifest['activated'])
            self.assertFalse(manifest['browser_isolation_verified'])
            self.assertEqual(source.read_bytes(), raw)
            with self.assertRaises(ValueError):
                prepare(source, ROOT / 'data')


if __name__ == '__main__':
    unittest.main()
