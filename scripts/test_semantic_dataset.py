"""Synthetic dataset contract tests; no live project writes."""
import copy
import io
import json
import tempfile
import unittest
import zipfile
from pathlib import Path
from project_store import ProjectStore, CATEGORIES
from test_project_store import SCENE, TMP, payload, observation
from semantic_dataset import export_dataset, read_dataset, encode, sha


class DatasetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=TMP)
        self.store = ProjectStore(Path(self.temp.name), SCENE)
        p = payload(count=3)
        p['objects'] = p['objects'][:2]
        self.eid = self.store.archive(observation())
        p['objects'][0]['evidence_ids'] = [self.eid]
        self.store.save(p)

    def tearDown(self):
        self.temp.cleanup()

    def package(self):
        out = io.BytesIO()
        export_dataset(self.store, 1, out, CATEGORIES)
        return out.getvalue()

    def mutate(self, name, change):
        with zipfile.ZipFile(io.BytesIO(self.package())) as z:
            files = {n: z.read(n) for n in z.namelist()}
        v = json.loads(files[name]); change(v); files[name] = encode(v)
        m = json.loads(files['manifest.json']); m['files'][name] = dict(bytes=len(files[name]), sha256=sha(files[name])); files['manifest.json'] = encode(m)
        out = io.BytesIO()
        with zipfile.ZipFile(out, 'w') as z:
            for n, raw in files.items(): z.writestr(n, raw)
        return io.BytesIO(out.getvalue())

    def test_roundtrip_without_renderer_and_no_mutation(self):
        before = copy.deepcopy(self.store.load())
        result = read_dataset(io.BytesIO(self.package()))
        self.assertEqual(result['regions'], before['document']['annotations']['boxes'])
        self.assertEqual(result['objects'], before['document']['objects'])
        self.assertEqual(result['quality']['unregistered_region_ids'], ['region-2'])
        self.assertEqual(result['quality']['state_counts'], {'unreviewed': 2})
        self.assertIsNone(result['quality']['accuracy'])
        self.assertEqual(result['evidence'][0]['status'], 'verified')
        self.assertEqual(self.store.load(), before)

    def test_missing_or_corrupt_blob_is_explicit(self):
        with unittest.mock.patch.object(self.store, 'blob', side_effect=FileNotFoundError('synthetic')):
            out = io.BytesIO(); export_dataset(self.store, 1, out, CATEGORIES)
        result = read_dataset(io.BytesIO(out.getvalue()))
        self.assertEqual(result['evidence'][0]['status'], 'unavailable_or_invalid')
        self.assertEqual(result['evidence'][0]['files'], [])

    def test_associated_candidate_exports_all_regions_without_identity_promotion(self):
        before = self.store.load()
        objects = copy.deepcopy(before['document']['objects'])
        objects[0]['region_ids'] += objects[1]['region_ids']
        objects = objects[:1]
        saved = self.store.save(dict(expected_revision=1, annotations=before['document']['annotations'],
            objects=objects, reason='synthetic_association'))
        output = io.BytesIO()
        export_dataset(self.store, saved['head'], output, CATEGORIES)
        result = read_dataset(io.BytesIO(output.getvalue()))
        self.assertEqual(result['regions'], before['document']['annotations']['boxes'])
        self.assertEqual(result['objects'], saved['document']['objects'])
        self.assertEqual(result['objects'][0]['region_ids'], ['region-0', 'region-1'])
        self.assertEqual(result['objects'][0]['identity_state'], 'candidate_not_human_confirmed')
        self.assertEqual(result['quality']['unregistered_region_ids'], ['region-2'])
        self.assertEqual(result['quality']['state_counts'], {'unreviewed': 1})
        self.assertEqual(self.store.load(1)['document'], before['document'])

    def test_quality_forgery_rejected_even_with_rehashed_file(self):
        stream = self.mutate('quality.json', lambda v: v.update(accuracy=1))
        with self.assertRaisesRegex(ValueError, 'Quality'): read_dataset(stream)

    def test_scene_evidence_mismatch_rejected(self):
        stream = self.mutate('scene.json', lambda v: v.update(coordinate_frame='WRONG'))
        with self.assertRaisesRegex(ValueError, 'Evidence scene'): read_dataset(stream)

    def test_missing_reference_inventory_rejected(self):
        stream = self.mutate('evidence.json', lambda v: v.clear())
        with self.assertRaisesRegex(ValueError, 'reference'): read_dataset(stream)

    def test_unknown_category_rejected(self):
        stream = self.mutate('regions.json', lambda v: v[0].update(category='fake-bank-role'))
        with self.assertRaisesRegex(ValueError, 'category'): read_dataset(stream)

    def test_hash_and_unexpected_path_rejected(self):
        raw = self.package()
        for name, content in [('regions.json', b'[]'), ('../escape', b'x')]:
            out = io.BytesIO()
            with zipfile.ZipFile(io.BytesIO(raw)) as src, zipfile.ZipFile(out, 'w') as dst:
                for n in src.namelist(): dst.writestr(n, content if n == name else src.read(n))
                if name not in src.namelist(): dst.writestr(name, content)
            with self.assertRaises(ValueError): read_dataset(io.BytesIO(out.getvalue()))

    def test_stale_and_missing_links_do_not_inherit_pass(self):
        from semantic_dataset import quality
        doc = self.store.load()['document']; o = doc['objects'][0]
        o['reviews'] = [dict(basis_sha256='wrong', checks={d:'pass' for d in ('category','identity','coverage','background','evidence','coordinate')})]
        q = quality(doc['annotations']['boxes'], doc['objects'])
        self.assertEqual(q['objects'][0]['status'], 'stale')
        self.assertEqual(set(q['objects'][0]['checks'].values()), {'pending'})
        o['region_ids'] = ['missing']
        self.assertEqual(quality(doc['annotations']['boxes'], doc['objects'])['objects'][0]['missing_region_ids'], ['missing'])


if __name__ == '__main__': unittest.main()
