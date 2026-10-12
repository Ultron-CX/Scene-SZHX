"""D4 isolated logical-zone persistence, ZIP and independent dataset tests."""
import copy
import io
import json
import tempfile
import unittest
import uuid
import zipfile
from pathlib import Path
from project_store import ProjectStore, CATEGORIES
from test_project_store import SCENE, TMP, payload
from bundle_import import BundleImporter
from semantic_dataset import validate_spatial, spatial_quality, export_dataset, read_dataset, encode, sha


def fixture(oid):
    floor, room = ['zone-' + str(uuid.uuid4()) for _ in range(2)]
    def zone(id, name, kind, parent):
        return dict(zone_id=id, name=name, kind=kind, parent_id=parent, source='manual', verification='unverified', notes='Synthetic, not bank truth')
    return dict(schema='scenesem.spatial-context.v1', zones=[zone(floor, 'Synthetic floor', 'floor', None), zone(room, 'Synthetic room', 'room', floor)],
        memberships=[dict(object_id=oid, zone_id=room, source='manual', verification='unverified', notes='Synthetic only')])


class SpatialTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=TMP); self.root = Path(self.temp.name)
        self.store = ProjectStore(self.root / 'source', SCENE)
        self.p = payload(count=2); self.s = fixture(self.p['objects'][0]['object_id'])
        self.p['spatial_context'] = self.s

    def tearDown(self):
        self.temp.cleanup()

    def test_contract_rejects_cycles_bad_references_and_truth_promotion(self):
        validate_spatial(self.s)
        mutations = [lambda s: s['zones'][0].update(parent_id=s['zones'][1]['zone_id']),
            lambda s: s['zones'][0].update(verification='verified'),
            lambda s: s['memberships'][0].update(zone_id='missing'),
            lambda s: s['memberships'].append(s['memberships'][0]), lambda s: s.update(extra=True)]
        for change in mutations:
            value = copy.deepcopy(self.s); change(value)
            with self.assertRaises(ValueError): validate_spatial(value)

    def test_save_legacy_client_guard_and_explicit_restore(self):
        saved = self.store.save(self.p)
        self.assertEqual(saved['document']['spatial_context'], self.s)
        old = copy.deepcopy(self.p); del old['spatial_context']; old['expected_revision'] = 1
        with self.assertRaises(ValueError): self.store.save(old)
        self.assertEqual(self.store.load()['head'], 1)
        old['spatial_context'] = None
        restored = self.store.save(old)
        self.assertNotIn('spatial_context', restored['document'])
        self.assertEqual(self.store.load(1)['document'], saved['document'])

    def test_missing_objects_are_reported_not_reassigned(self):
        self.assertEqual(spatial_quality(self.s, [])['missing_object_ids'], [self.p['objects'][0]['object_id']])
        self.p['objects'] = []; self.store.save(self.p)
        out = io.BytesIO(); export_dataset(self.store, 1, out, CATEGORIES)
        read = read_dataset(io.BytesIO(out.getvalue()))
        self.assertEqual(read['spatial'], self.s)
        self.assertEqual(len(read['quality']['spatial']['missing_object_ids']), 1)

    def test_v2_export_v1_compatibility_and_tamper_rejection(self):
        self.store.save(self.p)
        out = io.BytesIO(); export_dataset(self.store, 1, out, CATEGORIES)
        result = read_dataset(io.BytesIO(out.getvalue()))
        self.assertEqual(result['manifest']['schema'], 'scenesem.semantic-dataset.v2')
        self.assertEqual(result['spatial'], self.s)
        self.assertEqual(result['regions'], self.p['annotations']['boxes'])
        self.assertEqual(result['quality']['spatial']['unassigned_object_ids'], [self.p['objects'][1]['object_id']])
        with zipfile.ZipFile(io.BytesIO(out.getvalue())) as z: files = {n: z.read(n) for n in z.namelist()}
        bad = copy.deepcopy(self.s); bad['zones'][0]['parent_id'] = bad['zones'][1]['zone_id']; files['spatial.json'] = encode(bad)
        manifest = json.loads(files['manifest.json']); manifest['files']['spatial.json'] = dict(bytes=len(files['spatial.json']), sha256=sha(files['spatial.json'])); files['manifest.json'] = encode(manifest)
        forged = io.BytesIO()
        with zipfile.ZipFile(forged, 'w') as z:
            for n, raw in files.items(): z.writestr(n, raw)
        with self.assertRaises(ValueError): read_dataset(io.BytesIO(forged.getvalue()))
        legacy = payload(); self.store = ProjectStore(self.root/'legacy', SCENE); self.store.save(legacy)
        old = io.BytesIO(); export_dataset(self.store, 1, old, CATEGORIES)
        self.assertEqual(read_dataset(io.BytesIO(old.getvalue()))['manifest']['schema'], 'scenesem.semantic-dataset.v1')

    def test_boundary_contract_and_legacy_rejection(self):
        self.s['schema'] = 'scenesem.spatial-context.v2'
        self.s['zones'][0]['boundary'] = dict(type='aabb', min=[0, 0, 0], max=[5, 6, 3])
        validate_spatial(self.s)
        mutations = [lambda s: s.update(schema='scenesem.spatial-context.v1'),
            lambda s: s['zones'][0]['boundary'].update(type='obb'),
            lambda s: s['zones'][0]['boundary'].update(min=[0, True, 0]),
            lambda s: s['zones'][0]['boundary'].update(max=[0, 6, 3]),
            lambda s: s['zones'][0]['boundary'].update(extra=True)]
        for change in mutations:
            bad = copy.deepcopy(self.s); change(bad)
            with self.assertRaises(ValueError): validate_spatial(bad)

    def test_boundary_candidate_dataset_roundtrip(self):
        self.s['schema'] = 'scenesem.spatial-context.v2'
        self.s['zones'][1]['boundary'] = dict(type='aabb', min=[0, 0, 0], max=[5, 6, 3])
        self.s['memberships'][0].update(source='automatic_candidate', notes='Synthetic snapshot, not revalidated after edits')
        self.store.save(self.p)
        out = io.BytesIO(); export_dataset(self.store, 1, out, CATEGORIES)
        read = read_dataset(io.BytesIO(out.getvalue()))
        self.assertEqual(read['spatial'], self.s)
        self.assertEqual(read['manifest']['schema'], 'scenesem.semantic-dataset.v2')
        self.assertEqual(read['quality']['spatial']['verification'], 'unverified')

    def test_recheck_membership_version_export_and_restore(self):
        self.s['schema'] = 'scenesem.spatial-context.v2'
        self.s['memberships'][0]['source'] = 'automatic_candidate'
        initial = self.store.save(self.p)['document']
        changed = copy.deepcopy(self.p); changed['expected_revision'] = 1
        member = changed['spatial_context']['memberships'][0]
        member.update(zone_id=self.s['zones'][0]['zone_id'], notes='Synthetic recheck; snapshot only, not revalidated after edits')
        saved = self.store.save(changed)['document']
        self.assertEqual(saved['objects'], initial['objects'])
        self.assertEqual(saved['annotations'], initial['annotations'])
        out = io.BytesIO(); export_dataset(self.store, 2, out, CATEGORIES)
        data = read_dataset(io.BytesIO(out.getvalue()))
        self.assertEqual(data['spatial']['memberships'][0], member)
        self.assertEqual(data['quality']['spatial']['automatic_membership_currentness'], 'not_revalidated')
        restored = copy.deepcopy(self.p); restored['expected_revision'] = 2
        self.assertEqual(self.store.save(restored)['document']['spatial_context'], self.s)
        self.assertEqual(self.store.load(1)['document'], initial)
        self.assertEqual(self.store.load(2)['document'], saved)

    def test_bundle_import_preserves_context_and_checkpoint(self):
        self.s['schema'] = 'scenesem.spatial-context.v2'
        self.s['zones'][1]['boundary'] = dict(type='aabb', min=[0, 0, 0], max=[5, 6, 3])
        self.store.save(self.p); out = io.BytesIO(); self.store.export(1, out)
        target = ProjectStore(self.root/'target', SCENE); importer = BundleImporter(target, self.root/'staging')
        preview = importer.preview(io.BytesIO(out.getvalue()), len(out.getvalue()))
        self.assertEqual(preview['document']['spatial_context'], self.s)
        checkpoint = payload(); checkpoint['spatial_context'] = fixture(checkpoint['objects'][0]['object_id'])
        result = importer.commit(dict(import_id=preview['import_id'], expected_revision=0, checkpoint=checkpoint))
        self.assertEqual(result['document']['spatial_context'], self.s)
        self.assertEqual(target.load(result['checkpoint_revision'])['document']['spatial_context'], checkpoint['spatial_context'])


if __name__ == '__main__': unittest.main()
