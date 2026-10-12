"""Synthetic S1 persistence/HTTP tests. Never uses the user's live project data."""
import base64
import copy
import hashlib
import http.client
import io
import json
import tempfile
import threading
import unittest
import uuid
import zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import patch
import numpy as np
from PIL import Image
from project_store import ProjectStore, DIMENSIONS

ROOT = Path(__file__).resolve().parents[1]
TMP = ROOT / '.tmp' / 's1-tests'
TMP.mkdir(parents=True, exist_ok=True)
SCENE = dict(scene_id='synthetic-s1-not-a-real-bank', source_hashes={'test': 'a' * 64}, coordinate_frame='TEST', T_lcc_from_las=[])


def payload(revision=0, count=1):
    boxes = [dict(id=f'region-{i}', category='chair', sequence=i + 1, status='reviewed', min=[i, 0, 0], max=[i + 1, 1, 1]) for i in range(count)]
    objects = [dict(object_id='obj-' + str(uuid.uuid4()), alias='', region_ids=[r['id']], evidence_ids=[], reviews=[], task_qualification=False) for r in boxes]
    return dict(expected_revision=revision, annotations=dict(schema='digit.regions.v5', **SCENE, boxes=boxes, real_robot_commands=False), objects=objects)


def observation():
    rgb = io.BytesIO()
    Image.new('RGB', (64, 64), (20, 40, 60)).save(rgb, format='JPEG')
    depth = np.full(64 * 64, .2, dtype='<f4')
    depth[0] = np.nan
    identity = np.eye(4).flatten().tolist()
    return dict(**{k: SCENE[k] for k in ('scene_id', 'source_hashes')}, image=base64.b64encode(rgb.getvalue()).decode(), frame=dict(width=64, height=64,
        depth_f32le=base64.b64encode(depth.tobytes()).decode(), inverse=identity, projection=identity, world_matrix=identity,
        camera_position=[0, 0, 0], near=.1, far=10, fov=60, horizontal_fov=False, pose_id='synthetic'))


class StoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=TMP)
        self.store = ProjectStore(Path(self.temp.name), SCENE)

    def tearDown(self):
        self.temp.cleanup()

    def test_legacy_roundtrip_and_no_automatic_review(self):
        p = payload(count=25)
        original = copy.deepcopy(p)
        r = self.store.save(p)
        self.assertEqual(r['document']['annotations'], p['annotations'])
        self.assertEqual(p, original)
        self.assertTrue(all(not o['reviews'] and not o['task_qualification'] and o['original_evidence'] == 'missing' for o in r['document']['objects']))
        self.assertEqual(ProjectStore(self.store.root, SCENE).load(), r)

    def test_conflict_and_concurrent_save(self):
        self.store.save(payload())
        with self.assertRaises(RuntimeError):
            self.store.save(payload())
        def attempt(_):
            try:
                return self.store.save(payload(1))['head']
            except RuntimeError:
                return 'conflict'
        with ThreadPoolExecutor(2) as pool:
            self.assertCountEqual(list(pool.map(attempt, range(2))), [2, 'conflict'])

    def test_identity_association_checkpoint_stale_history_and_restore(self):
        p = payload(count=3)
        for o in p['objects'][:2]:
            o['pending_review'] = dict(reviewer='synthetic', notes='old individual object only',
                checks={k: 'pass' if k == 'identity' else 'pending' for k in DIMENSIONS}, coverage_scope='unknown')
        initial = self.store.save(p)
        self.assertTrue(all(o['identity_state'] == 'human_reviewed_same_object' for o in initial['document']['objects'][:2]))
        def next_payload(saved, reason):
            return dict(expected_revision=saved['head'], annotations=copy.deepcopy(saved['document']['annotations']),
                objects=copy.deepcopy(saved['document']['objects']), reason=reason)
        checkpoint = self.store.save(next_payload(initial, 'checkpoint_before_identity_association'))
        change = next_payload(checkpoint, 'synthetic_identity_association')
        target, source = change['objects'][:2]
        source_id = source['object_id']
        target['region_ids'] += source['region_ids']
        change['objects'].remove(source)
        after = self.store.save(change)
        kept = after['document']['objects'][0]
        self.assertTrue(kept['review_stale'])
        self.assertEqual(kept['identity_state'], 'candidate_not_human_confirmed')
        self.assertFalse(kept['task_qualification'])
        self.assertEqual(kept['reviews'], checkpoint['document']['objects'][0]['reviews'])
        self.assertEqual(after['document']['annotations'], checkpoint['document']['annotations'])
        self.assertEqual(after['document']['objects'][1], checkpoint['document']['objects'][2])
        removed = [e for e in after['document']['object_events'] if e['object_id'] == source_id]
        self.assertEqual(removed[0]['kind'], 'object_record_removed_not_regions')
        self.assertEqual(self.store.load(checkpoint['head'])['document']['objects'][1], checkpoint['document']['objects'][1])
        restore = next_payload(checkpoint, 'restore_pre_association')
        restore['expected_revision'] = after['head']
        restored = self.store.save(restore)
        self.assertEqual(restored['document']['objects'], checkpoint['document']['objects'])
        self.assertEqual(restored['document']['annotations'], checkpoint['document']['annotations'])
        self.assertEqual(self.store.load(after['head'])['document'], after['document'])

    def test_failed_atomic_commit_keeps_old_head(self):
        self.store.save(payload())
        with patch('project_store.os.replace', side_effect=OSError('synthetic disk failure')):
            with self.assertRaises(OSError):
                self.store.save(payload(1))
        self.assertEqual(self.store.load()['head'], 1)

    def test_evidence_raw_depth_validity_dedup_and_later_kind(self):
        p = observation()
        eid = self.store.archive(p)
        self.assertEqual(eid, self.store.archive(p))
        m = self.store.evidence(eid)
        self.assertEqual(m['kind'], 'later_observation')
        self.assertIsNone(m['result'])
        self.assertEqual(m['valid_pixels'], 4095)
        self.assertEqual(self.store.blob(m['depth']), base64.b64decode(p['frame']['depth_f32le']))
        self.assertEqual(self.store.blob(m['validity'])[0], 0)
        self.assertEqual(len(list((self.store.root / 'blobs').iterdir())), 4)

    def test_original_result_archive_and_hash_match(self):
        p = observation()
        result = dict(width=64, height=64, image_sha256=hashlib.sha256(base64.b64decode(p['image'])).hexdigest(), components=[dict(runs=[[1, 2, 9]], instance_id=7)])
        eid = self.store.archive(p, result)
        m = self.store.evidence(eid)
        self.assertEqual(m['kind'], 'recognition')
        self.assertEqual(json.loads(self.store.blob(m['result'])), result)
        result['image_sha256'] = '0' * 64
        with self.assertRaises(ValueError):
            self.store.archive(p, result)

    def test_invalid_evidence_inputs_and_traversal(self):
        for field, value in [('depth_f32le', 'AA=='), ('world_matrix', [1]), ('width', 63), ('far', -1)]:
            p = observation()
            p['frame'][field] = value
            with self.assertRaises(ValueError):
                self.store.archive(p)
        with self.assertRaises(ValueError):
            self.store.blob('../../config/local.json')

    def test_mask_correction_scope_history_revert_and_review_staleness(self):
        data = observation()
        result = dict(width=64, height=64, scene_id=SCENE['scene_id'], source_hashes=SCENE['source_hashes'], image_sha256=hashlib.sha256(base64.b64decode(data['image'])).hexdigest(), components=[dict(category='computer', score=.9, instance_id=2, runs=[[1, 2, 9]])])
        eid = self.store.archive(data, result)
        original = self.store.blob(self.store.evidence(eid)['result'])
        p = payload()
        p['objects'][0]['evidence_ids'] = [eid]
        p['objects'][0]['pending_review'] = dict(reviewer='test only', notes='synthetic review', coverage_scope='partial', checks={k:'pass' for k in DIMENSIONS})
        first = self.store.save(p)['document']
        p = dict(expected_revision=1, annotations=copy.deepcopy(first['annotations']), objects=copy.deepcopy(first['objects']))
        correction = dict(evidence_id=eid, component_index=0, category='screen', reviewer='synthetic tester', notes='fixture mask, not a real review')
        p['objects'][0]['pending_mask_correction'] = correction
        second = self.store.save(p)['document']
        o = second['objects'][0]
        self.assertEqual(second['annotations'], first['annotations'])
        self.assertEqual(o['mask_corrections'][0]['raw_category'], 'computer')
        self.assertEqual(o['mask_corrections'][0]['scope'], 'evidence_mask_only_not_region_or_identity')
        self.assertEqual(o['mask_corrections'][0]['revision'], 2)
        self.assertTrue(o['review_stale'])
        self.assertFalse(o['task_qualification'])
        self.assertEqual(o['reviews'], first['objects'][0]['reviews'])
        self.assertEqual(self.store.blob(self.store.evidence(eid)['result']), original)
        p = dict(expected_revision=2, annotations=second['annotations'], objects=copy.deepcopy(second['objects']))
        p['objects'][0]['pending_mask_correction'] = dict(correction, category=None)
        third = self.store.save(p)['document']
        self.assertEqual(len(third['objects'][0]['mask_corrections']), 2)
        self.assertIsNone(third['objects'][0]['mask_corrections'][-1]['category'])
        package = io.BytesIO()
        self.store.export(3, package)
        from bundle_import import BundleImporter
        other = ProjectStore(self.store.root / 'clean-restore', SCENE)
        importer = BundleImporter(other, self.store.root / 'import-stage')
        preview = importer.preview(io.BytesIO(package.getvalue()), len(package.getvalue()))
        self.assertEqual(preview['document']['objects'][0]['mask_corrections'], third['objects'][0]['mask_corrections'])
        self.assertTrue(preview['document']['objects'][0]['import_requires_review'])

    def test_mask_correction_rejects_invalid_refs_fields_and_observation(self):
        data = observation()
        eid = self.store.archive(data, dict(width=64, height=64, image_sha256=hashlib.sha256(base64.b64decode(data['image'])).hexdigest(), components=[dict(category='computer', runs=[[1, 2, 9]])]))
        p = payload()
        p['objects'][0]['evidence_ids'] = [eid]
        correction = dict(evidence_id=eid, component_index=0, category='screen', reviewer='test', notes='synthetic')
        for key, value in [('evidence_id','a'*64), ('component_index',True), ('component_index',1), ('category',[]), ('category','bogus'), ('reviewer',''), ('notes','')]:
            with self.subTest(key=key, value=value):
                p['objects'][0]['pending_mask_correction'] = dict(correction, **{key:value})
                with self.assertRaises(ValueError):
                    self.store.save(p)
                self.assertEqual(self.store.load()['head'], 0)
        later = self.store.archive(data)
        p['objects'][0]['evidence_ids'] = [later]
        p['objects'][0]['pending_mask_correction'] = dict(correction, evidence_id=later)
        with self.assertRaisesRegex(ValueError, '后补观察'):
            self.store.save(p)

    def test_missing_or_tampered_evidence_rejects_save(self):
        p = payload()
        p['objects'][0]['evidence_ids'] = ['a' * 64]
        with self.assertRaises(FileNotFoundError):
            self.store.save(p)
        eid = self.store.archive(observation())
        p['objects'][0]['evidence_ids'] = [eid]
        depth = self.store.evidence(eid)['depth']
        # Intentional corruption only inside this test-owned temporary directory.
        (self.store.root / 'blobs' / depth).write_bytes(b'broken')
        with self.assertRaises(RuntimeError):
            self.store.save(p)

    def test_review_dimensions_staleness_and_no_task_promotion(self):
        p = payload()
        p['objects'][0]['evidence_ids'] = [self.store.archive(observation())]
        p['objects'][0]['pending_review'] = dict(reviewer='synthetic tester', notes='logic only', coverage_scope='visible_inspected', checks={k: 'pass' for k in DIMENSIONS})
        saved = self.store.save(p)['document']
        o = saved['objects'][0]
        self.assertFalse(o['review_stale'])
        self.assertEqual(o['original_evidence'], 'missing')
        self.assertFalse(o['task_qualification'])
        p = dict(expected_revision=1, annotations=saved['annotations'], objects=saved['objects'])
        p['annotations']['boxes'][0]['max'][0] += .1
        updated = self.store.save(p)['document']
        self.assertTrue(updated['objects'][0]['review_stale'])
        self.assertEqual(updated['objects'][0]['reviews'], o['reviews'])

    def test_evidence_pass_requires_files_and_all_dimensions(self):
        p = payload()
        p['objects'][0]['pending_review'] = dict(reviewer='test', notes='', checks={k: 'pass' for k in DIMENSIONS})
        with self.assertRaisesRegex(ValueError, '没有归档证据'):
            self.store.save(p)
        p['objects'][0]['pending_review']['checks'].pop('identity')
        with self.assertRaisesRegex(ValueError, '六项'):
            self.store.save(p)

    def test_identity_link_events_and_deleted_region(self):
        p = payload()
        first = self.store.save(p)['document']
        p = dict(expected_revision=1, annotations=first['annotations'], objects=first['objects'])
        p['annotations']['boxes'] = []
        o = self.store.save(p)['document']['objects'][0]
        self.assertEqual(o['object_id'], first['objects'][0]['object_id'])
        self.assertEqual(o['missing_region_ids'], ['region-0'])
        self.assertEqual(first['object_events'][0]['kind'], 'registered')

    def test_source_collision_and_duplicate_ownership_rejected(self):
        p = payload(count=2)
        p['objects'][1]['region_ids'] = p['objects'][0]['region_ids']
        with self.assertRaises(ValueError):
            self.store.save(p)
        p = payload()
        p['annotations']['source_hashes'] = {'wrong': 'hash'}
        with self.assertRaises(ValueError):
            self.store.save(p)
        p = payload()
        p['annotations']['T_lcc_from_las'] = [1]
        with self.assertRaises(ValueError):
            self.store.save(p)

    def test_restore_as_new_revision_and_complete_export(self):
        p = payload()
        p['objects'][0]['evidence_ids'] = [self.store.archive(observation())]
        first = self.store.save(p)['document']
        p = dict(expected_revision=1, annotations=copy.deepcopy(first['annotations']), objects=copy.deepcopy(first['objects']))
        p['objects'][0]['alias'] = 'changed'
        self.store.save(p)
        restored = self.store.save(dict(expected_revision=2, annotations=first['annotations'], objects=first['objects'], reason='restore_v1'))
        self.assertEqual(restored['head'], 3)
        self.assertEqual(self.store.load(2)['document']['objects'][0]['alias'], 'changed')
        stream = io.BytesIO()
        self.store.export(3, stream)
        with zipfile.ZipFile(stream) as z:
            manifest = json.loads(z.read('manifest.json'))
            self.assertEqual(hashlib.sha256(z.read('project.json')).hexdigest(), manifest['project_sha256'])
            for sha in manifest['blobs']:
                self.assertEqual(hashlib.sha256(z.read('blobs/' + sha)).hexdigest(), sha)
            self.assertFalse(manifest['source_scans_included'])

    def test_http_protocol_isolated_not_live_workbench(self):
        from http.server import ThreadingHTTPServer
        import serve_annotation_tool as server_module
        class QuietHandler(server_module.Handler):
            def log_message(self, *args):
                pass
        # Ephemeral test-only listener; production remains fixed at 18876.
        with patch.object(server_module, 'PROJECTS', self.store), patch.object(server_module, 'SCENE', Path(self.temp.name)), patch.object(server_module, 'ROOT', Path(self.temp.name)):
            (Path(self.temp.name) / 'scene.json').write_text(json.dumps(SCENE), encoding='utf-8')
            server = ThreadingHTTPServer(('127.0.0.1', 0), QuietHandler)
            t = threading.Thread(target=server.serve_forever, daemon=True)
            t.start()
            def call(method, route, value=None, local=True):
                c = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=10)
                headers = {'Host': '127.0.0.1:18876', 'Content-Type': 'application/json'}
                if isinstance(value, bytes):
                    headers['Content-Type'] = 'application/zip'
                if local:
                    headers['X-Digit-Local'] = '1'
                c.request(method, route, value if isinstance(value, bytes) else None if value is None else json.dumps(value), headers)
                response = c.getresponse()
                data = response.read()
                status = response.status
                c.close()
                return status, data
            try:
                self.assertEqual(call('POST', '/project/save', payload(), False)[0], 403)
                self.assertEqual(call('POST', '/project/save', payload())[0], 200)
                self.assertEqual(call('POST', '/project/save', payload())[0], 409)
                self.assertEqual(call('GET', '/project/revision/1')[0], 200)
                self.assertEqual(call('GET', '/project/revision/999')[0], 404)
                self.assertEqual(call('POST', '/evidence/capture', observation())[0], 200)
                class FakeModel:
                    def infer(self, *args):
                        return dict(width=64, height=64, components=[dict(runs=[[1, 2, 5]], category='chair', score=.9, instance_id=1)], backend='instance')
                with patch.object(server_module, 'INSTANCES', FakeModel()):
                    status, data = call('POST', '/semantic/infer', dict(**observation(), backend='instance', confidence=.6))
                    self.assertEqual(status, 200, data)
                    eid = json.loads(data)['evidence_id']
                    self.assertEqual(self.store.evidence(eid)['kind'], 'recognition')
                    self.assertEqual(call('GET', f'/evidence/{eid}/rgb')[0], 200)
                package = io.BytesIO()
                self.store.export(1, package)
                self.assertEqual(call('POST', '/project/import/preview', package.getvalue(), False)[0], 403)
                status, data = call('POST', '/project/import/preview', package.getvalue())
                self.assertEqual(status, 200, data)
                preview = json.loads(data)
                self.assertEqual(self.store.load()['head'], 1)
                status, data = call('POST', '/project/import/commit', dict(import_id=preview['import_id'], expected_revision=1, checkpoint=payload(1)))
                self.assertEqual(status, 200, data)
                self.assertEqual(json.loads(data)['head'], 3)
                self.assertEqual(call('GET', '/project/export/3')[0], 200)
                from semantic_dataset import read_dataset
                status, data = call('GET', '/project/dataset/3')
                self.assertEqual(status, 200, data)
                self.assertEqual(read_dataset(io.BytesIO(data))['scene']['revision'], 3)
                self.assertEqual(self.store.load()['head'], 3)
                self.assertEqual(call('GET', '/project/dataset/999')[0], 404)
            finally:
                server.shutdown()
                server.server_close()
                t.join()


if __name__ == '__main__':
    unittest.main(verbosity=2)
