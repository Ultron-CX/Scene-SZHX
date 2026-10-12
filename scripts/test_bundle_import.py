"""Bounded ZIP recovery checks, using only test-owned temporary stores."""
import copy
import hashlib
import io
import json
import stat
import tempfile
import unittest
import warnings
import zipfile
from pathlib import Path
from unittest.mock import patch
from bundle_import import BundleImporter, ImportCommitError
from project_store import ProjectStore, encoded, DIMENSIONS
from test_project_store import ROOT, TMP, SCENE, payload, observation


def rebuild(raw, changes=None, extra=None):
    with zipfile.ZipFile(io.BytesIO(raw)) as z:
        files = {n: z.read(n) for n in z.namelist()}
    files.update(changes or {})
    if 'project.json' in (changes or {}):
        m = json.loads(files['manifest.json'])
        m['project_sha256'] = hashlib.sha256(files['project.json']).hexdigest()
        files['manifest.json'] = encoded(m)
    out = io.BytesIO()
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', UserWarning)
        with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED) as z:
            for name, content in files.items():
                z.writestr(name, content)
            if extra:
                z.writestr(*extra)
    return out.getvalue()


class BundleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=TMP)
        self.root = Path(self.temp.name)
        self.source = ProjectStore(self.root / 'source', SCENE)
        self.target = ProjectStore(self.root / 'target', SCENE)
        self.importer = BundleImporter(self.target, self.root / 'staging')
        p = payload(count=25)
        f = observation()
        later = self.source.archive(f)
        result = dict(scene_id=SCENE['scene_id'], source_hashes=SCENE['source_hashes'], width=64, height=64,
            image_sha256=hashlib.sha256(__import__('base64').b64decode(f['image'])).hexdigest(),
            components=[dict(category='chair', score=.9, instance_id=1, runs=[[1, 2, 8]])])
        original = self.source.archive(f, result)
        p['objects'][0]['evidence_ids'] = [later, original]
        p['objects'][0]['pending_review'] = dict(reviewer='synthetic only', notes='visible synthetic fixture', coverage_scope='visible_inspected', checks={d:'pass' for d in DIMENSIONS})
        self.original = self.source.save(p)['document']
        out = io.BytesIO()
        self.source.export(1, out)
        self.zip = out.getvalue()

    def tearDown(self):
        self.temp.cleanup()

    def preview(self, raw=None):
        raw = raw or self.zip
        return self.importer.preview(io.BytesIO(raw), len(raw))

    def commit_request(self, preview, checkpoint=None):
        checkpoint = checkpoint or payload(count=1)
        return dict(import_id=preview['import_id'], expected_revision=self.target.load()['head'], checkpoint=checkpoint)

    def test_empty_store_roundtrip_all_blobs_and_review_reconfirmation(self):
        p = self.preview()
        self.assertFalse(self.target.root.exists(), 'preview must not write the real target')
        self.assertEqual(p['objects'], 25)
        self.assertEqual(p['evidence'], 2)
        self.assertTrue(p['document']['objects'][0]['review_stale'])
        result = self.importer.commit(self.commit_request(p))
        self.assertEqual(result['head'], 2)
        self.assertEqual(result['checkpoint_revision'], 1)
        self.assertEqual(result['document']['annotations'], self.original['annotations'])
        self.assertEqual([o['object_id'] for o in result['document']['objects']], [o['object_id'] for o in self.original['objects']])
        self.assertTrue(result['document']['objects'][0]['review_stale'])
        ordinary = self.target.save(dict(expected_revision=2, annotations=result['document']['annotations'], objects=result['document']['objects']))
        self.assertTrue(ordinary['document']['objects'][0]['review_stale'], 'ordinary save cannot reapprove imported history')
        for eid in result['document']['objects'][0]['evidence_ids']:
            for key in ('rgb', 'depth', 'validity', 'result'):
                m = self.source.evidence(eid)
                if m.get(key):
                    self.assertEqual(self.source.blob(m[key]), self.target.blob(m[key]))
        out = io.BytesIO()
        self.target.export(3, out)
        second = BundleImporter(ProjectStore(self.root/'second', SCENE), self.root/'second-stage')
        again = second.preview(io.BytesIO(out.getvalue()), len(out.getvalue()))
        self.assertEqual(again['document']['annotations'], self.original['annotations'])

    def test_idempotent_response_loss_and_changed_work_rejected(self):
        p = self.preview()
        request = self.commit_request(p)
        first = self.importer.commit(request)
        again = self.importer.commit(request)
        self.assertTrue(again['already_committed'])
        self.assertEqual(first['head'], again['head'])
        self.assertEqual(self.target.revisions(), [1, 2])
        changed = copy.deepcopy(request)
        changed['checkpoint']['annotations']['boxes'][0]['category'] = 'table'
        with self.assertRaises(RuntimeError):
            self.importer.commit(changed)
        self.assertEqual(self.target.revisions(), [1, 2])

    def test_conflict_no_target_mutation_and_failed_final_write_recovery(self):
        p = self.preview()
        request = self.commit_request(p)
        self.target.save(payload())
        with self.assertRaises(RuntimeError):
            self.importer.commit(request)
        request['expected_revision'] = 1
        atomic = self.target.atomic
        def fail_final(path, data):
            if path == self.target.root / 'revisions' / '00000003.json':
                raise OSError('synthetic full disk during final commit')
            atomic(path, data)
        with patch.object(self.target, 'atomic', side_effect=fail_final):
            with self.assertRaises(ImportCommitError) as error:
                self.importer.commit(request)
        self.assertEqual(error.exception.checkpoint_revision, 2)
        self.assertEqual(self.target.revisions(), [1, 2])
        # Retry with the old expected revision, as if the error response was lost.
        result = self.importer.commit(request)
        self.assertEqual(result['head'], 3)
        self.assertEqual(result['checkpoint_revision'], 2)

    def test_archive_path_duplicate_symlink_and_expansion_limits(self):
        link = zipfile.ZipInfo('blobs/'+'f'*64)
        link.create_system = 3
        link.external_attr = (stat.S_IFLNK | 0o777) << 16
        for extra in [('../escape.txt', b'bad'), ('C:/escape.txt', b'bad'), ('blobs\\escape', b'bad'), ('project.json', b'{}'), (link, b'../escape')]:
            with self.subTest(extra=extra[0]), self.assertRaises(ValueError):
                self.preview(rebuild(self.zip, extra=extra))
        with patch('bundle_import.MAX_EXPANDED', 32), self.assertRaises(ValueError):
            self.preview()
        with self.assertRaises(ValueError):
            self.importer.preview(io.BytesIO(self.zip[:8]), len(self.zip))
        self.assertFalse(self.target.root.exists())
        self.assertFalse((self.root/'escape.txt').exists())

    def test_wrong_batch_invalid_geometry_and_pending_review_rejected(self):
        for change in ('batch', 'geometry', 'pending', 'nan', 'duplicate'):
            doc = copy.deepcopy(self.original)
            if change == 'batch':
                doc['annotations']['source_hashes'] = {'wrong':'batch'}
            elif change == 'geometry':
                doc['annotations']['boxes'][0]['geometry'] = dict(type='surface-voxels', origin=[0,0,0], cell=.04, cells=[[1,2,3]])
            elif change == 'pending':
                doc['objects'][0]['pending_review'] = dict(reviewer='must not execute')
            raw = encoded(doc)
            if change == 'nan':
                raw = raw.replace(b'"revision":1', b'"revision":NaN', 1)
            if change == 'duplicate':
                raw = raw.replace(b'"revision":1', b'"revision":1,"revision":1', 1)
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.preview(rebuild(self.zip, {'project.json':raw}))
        self.assertFalse(self.target.root.exists())

    def test_blob_hash_missing_file_and_staged_corruption(self):
        with zipfile.ZipFile(io.BytesIO(self.zip)) as z:
            names = z.namelist()
            blob = next(n for n in names if n.startswith('blobs/'))
            files = {n:z.read(n) for n in names if n != blob}
        with self.assertRaises(ValueError):
            self.preview(rebuild(self.zip, {blob:b'tampered'}))
        incomplete = io.BytesIO()
        with zipfile.ZipFile(incomplete, 'w') as z:
            for n, b in files.items():
                z.writestr(n, b)
        with self.assertRaises(ValueError):
            self.preview(incomplete.getvalue())
        p = self.preview()
        folder = self.importer.folder(p['import_id'])
        (folder / blob).write_bytes(b'corrupted inside test staging only')
        with self.assertRaises(RuntimeError):
            self.importer.commit(self.commit_request(p))
        self.assertEqual(self.target.revisions(), [])

    def test_existing_k2_structures_geometry_contract_read_only(self):
        scene = json.loads((ROOT/'artifacts/k2-annotation-scene-v2/scene.json').read_text(encoding='utf-8'))
        labels = json.loads((ROOT/'artifacts/k2-annotation-structures-v2/proposals.json').read_text(encoding='utf-8'))
        # The frozen proposals are v4; the actual workbench wraps accepted regions
        # in BoxCore.save's v5 envelope. Do the same in memory, never edit the fixture.
        labels['schema'] = 'digit.regions.v5'
        store = ProjectStore(self.root/'k2-fixture', scene)
        # Existing frozen fixture only; this is not current browser data or human QA.
        store.save(dict(expected_revision=0, annotations=labels, objects=[]))
        out = io.BytesIO()
        store.export(1, out)
        importer = BundleImporter(ProjectStore(self.root/'k2-clean', scene), self.root/'k2-stage')
        p = importer.preview(io.BytesIO(out.getvalue()), len(out.getvalue()))
        self.assertEqual(p['regions'], 25)
        self.assertEqual(p['evidence'], 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
