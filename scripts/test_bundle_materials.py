"""Isolated project+material restore, using synthetic bytes only."""
import io
import json
import unittest
from unittest.mock import patch
from project_store import ProjectStore, encoded
from bundle_import import BundleImporter, ImportCommitError
from test_project_store import SCENE, payload
import test_material_delivery as delivery
from test_bundle_import import rebuild


class BundleMaterialTests(unittest.TestCase):
    def setUp(self):
        self.fixture=delivery.DeliveryTests();self.fixture.setUp();self.source=self.fixture.store;self.root=self.fixture.root
        self.target=ProjectStore(self.root/'target',SCENE);self.importer=BundleImporter(self.target,self.root/'stage')
    def tearDown(self):self.fixture.tearDown()
    def package(self,selected=None):
        out=io.BytesIO();self.source.export(1,out,material_hashes=self.fixture.hashes if selected is None else selected);return out.getvalue()
    def preview(self,raw=None):
        raw=self.package() if raw is None else raw;return self.importer.preview(io.BytesIO(raw),len(raw))
    def request(self,p):return dict(import_id=p['import_id'],expected_revision=0,checkpoint=payload(),confirm_material_restore=True)
    def test_partial_permission_restore_and_checkpoint(self):
        p=self.preview(self.package(self.fixture.hashes[:1]));self.assertFalse(self.target.root.exists())
        self.assertEqual(p['material_restore_count'],1);self.assertIn('missing',[e['status'] for e in p['material_availability']])
        d=self.request(p)
        with self.assertRaises(ValueError):self.importer.commit(dict(d,confirm_material_restore=False))
        self.assertFalse(self.target.root.exists())
        result=self.importer.commit(d);self.assertEqual(result['head'],2);self.assertEqual(result['checkpoint_revision'],1)
        self.assertEqual(self.target.material_payload(self.fixture.hashes[0]),self.fixture.raw[0])
        self.assertFalse((self.target.root/'source-materials'/self.fixture.hashes[1]).exists())
        self.assertEqual(result['document']['business_context'],self.source.load()['document']['business_context'])
        self.assertTrue(all(o['import_requires_review'] for o in result['document']['objects']))
        self.assertEqual(self.importer.commit(d)['head'],2)
        checkpoint=self.target.load(1)['document'];self.target.save(dict(expected_revision=2,annotations=checkpoint['annotations'],objects=checkpoint['objects'],business_context=checkpoint.get('business_context')))
        self.assertEqual(self.target.material_payload(self.fixture.hashes[0]),self.fixture.raw[0])
    def test_legacy_reference_only_and_dedup(self):
        out=io.BytesIO();self.source.export(1,out);p=self.preview(out.getvalue());self.assertEqual(p['material_restore_count'],0)
        self.assertEqual({e['status'] for e in p['material_availability']},{'missing'})
        d=self.request(p);d.pop('confirm_material_restore');self.importer.commit(d)
        p=self.preview();d=self.request(p);d['expected_revision']=2;doc=self.target.load()['document'];d['checkpoint'].update(annotations=doc['annotations'],objects=doc['objects'],business_context=doc['business_context']);result=self.importer.commit(d);self.assertEqual(result['head'],4)
        p2=self.preview();self.assertEqual(p2['material_capacity']['additional_bytes'],0)
    def test_quota_and_conflict_rechecked_before_any_version(self):
        with patch('project_store.MATERIAL_QUOTA',1):
            with self.assertRaises(ValueError):self.preview()
        self.assertFalse(self.target.root.exists())
        p=self.preview()
        with patch('project_store.MATERIAL_QUOTA',1):
            with self.assertRaises(ValueError):self.importer.commit(self.request(p))
        self.assertFalse(self.target.root.exists())
        folder=self.target.root/'source-materials';folder.mkdir(parents=True);bad=folder/self.fixture.hashes[0];bad.write_bytes(b'corrupt')
        with self.assertRaises(RuntimeError):self.importer.commit(self.request(p))
        with self.assertRaises(RuntimeError):self.preview()
        self.assertEqual(bad.read_bytes(),b'corrupt');self.assertEqual(self.target.load()['head'],0)
    def test_staged_corruption_and_failed_final_write_retry(self):
        p=self.preview();stage=self.importer.folder(p['import_id'])/'source-materials'/self.fixture.hashes[0];stage.write_bytes(b'bad')
        with self.assertRaises(RuntimeError):self.importer.commit(self.request(p))
        self.assertFalse(self.target.root.exists())
        p=self.preview();d=self.request(p);save=self.target.save
        def fail(data,**kw):
            if data.get('reason','').startswith('bundle_import:') and not kw.get('validate_only'):raise OSError('synthetic final write failure')
            return save(data,**kw)
        with patch.object(self.target,'save',side_effect=fail):
            with self.assertRaises(ImportCommitError):self.importer.commit(d)
        self.assertEqual(self.target.load()['head'],1);self.assertEqual(self.target.material_payload(self.fixture.hashes[0]),self.fixture.raw[0])
        self.assertEqual(self.importer.commit(d)['head'],2);self.assertEqual(self.importer.commit(d)['head'],2)
    def test_manifest_and_byte_forgery_rejected(self):
        raw=self.package(self.fixture.hashes[:1]);h=self.fixture.hashes[0]
        import zipfile
        with zipfile.ZipFile(io.BytesIO(raw)) as z:m=json.loads(z.read('manifest.json'))
        for modified in (dict(m,schema='scenesem.bundle.v1'),dict(m,source_materials_included=False),dict(m,materials=dict(m['materials'],all_referenced_snapshots_included=True))):
            with self.assertRaises(ValueError):self.preview(rebuild(raw,{'manifest.json':encoded(modified)}))
        with self.assertRaises(ValueError):self.preview(rebuild(raw,{'materials/'+h:b'forged'}))
        with self.assertRaises(ValueError):self.preview(rebuild(raw,extra=('materials/../secret',b'x')))
        self.assertFalse(self.target.root.exists())


if __name__=='__main__':unittest.main()
