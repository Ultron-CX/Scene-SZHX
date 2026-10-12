"""Synthetic offline material delivery. No production store or source files."""
import base64
import copy
import io
import json
import tempfile
import unittest
import zipfile
import uuid
from pathlib import Path
from unittest.mock import patch
from project_store import ProjectStore, CATEGORIES
from test_project_store import SCENE, TMP, payload
from test_business_context import fixture
from semantic_dataset import export_dataset, read_dataset, encode, sha


class DeliveryTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(dir=TMP);self.root=Path(self.tmp.name);self.store=ProjectStore(self.root/'store',SCENE)
        p=payload();b=fixture(p['objects'][0]['object_id']);b['schema']='scenesem.business-context.v2';p['business_context']=b
        b['attributes']=[dict(copy.deepcopy(b['attributes'][0]),attribute_id='attr-'+str(uuid.uuid4())) for _ in range(3)]
        self.raw=[b'Synthetic source A',b'Synthetic source B'];self.hashes=[sha(r) for r in self.raw]
        for i,r in enumerate(self.raw):
            s=dict(name='synthetic-'+str(i)+'.txt',bytes=len(r),sha256=sha(r));b['attributes'][i]['source_snapshot']=s
            self.store.archive_material(dict(scene_id=SCENE['scene_id'],source_hashes=SCENE['source_hashes'],snapshot=s,base64=base64.b64encode(r).decode(),confirm_local_archive=True))
        self.store.save(p)
    def tearDown(self):self.tmp.cleanup()
    def package(self,selected):
        out=io.BytesIO();export_dataset(self.store,1,out,CATEGORIES,material_hashes=selected);return out.getvalue()
    def rewrite(self,raw,change):
        with zipfile.ZipFile(io.BytesIO(raw)) as z:files={n:z.read(n) for n in z.namelist()}
        change(files);m=json.loads(files['manifest.json']);m['files']={n:dict(bytes=len(r),sha256=sha(r)) for n,r in files.items() if n!='manifest.json'};files['manifest.json']=encode(m)
        out=io.BytesIO()
        with zipfile.ZipFile(out,'w') as z:
            for n,r in files.items():z.writestr(n,r)
        return io.BytesIO(out.getvalue())
    def test_partial_all_and_empty_offline(self):
        before=self.store.load()
        for selected in ([],self.hashes[:1],self.hashes):
            raw=self.package(selected)
            with patch.object(self.store,'material_payload',side_effect=AssertionError('reader must be offline')):
                read=read_dataset(io.BytesIO(raw))
            self.assertEqual(read['manifest']['schema'],'scenesem.semantic-dataset.v4')
            self.assertEqual(read['quality']['business']['included_snapshot_count'],len(selected))
            self.assertEqual(read['materials']['all_referenced_snapshots_included'],len(selected)==2)
            self.assertEqual(len(read['materials']['unbound_attribute_ids']),1)
            self.assertFalse(read['materials']['source_content_verified'])
            with zipfile.ZipFile(io.BytesIO(raw)) as z:self.assertEqual({n.split('/')[1] for n in z.namelist() if n.startswith('materials/')},set(selected))
        self.assertEqual(self.store.load(),before)
    def test_missing_or_corrupt_selected_refused_omitted_allowed(self):
        with patch.object(self.store,'material_payload',side_effect=FileNotFoundError('missing')):
            with self.assertRaises(FileNotFoundError):self.package(self.hashes[:1])
            read_dataset(io.BytesIO(self.package([])))
        (self.store.root/'source-materials'/self.hashes[0]).write_bytes(b'corrupt')
        with self.assertRaises(RuntimeError):self.package(self.hashes[:1])
        for selected in ([self.hashes[0]]*2,['a'*64],['../path'],'all'):
            with self.assertRaises(ValueError):self.package(selected)
    def test_tampered_bytes_inventory_paths_and_legacy_schema_rejected(self):
        raw=self.package(self.hashes[:1]);h=self.hashes[0]
        def forge_inventory(files):
            v=json.loads(files['materials.json']);v['all_referenced_snapshots_included']=True;files['materials.json']=encode(v)
        def downgrade(files):
            v=json.loads(files['manifest.json']);v['schema']='scenesem.semantic-dataset.v3';files['manifest.json']=encode(v)
        for change in (lambda f:f.update({'materials/'+h:b'forged'}),forge_inventory,downgrade,
                       lambda f:f.pop('materials/'+h),lambda f:f.update({'materials/../secret':b'x'}),
                       lambda f:f.update({'materials/'+sha(b'unreferenced'):b'unreferenced'})):
            with self.assertRaises((ValueError,KeyError)):read_dataset(self.rewrite(raw,change))
    def test_shared_hash_dedup_and_conflicting_reference_size(self):
        doc=self.store.load()['document'];b=copy.deepcopy(doc['business_context']);b['attributes'][1]['source_snapshot']=copy.deepcopy(b['attributes'][0]['source_snapshot']);b['attributes'][1]['source_snapshot']['name']='alias.txt'
        self.store.save(dict(expected_revision=1,annotations=doc['annotations'],objects=doc['objects'],business_context=b))
        out=io.BytesIO();export_dataset(self.store,2,out,CATEGORIES,material_hashes=self.hashes[:1]);read=read_dataset(io.BytesIO(out.getvalue()));e=read['materials']['files'][0]
        self.assertEqual(len(e['names']),2);self.assertEqual(len(e['attribute_ids']),2)
        b['attributes'][1]['source_snapshot']['bytes']+=1
        with patch.object(self.store,'load',return_value={'document':dict(doc,business_context=b)}):
            with self.assertRaisesRegex(ValueError,'Conflicting'):self.package(self.hashes[:1])
    def test_isolated_http_permission_selection_and_binary(self):
        from http.server import ThreadingHTTPServer
        import http.client
        import threading
        import serve_annotation_tool as module
        class Quiet(module.Handler):
            def log_message(self,*args):pass
        (self.root/'scene.json').write_text(json.dumps(SCENE),encoding='utf8')
        with patch.object(module,'PROJECTS',self.store),patch.object(module,'SCENE',self.root),patch.object(module,'ROOT',self.root):
            server=ThreadingHTTPServer(('127.0.0.1',0),Quiet);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            def call(data,local=True,route='/project/dataset-selected'):
                c=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=5);headers={'Host':'127.0.0.1:18876','Content-Type':'application/json'}
                if local:headers['X-Digit-Local']='1'
                c.request('POST',route,json.dumps(data),headers);r=c.getresponse();out=(r.status,r.getheader('Content-Type'),r.read());c.close();return out
            data=dict(scene_id=SCENE['scene_id'],source_hashes=SCENE['source_hashes'],revision=1,material_hashes=self.hashes[:1],confirm_material_export=True)
            try:
                self.assertEqual(call(data,False)[0],403)
                self.assertEqual(call(dict(data,confirm_material_export=False))[0],400)
                self.assertEqual(call(dict(data,material_hashes=['a'*64]))[0],400)
                status,mime,raw=call(data);self.assertEqual(status,200);self.assertEqual(mime,'application/zip');read=read_dataset(io.BytesIO(raw));self.assertEqual(read['quality']['business']['included_snapshot_count'],1)
                self.assertEqual(self.store.load()['head'],1)
                from bundle_import import BundleImporter
                status,mime,backup=call(data,route='/project/bundle-selected');self.assertEqual(status,200);self.assertEqual(mime,'application/zip')
                target=ProjectStore(self.root/'http-target',SCENE);imp=BundleImporter(target,self.root/'http-stage');p=imp.preview(io.BytesIO(backup),len(backup));self.assertEqual(p['material_restore_count'],1)
                self.assertEqual(call(dict(data,confirm_material_export=False),route='/project/bundle-selected')[0],400)
            finally:server.shutdown();server.server_close();thread.join()


if __name__=='__main__':unittest.main()
