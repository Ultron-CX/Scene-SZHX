import base64
import copy
import hashlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from project_store import ProjectStore, CATEGORIES
from test_project_store import SCENE, TMP, payload
from test_business_context import fixture
from semantic_dataset import validate_business, validate_source_snapshot, export_dataset, read_dataset
from bundle_import import BundleImporter


class MaterialTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(dir=TMP);self.root=Path(self.tmp.name);self.store=ProjectStore(self.root/'store',SCENE)
        self.raw=b'Synthetic source; not bank truth';self.snap=dict(name='source.txt',bytes=len(self.raw),sha256=hashlib.sha256(self.raw).hexdigest())
        self.data=dict(scene_id=SCENE['scene_id'],source_hashes=SCENE['source_hashes'],snapshot=self.snap,base64=base64.b64encode(self.raw).decode(),confirm_local_archive=True)
    def tearDown(self):self.tmp.cleanup()
    def test_archive_dedup_status_and_no_revision(self):
        self.assertEqual(self.store.material_status(self.data)['status'],'missing')
        self.store.archive_material(self.data);self.store.archive_material(self.data)
        self.assertEqual(self.store.material_payload(self.snap['sha256']),self.raw);self.assertEqual(len(list((self.store.root/'source-materials').iterdir())),1)
        self.assertEqual(self.store.load()['head'],0);self.assertEqual(self.store.material_status(self.data)['status'],'integrity_verified')
        (self.store.root/'source-materials'/self.snap['sha256']).write_bytes(b'corrupt')
        self.assertEqual(self.store.material_status(self.data)['status'],'invalid')
        with self.assertRaises(RuntimeError):self.store.archive_material(self.data)
    def test_reject_unconfirmed_wrong_scene_hash_size_path(self):
        for k,v in [('confirm_local_archive',False),('scene_id','wrong'),('base64','?')]:
            bad=copy.deepcopy(self.data);bad[k]=v
            with self.assertRaises(ValueError):self.store.archive_material(bad)
        for k,v in [('name','../source.txt'),('bytes',True),('bytes',9*1024*1024),('sha256','bad')]:
            s=copy.deepcopy(self.snap);s[k]=v
            with self.assertRaises(ValueError):validate_source_snapshot(s)
        bad=copy.deepcopy(self.data);bad['snapshot']['bytes']+=1
        with self.assertRaises(ValueError):self.store.archive_material(bad)
        self.assertFalse((self.store.root/'source-materials').exists())
    def test_reference_roundtrip_but_bytes_not_in_packages(self):
        self.store.archive_material(self.data);p=payload();p['business_context']=fixture(p['objects'][0]['object_id']);b=p['business_context'];b['attributes'][0]['source_snapshot']=self.snap
        with self.assertRaises(ValueError):validate_business(b)
        b['schema']='scenesem.business-context.v2';self.store.save(p)
        semantic=io.BytesIO();export_dataset(self.store,1,semantic,CATEGORIES);read=read_dataset(io.BytesIO(semantic.getvalue()))
        self.assertEqual(read['business'],b);self.assertFalse(read['quality']['business']['snapshot_bytes_included']);self.assertEqual(read['quality']['business']['snapshot_availability'],'requires_local_check')
        backup=io.BytesIO();self.store.export(1,backup);target=ProjectStore(self.root/'new-machine',SCENE);imp=BundleImporter(target,self.root/'stage');preview=imp.preview(io.BytesIO(backup.getvalue()),len(backup.getvalue()))
        result=imp.commit(dict(import_id=preview['import_id'],expected_revision=0,checkpoint=payload()))
        self.assertEqual(result['document']['business_context'],b);self.assertEqual(target.material_status(self.data)['status'],'missing')
        target.archive_material(self.data);self.assertEqual(target.material_status(self.data)['status'],'integrity_verified')
    def test_isolated_http_archive_and_safe_download(self):
        from http.server import ThreadingHTTPServer
        from unittest.mock import patch
        import http.client
        import threading
        import serve_annotation_tool as module
        class Quiet(module.Handler):
            def log_message(self,*args):pass
        (self.root/'scene.json').write_text(json.dumps(SCENE),encoding='utf8')
        with patch.object(module,'PROJECTS',self.store),patch.object(module,'SCENE',self.root):
            server=ThreadingHTTPServer(('127.0.0.1',0),Quiet);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            def call(method,route,data=None,local=True):
                c=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=5);headers={'Host':'127.0.0.1:18876','Content-Type':'application/json'}
                if local:headers['X-Digit-Local']='1'
                c.request(method,route,json.dumps(data) if data is not None else None,headers);r=c.getresponse();out=(r.status,dict(r.getheaders()),r.read());c.close();return out
            try:
                self.assertEqual(call('POST','/project/material/archive',self.data,False)[0],403)
                self.assertEqual(call('POST','/project/material/archive',self.data)[0],200)
                self.assertEqual(json.loads(call('POST','/project/material/check',self.data)[2])['status'],'integrity_verified')
                status,headers,raw=call('GET','/project/material/download/'+self.snap['sha256']);self.assertEqual(status,200);self.assertEqual(raw,self.raw);self.assertEqual(headers['Content-Type'],'application/octet-stream');self.assertTrue(headers['Content-Disposition'].startswith('attachment;'));self.assertEqual(headers['X-Content-Type-Options'],'nosniff')
                self.assertEqual(call('GET','/project/material/download/not-a-hash')[0],400)
            finally:server.shutdown();server.server_close();thread.join()


if __name__=='__main__':unittest.main()
