"""HTTP routing tests use synthetic stores on an ephemeral test port only."""
import http.client
import json
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch
from http.server import ThreadingHTTPServer
import prepare_r1_sample as prep
from project_store import ProjectStore
from sample_scope import resolve_sample


class ScopeTests(unittest.TestCase):
    def test_two_sample_writes_cannot_change_live_head(self):
        import serve_annotation_tool as service
        parent = prep.ROOT / '.tmp' / 'scope-tests'
        parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=parent) as temporary:
            root = Path(temporary)
            scene = dict(scene_id='synthetic-scope',source_hashes={'test':'a'*64},coordinate_frame='TEST',T_lcc_from_las=[])
            source = root / 'scene.json';source.write_text(json.dumps(scene))
            with patch.object(prep,'ROOT',root):
                a=prep.prepare(source,root/'.tmp'/'r1-samples')
                b=prep.prepare(source,root/'.tmp'/'r1-samples')
            live=ProjectStore(root/'live',scene)
            payload=dict(expected_revision=0,annotations=dict(schema='digit.regions.v5',**scene,boxes=[],real_robot_commands=False),objects=[])
            live.save(payload)
            class Quiet(service.Handler):
                def log_message(self,*args): pass
            with patch.object(service,'ROOT',root),patch.object(service,'SCENE',root),patch.object(service,'PROJECTS',live):
                server=ThreadingHTTPServer(('127.0.0.1',0),Quiet)
                thread=threading.Thread(target=server.serve_forever);thread.start()
                def call(method,path,data=None):
                    conn=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=10)
                    conn.request(method,path,json.dumps(data) if data else None,{'Host':'127.0.0.1:18876','X-Digit-Local':'1','Content-Type':'application/json'})
                    response=conn.getresponse();status=response.status;body=response.read();conn.close();return status,body
                try:
                    payload['expected_revision']=1
                    self.assertEqual(call('POST',f'/r1-sample/{a.name}/project/save',payload)[0],200)
                    self.assertEqual(resolve_sample(root,a.name,source).load()['head'],2)
                    self.assertEqual(resolve_sample(root,b.name,source).load()['head'],1)
                    self.assertEqual(live.load()['head'],1)
                    self.assertEqual(call('GET','/r1-sample/r1-sample-missing0/project/head')[0],400)
                    self.assertEqual(call('GET',f'/r1-sample/{a.name}/project/dataset/2')[0],200)
                    self.assertEqual(call('GET','/project/head')[0],200)
                    source.write_text(json.dumps({**scene,'coordinate_frame':'CHANGED'}))
                    self.assertEqual(call('POST',f'/r1-sample/{a.name}/project/save',payload)[0],400)
                    self.assertEqual(live.load()['head'],1)
                finally:
                    server.shutdown();server.server_close();thread.join()


if __name__=='__main__':unittest.main()
