"""Local-only annotation workbench. Source scans are never served or modified."""
import json
import importlib.util
import tempfile
from pathlib import Path
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import numpy as np
from object_semantics import ObjectSemantics, decode_request, tour_positions
from instance_semantics import InstanceSemantics, MODEL as INSTANCE_MODEL, MODEL_ID as INSTANCE_MODEL_ID
from scenesem_config import ROOT, FIXTURE, SCENE, ENGINE, PROPOSALS, LEGACY_PROPOSALS, VISUALS, HOST, PORT, check_inputs
from project_store import ProjectStore, CATEGORIES
from object_geometry import region_ply, geometry_filename
from semantic_dataset import export_dataset, read_dataset
from sample_scope import resolve_sample
from bundle_import import BundleImporter, ImportCommitError, MAX_UPLOAD

WEB = ROOT / 'frontend'
SEMANTICS = ObjectSemantics()
INSTANCES = InstanceSemantics()
SCENE_META = json.loads((SCENE / 'scene.json').read_text(encoding='utf-8'))
# A fixed directory is bound to the scene/source hashes in every stored document.
PROJECTS = ProjectStore(ROOT / 'data' / 'projects' / 'k2', SCENE_META)


def make_data():
    report = json.loads((FIXTURE / 'report.json').read_text(encoding='utf-8'))
    with np.load(FIXTURE / 'annotation-fixture.npz', allow_pickle=False) as data:
        return dict(schema='digit.point-block.v1', name='示例场景 3 · K2 · 局部块 01',
                    source_sha256=next(v for k, v in report['hashes'].items() if k.endswith('.las')),
                    coordinate_frame='LAS_SOURCE', coverage='explicit_source_points_only',
                    xyz=data['xyz'].tolist(), rgb=data['rgb'].tolist(),
                    ids=data['source_point_ids'].tolist())


class Handler(BaseHTTPRequestHandler):
    def json_response(self, status, data):
        payload = json.dumps(data, ensure_ascii=False, allow_nan=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def select_scope(self):
        self.projects = PROJECTS
        if self.path.startswith('/r1-sample/'):
            parts = self.path.split('/', 3)
            if len(parts) != 4:
                raise ValueError('Sample path requires trailing slash')
            self.projects = resolve_sample(ROOT, parts[2], SCENE / 'scene.json')
            self.path = '/' + parts[3]

    def do_POST(self):
        try:
            self.select_scope()
        except (ValueError, FileNotFoundError, KeyError) as error:
            self.json_response(400, {'error': str(error)})
            return
        if self.path not in ('/semantic/infer', '/project/save', '/evidence/capture', '/project/import/preview', '/project/import/commit', '/project/material/archive', '/project/material/check', '/project/dataset-selected', '/project/bundle-selected', '/project/geometry-export'):
            self.send_error(404)
            return
        origins = ('http://127.0.0.1:18876', 'http://localhost:18876')
        if (self.headers.get('Host') not in ('127.0.0.1:18876', 'localhost:18876') or
                self.headers.get('Origin') not in (None, *origins) or
                self.headers.get('X-Digit-Local') != '1'):
            self.json_response(403, {'error': '仅允许本机工作台调用'})
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            importer = BundleImporter(self.projects, ROOT / '.tmp' / 'bundle-import' if self.projects is PROJECTS else self.projects.root / 'import-staging')
            if self.path == '/project/import/preview':
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/zip' or not 0 < length <= MAX_UPLOAD:
                    raise ValueError('请上传小于或等于 512 MiB 的 SceneSem ZIP 包')
                self.connection.settimeout(60)
                self.json_response(200, importer.preview(self.rfile, length))
                return
            limit = 64 if self.path in ('/project/save', '/project/import/commit') else 16
            if not 0 < length <= limit * 1024 * 1024:
                raise ValueError('请求尺寸超出限制')
            data = json.loads(self.rfile.read(length))
            scene = json.loads((SCENE / 'scene.json').read_text(encoding='utf-8'))
            if self.path in ('/project/dataset-selected','/project/bundle-selected'):
                self.projects.scene_check(data)
                if data.get('confirm_material_export') is not True: raise ValueError('Explicit material export permission required')
                revision=data.get('revision')
                if type(revision) is not int or revision<1: raise ValueError('Explicit saved revision required')
                if not isinstance(data.get('material_hashes'),list): raise ValueError('Explicit material selection required')
                temp_root=ROOT/'.tmp'/'project-export';temp_root.mkdir(parents=True,exist_ok=True)
                with tempfile.TemporaryFile(dir=temp_root) as stream:
                    is_bundle=self.path=='/project/bundle-selected'
                    if is_bundle:self.projects.export(revision,stream,material_hashes=data['material_hashes'])
                    else:export_dataset(self.projects,revision,stream,CATEGORIES,material_hashes=data['material_hashes'])
                    if not is_bundle:stream.seek(0);read_dataset(stream)
                    stream.seek(0,2);size=stream.tell();stream.seek(0)
                    self.send_response(200);self.send_header('Content-Type','application/zip')
                    self.send_header('Content-Length',str(size));self.send_header('Content-Disposition',f'attachment; filename="SceneSem-{"project" if is_bundle else "semantic-dataset"}-v{revision}-materials.zip"')
                    self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers()
                    while chunk:=stream.read(1024*1024):self.wfile.write(chunk)
                return
            if self.path == '/project/import/commit':
                self.json_response(200, importer.commit(data))
                return
            if self.path == '/project/material/archive':
                self.json_response(200, self.projects.archive_material(data)); return
            if self.path == '/project/material/check':
                self.json_response(200, self.projects.material_status(data)); return
            if self.path == '/project/geometry-export':
                revision = data.get('revision')
                if type(revision) is not int or revision < 1:
                    raise ValueError('需要先保存项目版本，再导出单个物体的几何')
                region_id = data.get('region_id')
                if not isinstance(region_id, str) or not region_id:
                    raise ValueError('未选中有效区域')
                document = self.projects.load(revision)['document']
                region = next((r for r in document['annotations']['boxes'] if r.get('id') == region_id), None)
                if region is None:
                    raise ValueError('该区域不在所选的已保存版本中，请先保存项目再导出')
                payload = region_ply(scene, region)
                self.send_response(200)
                self.send_header('Content-Type', 'application/octet-stream')
                self.send_header('Content-Length', str(len(payload)))
                self.send_header('Content-Disposition', 'attachment; filename="%s"' % geometry_filename(region, revision))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.end_headers()
                self.wfile.write(payload)
                return
            if self.path == '/project/save':
                self.json_response(200, self.projects.save(data))
                return
            if self.path == '/evidence/capture':
                self.json_response(200, {'evidence_id': self.projects.archive(data)})
                return
            prepared = self.projects.prepare_frame(data) if 'frame' in data else None
            image, confidence, image_hash = decode_request(data, scene)
            # Old open tabs remain on their old protocol until the user reloads.
            backend = data.get('backend', 'legacy')
            if backend == 'instance':
                result = INSTANCES.infer(image, confidence, data.get('detail', 'standard'), data.get('target_roi'))
            elif backend == 'legacy':
                result = SEMANTICS.infer(image, confidence)
            else:
                raise ValueError('未知识别后端')
            result.update(image_sha256=image_hash, scene_id=scene['scene_id'], source_hashes=scene['source_hashes'])
            if prepared is not None:
                result['evidence_id'] = self.projects.archive(data, result, prepared)
            self.json_response(200, result)
        except (ValueError, KeyError, TypeError) as error:
            self.json_response(400, {'error': str(error)})
        except ImportCommitError as error:
            self.json_response(503, {'error': str(error), 'checkpoint_revision': error.checkpoint_revision})
        except RuntimeError as error:
            self.json_response(409, {'error': str(error)})
        except FileNotFoundError as error:
            self.json_response(400, {'error': '导入包或引用文件缺失，请重新选择完整项目包：' + str(error)})
        except Exception as error:
            self.json_response(503, {'error': '本地处理失败（未返回成功保存）：' + str(error)})

    def do_GET(self):
        try:
            self.select_scope()
        except (ValueError, FileNotFoundError, KeyError) as error:
            self.json_response(400, {'error': str(error)})
            return
        route = self.path.split('?')[0]
        if route.startswith(('/project/', '/evidence/')):
            if self.headers.get('Host') not in ('127.0.0.1:18876', 'localhost:18876') or self.headers.get('Origin') not in (None, 'http://127.0.0.1:18876', 'http://localhost:18876'):
                self.json_response(403, {'error': '仅允许本机工作台读取'})
                return
            try:
                if route == '/project/head':
                    self.json_response(200, self.projects.load())
                elif route == '/project/revisions':
                    self.json_response(200, {'revisions': self.projects.revisions()})
                elif route.startswith('/project/revision/'):
                    self.json_response(200, self.projects.load(int(route.rsplit('/', 1)[1])))
                elif route.startswith(('/project/export/', '/project/dataset/')):
                    revision = int(route.rsplit('/', 1)[1])
                    temp_root = ROOT / '.tmp' / 'project-export'
                    temp_root.mkdir(parents=True, exist_ok=True)
                    with tempfile.TemporaryFile(dir=temp_root) as stream:
                        is_dataset = route.startswith('/project/dataset/')
                        if is_dataset:
                            export_dataset(self.projects, revision, stream, CATEGORIES)
                            stream.seek(0)
                            read_dataset(stream)
                            stream.seek(0, 2)
                        else:
                            self.projects.export(revision, stream)
                        size = stream.tell()
                        stream.seek(0)
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/zip')
                        self.send_header('Content-Length', str(size))
                        label = 'semantic-dataset' if is_dataset else 'project'
                        self.send_header('Content-Disposition', f'attachment; filename="SceneSem-{label}-v{revision}.zip"')
                        self.send_header('Cache-Control', 'no-store')
                        self.end_headers()
                        while chunk := stream.read(1024 * 1024):
                            self.wfile.write(chunk)
                elif route.startswith('/project/material/download/'):
                    sha=route.rsplit('/',1)[1]; payload=self.projects.material_payload(sha)
                    self.send_response(200); self.send_header('Content-Type','application/octet-stream')
                    self.send_header('Content-Length',str(len(payload))); self.send_header('Content-Disposition','attachment; filename="source-'+sha+'.bin"')
                    self.send_header('X-Content-Type-Options','nosniff'); self.send_header('Content-Security-Policy',"sandbox; default-src 'none'")
                    self.send_header('Cache-Control','no-store'); self.end_headers(); self.wfile.write(payload)
                elif route.startswith('/evidence/'):
                    _, _, eid, asset = route.split('/')
                    if asset == 'manifest':
                        self.json_response(200, self.projects.evidence(eid))
                    else:
                        payload, mime = self.projects.evidence_asset(eid, asset)
                        self.send_response(200)
                        self.send_header('Content-Type', mime)
                        self.send_header('Content-Length', str(len(payload)))
                        self.send_header('Cache-Control', 'no-store')
                        self.send_header('X-Content-Type-Options', 'nosniff')
                        self.end_headers()
                        self.wfile.write(payload)
                else:
                    self.send_error(404)
            except FileNotFoundError:
                self.json_response(404, {'error': '版本或证据文件不存在'})
            except (ValueError, KeyError, TypeError) as error:
                self.json_response(400, {'error': str(error)})
            except Exception as error:
                self.json_response(503, {'error': str(error)})
            return
        if route == '/health':
            self.json_response(200, dict(product='SceneSem', version='0.2.2', workbench_baseline='v13+s1', project_schema='scenesem.project.v1', bundle_import=True, mask_category_correction=True,
                project_root=str(ROOT), host=HOST, port=PORT, spatial_context=True, zone_boundaries=True, business_context=True, source_materials=True, dataset_materials=True, bundle_materials=True, real_robot_commands=False))
            return
        if route == '/semantic/status':
            ready = all(importlib.util.find_spec(m) for m in ('torch', 'transformers', 'scipy'))
            instance_ready = ready and all((INSTANCE_MODEL/f).is_file() for f in ('config.json', 'preprocessor_config.json', 'model.safetensors'))
            self.json_response(200, dict(ready=ready, instance_ready=instance_ready, default_model=INSTANCE_MODEL_ID,
                license_scope='evaluation_pending_weights_license_review', backend='local_cpu', max_image_side=1024))
            return
        if route == '/semantic/tour':
            try:
                self.json_response(200, tour_positions(json.loads((SCENE / 'scene.json').read_text(encoding='utf-8'))))
            except Exception as error:
                self.json_response(503, {'error': str(error)})
            return
        mapped = {
            '/auto-labels.json': PROPOSALS,
            '/auto-labels-legacy.json': LEGACY_PROPOSALS,
            '/scene.json': SCENE / 'scene.json', '/points.bin': SCENE / 'points.bin',
            '/mesh-vertices.bin': SCENE / 'mesh-vertices.bin', '/mesh-indices.bin': SCENE / 'mesh-indices.bin',
            '/vendor/playcanvas.js': ENGINE / 'build/playcanvas.js',
            '/vendor/LICENSE': ENGINE / 'LICENSE',
            '/scene-low.ply': VISUALS['low'],
            '/scene-medium.ply': VISUALS['medium'],
            '/scene-full.ply': VISUALS['full'],
        }
        if route in mapped:
            file = mapped[route]
            if not file.is_file():
                self.send_error(404)
                return
            mime = 'application/json' if file.suffix == '.json' else 'text/javascript' if file.suffix == '.js' else 'application/octet-stream'
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(file.stat().st_size))
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                with file.open('rb') as f:
                    while chunk := f.read(1024 * 1024):
                        self.wfile.write(chunk)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
            return
        if route == '/data.json':
            payload, mime = self.server.point_data, 'application/json; charset=utf-8'
        elif route in ('/', '/index.html', '/legacy', '/app.js', '/core.js', '/scene.html', '/scene-app.js', '/boxes.js', '/object-core.js', '/instance-tools.js', '/completion-core.js', '/completion-ui.js', '/cleanup-core.js', '/cleanup-ui.js', '/support-depth.js', '/object-ui.js', '/initial-core.js', '/sample-scope.js', '/project-core.js', '/project-ui.js', '/workbench-ux.js', '/project.css', '/scene.css', '/workflow.css', '/workbench.html'):
            file = WEB / ('workbench.html' if route in ('/', '/index.html', '/scene.html') else 'index.html' if route == '/legacy' else route[1:])
            payload = file.read_bytes()
            mime = 'text/html; charset=utf-8' if file.suffix == '.html' else 'text/css; charset=utf-8' if file.suffix == '.css' else 'text/javascript; charset=utf-8'
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self' 'wasm-unsafe-eval'; worker-src 'self' blob:; style-src 'self' 'unsafe-inline'; connect-src 'self'; img-src 'self' blob: data:; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(payload)


if __name__ == '__main__':
    # Fixed port. Fail clearly when occupied; never silently choose a new port.
    check_inputs()
    with ThreadingHTTPServer((HOST, PORT), Handler) as server:
        server.point_data = json.dumps(make_data(), ensure_ascii=False, separators=(',', ':')).encode('utf-8')
        print('SceneSem: http://127.0.0.1:18876', flush=True)
        server.serve_forever()
