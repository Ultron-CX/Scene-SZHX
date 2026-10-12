"""S1 local, immutable JSON revisions and content-addressed observation evidence.

Single local server process; a stale tab must reload explicitly after a conflict.
No deletion API, original scans remain outside this store.
"""
import base64
import hashlib
import io
import json
import os
import re
import threading
import zipfile
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
from PIL import Image
from region_validation import geometry_valid

from semantic_dataset import validate_spatial, validate_business, validate_source_snapshot, material_inventory

DIMENSIONS = ('category', 'identity', 'coverage', 'background', 'evidence', 'coordinate')
SHA = re.compile(r'^[0-9a-f]{64}$')
MATERIAL_QUOTA = 256 * 1024 * 1024
CATEGORIES = set('wall floor ceiling door window counter chair device sign other table cabinet sofa shelf plant computer screen lamp picture book bottle keyboard mouse cup'.split())


def encoded(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')


def digest(value):
    return hashlib.sha256(encoded(value)).hexdigest()


def now():
    return datetime.now(timezone.utc).isoformat()


class ProjectStore:
    def __init__(self, root, scene):
        self.root = Path(root)
        self.scene = scene
        self.lock = threading.RLock()

    def scene_check(self, value):
        for field in ('scene_id', 'source_hashes'):
            if value.get(field) != self.scene[field]:
                raise ValueError('项目与当前场景批次不一致：' + field)

    def put_blob(self, payload):
        sha = hashlib.sha256(payload).hexdigest()
        folder = self.root / 'blobs'
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / sha
        with self.lock:
            if path.exists():
                if hashlib.sha256(path.read_bytes()).hexdigest() != sha:
                    raise RuntimeError('已有证据文件损坏，停止保存')
            else:
                self.atomic(path, payload)
        return sha

    @staticmethod
    def atomic(path, payload):
        # Only known store paths; replace a unique staging file, never a scan.
        import uuid
        temp = path.with_name('.' + path.name + '.' + uuid.uuid4().hex + '.tmp')
        with temp.open('xb') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)

    def blob(self, sha):
        if not isinstance(sha, str) or not SHA.fullmatch(sha):
            raise ValueError('证据 ID 无效')
        payload = (self.root / 'blobs' / sha).read_bytes()
        if hashlib.sha256(payload).hexdigest() != sha:
            raise RuntimeError('证据哈希校验失败')
        return payload

    def material_payload(self, sha):
        if not isinstance(sha,str) or not SHA.fullmatch(sha): raise ValueError('Invalid material hash')
        path=self.root/'source-materials'/sha
        if not 1<=path.stat().st_size<=8*1024*1024: raise RuntimeError('Material size invalid')
        raw=path.read_bytes()
        if hashlib.sha256(raw).hexdigest()!=sha: raise RuntimeError('Material integrity failed')
        return raw

    def check_material_capacity(self, snapshots):
        """Read-only preflight; callers hold lock through subsequent writes."""
        with self.lock:
            folder=self.root/'source-materials';seen={};additional=0
            used=sum(p.stat().st_size for p in folder.iterdir() if p.is_file()) if folder.exists() else 0
            for s in snapshots:
                validate_source_snapshot(s)
                if s['sha256'] in seen:
                    if seen[s['sha256']]!=s['bytes']: raise ValueError('Conflicting material sizes')
                    continue
                seen[s['sha256']]=s['bytes']
                if (folder/s['sha256']).exists():
                    if len(self.material_payload(s['sha256']))!=s['bytes']: raise RuntimeError('Existing material size mismatch; not overwritten')
                else: additional+=s['bytes']
            if used+additional>MATERIAL_QUOTA: raise ValueError('Local material quota 256 MiB exceeded; no automatic deletion')
            return dict(existing_bytes=used,additional_bytes=additional,limit_bytes=MATERIAL_QUOTA)

    def archive_material(self, data):
        self.scene_check(data)
        if data.get('confirm_local_archive') is not True: raise ValueError('Explicit local archive confirmation required')
        s=validate_source_snapshot(data['snapshot'])
        encoded_data=data.get('base64')
        if not isinstance(encoded_data,str) or len(encoded_data)>12*1024*1024: raise ValueError('Material upload too large')
        raw=base64.b64decode(encoded_data,validate=True)
        if len(raw)!=s['bytes'] or hashlib.sha256(raw).hexdigest()!=s['sha256']: raise ValueError('Material bytes/hash mismatch')
        folder=self.root/'source-materials';path=folder/s['sha256']
        with self.lock:
            self.check_material_capacity([s])
            folder.mkdir(parents=True,exist_ok=True)
            if path.exists():self.material_payload(s['sha256'])
            else:
                self.atomic(path,raw)
        return dict(snapshot=s,status='integrity_verified',content_verified=False,project_saved=False)

    def material_status(self,data):
        self.scene_check(data);s=validate_source_snapshot(data['snapshot'])
        try:
            raw=self.material_payload(s['sha256']);status='integrity_verified' if len(raw)==s['bytes'] else 'invalid'
        except FileNotFoundError:status='missing'
        except RuntimeError:status='invalid'
        return dict(snapshot=s,status=status,content_verified=False)

    def prepare_frame(self, data):
        self.scene_check(data)
        f = data['frame']
        w, h = f['width'], f['height']
        if type(w) is not int or type(h) is not int or not (64 <= w <= 1024 and 64 <= h <= 1024):
            raise ValueError('证据图像尺寸无效')
        rgb = base64.b64decode(data['image'], validate=True)
        if len(rgb) > 3 * 1024 * 1024:
            raise ValueError('RGB 过大')
        with Image.open(io.BytesIO(rgb)) as im:
            if im.size != (w, h) or im.format not in ('JPEG', 'PNG'):
                raise ValueError('RGB 与证据尺寸不一致')
            im.load()
            mime = 'image/jpeg' if im.format == 'JPEG' else 'image/png'
        raw = base64.b64decode(f['depth_f32le'], validate=True)
        if len(raw) != w * h * 4:
            raise ValueError('深度字节数不匹配')
        depth = np.frombuffer(raw, dtype='<f4')
        valid = np.isfinite(depth) & (depth >= 0) & (depth < 1)
        camera = {k: f[k] for k in ('inverse', 'projection', 'camera_position', 'world_matrix', 'near', 'far', 'fov', 'horizontal_fov', 'pose_id')}
        for field, size in (('inverse', 16), ('projection', 16), ('world_matrix', 16), ('camera_position', 3)):
            if not isinstance(camera[field], list) or len(camera[field]) != size or not all(type(v) in (int, float) and np.isfinite(v) for v in camera[field]):
                raise ValueError('相机矩阵无效：' + field)
        if not (0 < camera['near'] < camera['far'] and 0 < camera['fov'] < 180) or type(camera['horizontal_fov']) is not bool:
            raise ValueError('相机投影参数无效')
        if not isinstance(camera['pose_id'], str) or len(camera['pose_id']) > 1024:
            raise ValueError('视点 ID 无效')
        encoded(camera)
        # Stored as actually captured, including invalid samples; validity is separate.
        return rgb, raw, valid.astype('u1').tobytes(), camera, mime, w, h

    def archive(self, data, result=None, prepared=None):
        rgb, raw, valid, camera, mime, w, h = prepared or self.prepare_frame(data)
        sha = hashlib.sha256(rgb).hexdigest()
        if result is not None and (result.get('image_sha256') != sha or result.get('width') != w or result.get('height') != h):
            raise ValueError('识别结果与归档帧不一致')
        manifest = dict(schema='scenesem.evidence.v1', scene_id=self.scene['scene_id'], source_hashes=self.scene['source_hashes'],
            coordinate_frame=self.scene['coordinate_frame'], kind='recognition' if result is not None else 'later_observation',
            capture_scope='annotation_free_camera_not_robot_sensor', depth_source='playcanvas_gs_alpha_depth_not_metric_ground_truth',
            depth_encoding='float32-le-linear-normalized-top-left', validity_encoding='uint8-top-left-0-or-1',
            width=w, height=h, valid_pixels=sum(valid), camera=camera, image_mime=mime,
            rgb=self.put_blob(rgb), depth=self.put_blob(raw), validity=self.put_blob(valid),
            result=self.put_blob(encoded(result)) if result is not None else None,
            inference_request={k: data[k] for k in ('backend', 'detail', 'confidence', 'target_roi') if k in data},
            original_missing_not_reconstructed=result is None, real_robot_commands=False)
        # Timestamp lives in revision references, not in the deduplicated identity.
        return self.put_blob(encoded(manifest))

    def evidence(self, evidence_id):
        manifest = json.loads(self.blob(evidence_id))
        if manifest.get('schema') != 'scenesem.evidence.v1':
            raise ValueError('不是证据清单')
        self.scene_check(manifest)
        return manifest

    def evidence_asset(self, evidence_id, asset):
        m = self.evidence(evidence_id)
        if asset not in ('rgb', 'depth', 'validity', 'result') or not m.get(asset):
            raise FileNotFoundError('证据内容不存在')
        return self.blob(m[asset]), m['image_mime'] if asset == 'rgb' else 'application/json' if asset == 'result' else 'application/octet-stream'

    def revisions(self):
        folder = self.root / 'revisions'
        return sorted(int(p.stem) for p in folder.glob('*.json') if re.fullmatch(r'[0-9]{8}', p.stem)) if folder.exists() else []

    def load(self, revision=None):
        ids = self.revisions()
        head = ids[-1] if ids else 0
        if revision is None and not head:
            return dict(head=0, document=None)
        revision = head if revision is None else revision
        if type(revision) is not int or revision not in ids:
            raise FileNotFoundError('项目版本不存在')
        document = json.loads((self.root / 'revisions' / f'{revision:08d}.json').read_bytes())
        self.scene_check(document['annotations'])
        return dict(head=head, document=document)

    def export(self, revision, stream, material_hashes=None):
        """Portable evidence bundle; includes no source scans or model weights."""
        doc = self.load(revision)['document']
        refs = set()

        def collect(v):
            if isinstance(v, dict):
                if isinstance(v.get('evidence_id'), str):
                    refs.add(v['evidence_id'])
                refs.update(v.get('evidence_ids', []))
                for k, x in v.items():
                    if k not in ('geometry', 'cells', 'runs', 'flagged', 'positive', 'negative'):
                        collect(x)
            elif isinstance(v, list):
                for x in v:
                    if isinstance(x, (dict, list)):
                        collect(x)

        collect(doc)
        blobs = set(refs)
        for eid in refs:
            m = self.evidence(eid)
            blobs.update(m[k] for k in ('rgb', 'depth', 'validity', 'result') if m.get(k))
        materials={};inventory=None
        if material_hashes is not None:
            if not isinstance(material_hashes,list) or len(material_hashes)>2000 or any(not isinstance(h,str) or not SHA.fullmatch(h) for h in material_hashes) or len(set(material_hashes))!=len(material_hashes): raise ValueError('Invalid material selection')
            inventory=material_inventory(doc.get('business_context'),material_hashes)
            if sum(e['bytes'] for e in inventory['files'] if e['status']=='included')>MATERIAL_QUOTA: raise ValueError('Material export exceeds 256 MiB')
            for e in inventory['files']:
                if e['status']=='included':
                    raw=self.material_payload(e['sha256'])
                    if len(raw)!=e['bytes']: raise ValueError('Material size mismatch')
                    materials[e['sha256']]=raw
        with zipfile.ZipFile(stream, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
            project = encoded(doc)
            archive.writestr('project.json', project)
            for sha in sorted(blobs):
                archive.writestr('blobs/' + sha, self.blob(sha))
            for h,raw in sorted(materials.items()):archive.writestr('materials/'+h,raw)
            manifest=dict(schema='scenesem.bundle.v2' if inventory is not None else 'scenesem.bundle.v1', revision=revision,
                project_sha256=hashlib.sha256(project).hexdigest(), blobs=sorted(blobs), source_scans_included=False, source_materials_included=bool(materials),
                private_local_evidence=True, real_robot_commands=False)
            if inventory is not None:manifest['materials']=inventory
            archive.writestr('manifest.json', encoded(manifest))

    def save(self, data, *, validate_only=False):
        with self.lock:
            head = self.load()['head']
            if type(data.get('expected_revision')) is not int or data['expected_revision'] != head:
                raise RuntimeError('项目已有新版本；请先备份当前草稿，再显式载入最新版本，不能覆盖')
            if 'spatial_context' not in data and head and self.load()['document'].get('spatial_context') is not None:
                raise ValueError('旧客户端未携带区域层级，已阻止丢失；请保留草稿并刷新升级')
            spatial = validate_spatial(data.get('spatial_context'))
            if 'business_context' not in data and head and self.load()['document'].get('business_context') is not None:
                raise ValueError('旧客户端遗漏业务属性，已阻止丢失，请升级')
            business = validate_business(data.get('business_context'))
            a = data['annotations']
            self.scene_check(a)
            if a.get('schema') != 'digit.regions.v5' or a.get('real_robot_commands') is not False or a.get('coordinate_frame') != self.scene['coordinate_frame'] or a.get('T_lcc_from_las') != self.scene['T_lcc_from_las']:
                raise ValueError('区域格式或坐标契约不匹配')
            regions = a['boxes']
            if not isinstance(regions, list) or len(regions) > 5000:
                raise ValueError('区域数量无效')
            by_id, numbers = {}, set()
            for r in regions:
                if not isinstance(r.get('id'), str) or not r['id'] or len(r['id']) > 200 or r['id'] in by_id:
                    raise ValueError('区域 ID 缺失或重复')
                if not isinstance(r.get('category'), str) or not all(isinstance(r.get(k), list) and len(r[k]) == 3 and all(type(n) in (int, float) and np.isfinite(n) for n in r[k]) for k in ('min', 'max')) or any(r['min'][i] > r['max'][i] for i in range(3)):
                    raise ValueError('区域范围无效')
                if r['category'] not in CATEGORIES or type(r.get('sequence')) is not int or not 0 < r['sequence'] < 1000000 or r.get('status') not in ('draft', 'proposed', 'reviewed') or len(r['id']) >= 100 or any(r['max'][i] - r['min'][i] < .0099 for i in range(3)):
                    raise ValueError('区域类别、序号、状态或尺寸无效')
                number = (r['category'], r['sequence'])
                if number in numbers:
                    raise ValueError('同一类别的区域序号重复')
                numbers.add(number)
                g = r.get('geometry')
                if g is not None and (not isinstance(g, dict) or g.get('type') not in ('surface-voxels', 'surface-mask', 'prism')):
                    raise ValueError('未知区域几何类型')
                if not geometry_valid(r):
                    raise ValueError('区域几何、边界或支持点数据无效')
                by_id[r['id']] = r
            objects = json.loads(encoded(data.get('objects', [])))
            if not isinstance(objects, list) or len(objects) > 5000:
                raise ValueError('对象数量无效')
            ids, owners = set(), set()
            for o in objects:
                if not isinstance(o, dict):
                    raise ValueError('对象档案必须是记录')
                o.setdefault('alias', '')
                o.setdefault('evidence_ids', [])
                o.setdefault('reviews', [])
                oid = o.get('object_id', '')
                if not isinstance(oid, str) or not re.fullmatch(r'obj-[0-9a-f-]{36}', oid) or oid in ids:
                    raise ValueError('对象 ID 无效或重复')
                ids.add(oid)
                if not isinstance(o.get('alias', ''), str) or len(o.get('alias', '')) > 100:
                    raise ValueError('对象别名无效')
                refs = o.get('region_ids')
                if not isinstance(refs, list) or not refs or not all(isinstance(r, str) for r in refs) or len(set(refs)) != len(refs) or owners.intersection(refs):
                    raise ValueError('区域不能重复归属；多个片段需显式关联')
                owners.update(refs)
                evidence_ids = o.get('evidence_ids', [])
                if not isinstance(evidence_ids, list) or len(evidence_ids) > 500 or len(set(evidence_ids)) != len(evidence_ids):
                    raise ValueError('证据引用无效')
                evidence = [self.evidence(eid) for eid in evidence_ids]
                # Check all referenced blobs, not just the manifest.
                for m in evidence:
                    for k in ('rgb', 'depth', 'validity', 'result'):
                        if m.get(k):
                            self.blob(m[k])
                corrections = o.get('mask_corrections', [])
                if not isinstance(corrections, list) or len(corrections) > 500:
                    raise ValueError('掩膜纠正历史超限或格式无效')
                result_cache = {}
                def check_correction(c):
                    if not isinstance(c, dict) or c.get('evidence_id') not in evidence_ids:
                        raise ValueError('掩膜纠正必须引用当前对象的归档证据')
                    eid = c['evidence_id']
                    if eid not in result_cache:
                        m = self.evidence(eid)
                        if m.get('kind') != 'recognition' or not m.get('result'):
                            raise ValueError('后补观察没有可纠正的原始识别掩膜')
                        result_cache[eid] = json.loads(self.blob(m['result']))['components']
                    index = c.get('component_index')
                    if type(index) is not int or not 0 <= index < len(result_cache[eid]):
                        raise ValueError('掩膜索引无效')
                    if 'category' not in c or c['category'] is not None and (not isinstance(c['category'], str) or c['category'] not in CATEGORIES):
                        raise ValueError('掩膜纠正类别无效')
                    if any(not isinstance(c.get(k), str) or not c[k].strip() or len(c[k]) > limit for k, limit in [('reviewer', 100), ('notes', 4000)]):
                        raise ValueError('请填写掩膜纠正人和依据')
                    return result_cache[eid][index]
                for correction in corrections:
                    component = check_correction(correction)
                    if correction.get('raw_category') != component.get('category') or correction.get('scope') != 'evidence_mask_only_not_region_or_identity' or type(correction.get('revision')) is not int or not isinstance(correction.get('at'), str):
                        raise ValueError('掩膜纠正来源或历史格式无效')
                pending_correction = o.pop('pending_mask_correction', None)
                if pending_correction is not None:
                    if len(corrections) >= 500:
                        raise ValueError('掩膜纠正历史超限')
                    if o.get('import_requires_review'):
                        raise ValueError('导入档案需先完成本机分项复核，再补充掩膜纠正')
                    component = check_correction(pending_correction)
                    corrections.append(dict(**{k: pending_correction[k] for k in ('evidence_id', 'component_index', 'category', 'reviewer', 'notes')},
                        raw_category=component.get('category'), scope='evidence_mask_only_not_region_or_identity', revision=head+1, at=now()))
                    o['mask_corrections'] = corrections
                basis_data = dict(regions=[by_id.get(r) for r in refs], evidence_ids=evidence_ids, alias=o.get('alias', ''))
                if corrections:
                    basis_data['mask_corrections'] = corrections
                basis = digest(basis_data)
                pending = o.pop('pending_review', None)
                if pending is not None:
                    if not isinstance(pending.get('reviewer'), str) or not pending['reviewer'].strip() or len(pending['reviewer']) > 100 or not isinstance(pending.get('notes'), str) or len(pending['notes']) > 4000:
                        raise ValueError('请填写审核人及有效备注')
                    checks = pending.get('checks', {})
                    if set(checks) != set(DIMENSIONS) or any(v not in ('pending', 'pass', 'issue', 'unknown') for v in checks.values()):
                        raise ValueError('必须分别记录六项审核状态')
                    if checks['evidence'] == 'pass' and not evidence:
                        raise ValueError('没有归档证据，不能标为证据通过')
                    scope = pending.get('coverage_scope', 'unknown')
                    if scope not in ('visible_inspected', 'partial', 'occluded', 'unknown'):
                        raise ValueError('覆盖范围无效')
                    if checks['coverage'] == 'pass' and (scope == 'unknown' or not pending['notes'].strip()):
                        raise ValueError('覆盖通过必须说明核查范围，不能默认为整件物体完整')
                    if any(r not in by_id for r in refs):
                        raise ValueError('关联区域已失效，请先修复关联再审核')
                    o.setdefault('reviews', []).append(dict(**pending, basis_sha256=basis, revision=head + 1, at=now()))
                    o['import_requires_review'] = False
                reviews = o.get('reviews', [])
                if not isinstance(reviews, list) or len(reviews) > 500:
                    raise ValueError('审核历史超出限制')
                for review in reviews:
                    if not isinstance(review, dict) or not isinstance(review.get('reviewer'), str) or not isinstance(review.get('checks'), dict) or set(review['checks']) != set(DIMENSIONS) or any(v not in ('pending', 'pass', 'issue', 'unknown') for v in review['checks'].values()):
                        raise ValueError('审核历史格式无效')
                o['basis_sha256'] = basis
                o['review_stale'] = bool(reviews and (reviews[-1].get('basis_sha256') != basis or o.get('import_requires_review', False)))
                o['identity_state'] = 'human_reviewed_same_object' if reviews and not o['review_stale'] and reviews[-1]['checks']['identity'] == 'pass' else 'candidate_not_human_confirmed'
                o['missing_region_ids'] = [r for r in refs if r not in by_id]
                o['original_evidence'] = 'archived' if any(m['kind'] == 'recognition' for m in evidence) else 'missing'
                o['task_qualification'] = False
            previous = {o['object_id']: o for o in self.load()['document']['objects']} if head else {}
            events = []
            for o in objects:
                before = previous.get(o['object_id'])
                if before is None or before['region_ids'] != o['region_ids'] or before.get('alias', '') != o.get('alias', ''):
                    events.append(dict(object_id=o['object_id'], kind='registered' if before is None else 'links_or_alias_changed',
                        before_region_ids=before['region_ids'] if before else [], after_region_ids=o['region_ids'], alias=o.get('alias', '')))
            for oid, old in previous.items():
                if oid not in ids:
                    events.append(dict(object_id=oid, kind='object_record_removed_not_regions', before_region_ids=old['region_ids'], after_region_ids=[]))
            doc = dict(schema='scenesem.project.v1', revision=head + 1, parent_revision=head, saved_at=now(), object_events=events,
                reason=str(data.get('reason', 'manual_save'))[:500], annotations=a, objects=objects, real_robot_commands=False)
            if spatial is not None:
                doc['spatial_context'] = json.loads(encoded(spatial))
            if business is not None:
                doc['business_context'] = json.loads(encoded(business))
            payload = encoded(doc)
            if len(payload) > 64 * 1024 * 1024:
                raise ValueError('项目超过 64 MB，请分批整理来源记录')
            if validate_only:
                return dict(head=head + 1, document=doc)
            folder = self.root / 'revisions'
            folder.mkdir(parents=True, exist_ok=True)
            self.atomic(folder / f'{head + 1:08d}.json', payload)
            return dict(head=head + 1, document=doc)
