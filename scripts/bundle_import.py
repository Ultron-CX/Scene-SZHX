"""Local two-phase SceneSem ZIP restore; ZIP member paths are never extracted.

Staging is a private, disposable task cache, not the source scan directory.
Commit is versioned, preserves a checkpoint, and supports receipt-based retries.
"""
import base64
import copy
import hashlib
import json
import math
import re
import shutil
import stat
import uuid
import zipfile
from pathlib import Path
from project_store import ProjectStore, SHA, CATEGORIES, encoded, now
from semantic_dataset import material_inventory

MAX_UPLOAD = 512 * 1024 * 1024
MAX_EXPANDED = 1024 * 1024 * 1024
MAX_ENTRY = 64 * 1024 * 1024
MAX_FILES = 8192


def read_json(payload):
    def pairs(items):
        out = {}
        for key, value in items:
            if key in out:
                raise ValueError('JSON 字段重复：' + key)
            out[key] = value
        return out
    def invalid(value):
        raise ValueError('JSON 中包含无效数值：' + value)
    return json.loads(payload, object_pairs_hook=pairs, parse_constant=invalid)


def evidence_refs(value, depth=0):
    if depth > 64:
        raise ValueError('来源嵌套超过 64 层，请先整理来源历史')
    refs = set()
    if isinstance(value, dict):
        ids = value.get('evidence_ids', [])
        if 'evidence_id' in value:
            ids = [*ids, value['evidence_id']]
        if not isinstance(ids, list) or any(not isinstance(e, str) or not SHA.fullmatch(e) for e in ids):
            raise ValueError('存在无效证据引用')
        refs.update(ids)
        for k, child in value.items():
            if k not in ('geometry', 'cells', 'runs', 'flagged', 'positive', 'negative'):
                refs.update(evidence_refs(child, depth+1))
    elif isinstance(value, list):
        for child in value:
            if isinstance(child, (dict, list)):
                refs.update(evidence_refs(child, depth+1))
    return refs


class ImportCommitError(RuntimeError):
    def __init__(self, message, checkpoint_revision):
        super().__init__(message)
        self.checkpoint_revision = checkpoint_revision


class BundleImporter:
    def __init__(self, store, staging_root):
        self.store = store
        self.root = Path(staging_root).resolve()

    def folder(self, token):
        if not isinstance(token, str) or not re.fullmatch(r'[0-9a-f]{32}', token):
            raise ValueError('导入预检 ID 无效')
        path = (self.root / token).resolve()
        if path.parent != self.root:
            raise ValueError('导入预检路径越界')
        return path

    def preview(self, stream, length):
        if type(length) is not int or not 0 < length <= MAX_UPLOAD:
            raise ValueError('ZIP 文件须小于或等于 512 MiB')
        self.root.mkdir(parents=True, exist_ok=True)
        if shutil.disk_usage(self.root).free < length + MAX_ENTRY:
            raise OSError('本地临时目录空间不足')
        token = uuid.uuid4().hex
        folder = self.folder(token)
        folder.mkdir()
        hasher = hashlib.sha256()
        with (folder / 'upload.zip').open('xb') as out:
            remaining = length
            while remaining:
                chunk = stream.read(min(remaining, 1024*1024))
                if not chunk:
                    raise ValueError('ZIP 上传不完整，未修改项目')
                hasher.update(chunk)
                out.write(chunk)
                remaining -= len(chunk)
        try:
            with zipfile.ZipFile(folder / 'upload.zip') as archive:
                members = archive.infolist()
                if len(members) > MAX_FILES or sum(m.file_size for m in members) > MAX_EXPANDED:
                    raise ValueError('ZIP 文件数量或展开体积超限（最多 8192 项 / 1 GiB）')
                names = set()
                for info in members:
                    name = info.filename
                    if name in names:
                        raise ValueError('ZIP 内有重复路径')
                    names.add(name)
                    allowed = name in ('manifest.json', 'project.json') or re.fullmatch(r'(blobs|materials)/[0-9a-f]{64}', name)
                    if not allowed or info.is_dir() or '\\' in name or info.file_size > MAX_ENTRY:
                        raise ValueError('ZIP 含非 SceneSem 文件、非法路径或超大单文件：' + name[:100])
                    if info.flag_bits & 1 or stat.S_IFMT(info.external_attr >> 16) not in (0, stat.S_IFREG):
                        raise ValueError('不接受加密文件、链接或特殊文件')
                    if info.compress_type not in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED):
                        raise ValueError('仅支持标准 ZIP stored/deflate 压缩')
                if not {'project.json', 'manifest.json'} <= names:
                    raise ValueError('不是 SceneSem 项目包：缺少 project.json 或 manifest.json')
                manifest = read_json(archive.read('manifest.json'))
                if not isinstance(manifest, dict) or manifest.get('schema') not in ('scenesem.bundle.v1','scenesem.bundle.v2') or manifest.get('real_robot_commands') is not False or manifest.get('source_scans_included') is not False:
                    raise ValueError('项目包清单格式或使用边界不匹配')
                listed = manifest.get('blobs')
                if not isinstance(listed, list) or any(not isinstance(v, str) or not SHA.fullmatch(v) for v in listed) or len(set(listed)) != len(listed):
                    raise ValueError('项目包证据清单无效')
                portable=manifest['schema']=='scenesem.bundle.v2'
                material_hashes=sorted(n.split('/')[1] for n in names if n.startswith('materials/'))
                if not portable and (material_hashes or 'materials' in manifest or manifest.get('source_materials_included',False) is not False): raise ValueError('Materials in legacy bundle')
                if names != {'project.json', 'manifest.json', *('blobs/'+v for v in listed), *('materials/'+v for v in material_hashes)}:
                    raise ValueError('实际文件与清单不一致，存在缺失或额外内容')
                raw = archive.read('project.json')
                if hashlib.sha256(raw).hexdigest() != manifest.get('project_sha256'):
                    raise ValueError('project.json 摘要校验失败')
                doc = read_json(raw)
                if not isinstance(doc, dict) or doc.get('schema') != 'scenesem.project.v1' or doc.get('real_robot_commands') is not False or type(doc.get('revision')) is not int or not 0 < doc['revision'] < 100000000 or doc['revision'] != manifest.get('revision'):
                    raise ValueError('项目版本格式无效')
                self.store.scene_check(doc['annotations'])
                if shutil.disk_usage(folder).free < sum(m.file_size for m in members) + MAX_ENTRY:
                    raise OSError('临时目录空间不足以校验证据包')
                stage = ProjectStore(folder, self.store.scene)
                inventory=material_inventory(doc.get('business_context'),material_hashes)
                if portable and (manifest.get('materials')!=inventory or manifest.get('source_materials_included') is not bool(material_hashes)): raise ValueError('Material inventory mismatch')
                snapshots=[dict(sha256=e['sha256'],bytes=e['bytes'],name=e['names'][0]) for e in inventory['files'] if e['status']=='included']
                stage.check_material_capacity(snapshots)
                for s in snapshots:
                    raw=archive.read('materials/'+s['sha256'])
                    stage.archive_material(dict(scene_id=self.store.scene['scene_id'],source_hashes=self.store.scene['source_hashes'],snapshot=s,base64=base64.b64encode(raw).decode(),confirm_local_archive=True))
                capacity=self.store.check_material_capacity(snapshots) if snapshots else None
                availability=[]
                for e in inventory['files']:
                    s=dict(sha256=e['sha256'],bytes=e['bytes'],name=e['names'][0])
                    state='in_package' if e['status']=='included' else self.store.material_status(dict(scene_id=self.store.scene['scene_id'],source_hashes=self.store.scene['source_hashes'],snapshot=s))['status']
                    availability.append(dict(sha256=e['sha256'],names=e['names'],status=state))
                for sha in listed:
                    payload = archive.read('blobs/'+sha)
                    if hashlib.sha256(payload).hexdigest() != sha:
                        raise ValueError('证据内容摘要校验失败：' + sha[:12])
                    stage.put_blob(payload)
                needed = set(evidence_refs(doc))
                for eid in list(needed):
                    m = self.check_evidence(stage, eid)
                    needed.update(m[k] for k in ('rgb', 'depth', 'validity', 'result') if m.get(k))
                if needed != set(listed):
                    raise ValueError('证据引用与文件清单不一致（缺失引用或无引用内容）')
                objects = copy.deepcopy(doc['objects'])
                for o in objects:
                    if 'pending_review' in o:
                        raise ValueError('项目包含未提交审核，不能自动执行')
                    o['import_requires_review'] = True
                    o['imported_from'] = dict(bundle_sha256=hasher.hexdigest(), original_revision=doc['revision'])
                data = dict(expected_revision=0, annotations=doc['annotations'], objects=objects, spatial_context=doc.get('spatial_context'), business_context=doc.get('business_context'), reason='bundle_preview_only')
                normalized = stage.save(data, validate_only=True)['document']
                summary = dict(import_id=token, bundle_sha256=hasher.hexdigest(), source_revision=doc['revision'], scene_id=doc['annotations']['scene_id'],
                    regions=len(normalized['annotations']['boxes']), objects=len(objects), evidence=len(evidence_refs(doc)), blobs=len(listed),
                    missing_original_evidence=sum(o['original_evidence'] == 'missing' for o in normalized['objects']),
                    missing_region_links=sum(len(o['missing_region_ids']) for o in normalized['objects']),
                    review_policy='preserve_history_require_local_reconfirmation', expanded_bytes=sum(m.file_size for m in members),
                    material_restore_count=len(snapshots),material_availability=availability,material_capacity=capacity,
                    unbound_material_attribute_ids=inventory['unbound_attribute_ids'],
                    stage_created_at=now(), real_robot_commands=False)
                stage.atomic(folder / 'prepared.json', encoded(dict(summary=summary, document=normalized, blobs=listed, materials=inventory)))
                return dict(**summary, document=normalized)
        except (zipfile.BadZipFile, zipfile.LargeZipFile, EOFError, RecursionError) as error:
            raise ValueError('项目 ZIP 无效或损坏，未修改项目') from error

    def check_evidence(self, stage, eid):
        m = stage.evidence(eid)
        if m.get('kind') not in ('recognition', 'later_observation') or m.get('coordinate_frame') != self.store.scene['coordinate_frame'] or m.get('real_robot_commands') is not False:
            raise ValueError('证据类型、坐标或使用边界不匹配')
        if m.get('depth_encoding') != 'float32-le-linear-normalized-top-left' or m.get('validity_encoding') != 'uint8-top-left-0-or-1':
            raise ValueError('不支持此深度或有效域编码')
        rgb, raw = stage.blob(m['rgb']), stage.blob(m['depth'])
        prepared = stage.prepare_frame(dict(scene_id=m['scene_id'], source_hashes=m['source_hashes'], image=base64.b64encode(rgb).decode('ascii'),
            frame=dict(**m['camera'], width=m['width'], height=m['height'], depth_f32le=base64.b64encode(raw).decode('ascii'))))
        if stage.blob(m['validity']) != prepared[2] or m.get('valid_pixels') != sum(prepared[2]) or m.get('image_mime') != prepared[4]:
            raise ValueError('证据有效域或图像类型与实际内容不一致')
        if m['kind'] == 'later_observation' and m.get('result') is not None:
            raise ValueError('后补观察不能伪装成原始识别结果')
        if m['kind'] == 'recognition':
            result = read_json(stage.blob(m['result']))
            self.store.scene_check(result)
            if result.get('image_sha256') != m['rgb'] or result.get('width') != m['width'] or result.get('height') != m['height'] or not isinstance(result.get('components'), list) or len(result['components']) > 10000:
                raise ValueError('原始识别结果与 RGB 不匹配')
            total = 0
            for c in result['components']:
                if not isinstance(c, dict) or c.get('category') not in CATEGORIES or not isinstance(c.get('runs'), list):
                    raise ValueError('识别掩膜格式无效')
                score = c.get('score')
                if score is not None and (type(score) not in (float, int) or not math.isfinite(score) or not 0 <= score <= 1):
                    raise ValueError('识别置信度无效')
                total += len(c['runs'])
                if total > 1000000:
                    raise ValueError('识别掩膜超出导入限制')
                for run in c['runs']:
                    if not isinstance(run, list) or len(run) != 3 or not all(type(n) is int for n in run) or not (0 <= run[0] < m['height'] and 0 <= run[1] < run[2] <= m['width']):
                        raise ValueError('识别掩膜像素范围无效')
        return m

    def commit(self, data):
        token = data['import_id']
        folder = self.folder(token)
        with self.store.lock:
            prepared = read_json((folder / 'prepared.json').read_bytes())
            included=[e for e in prepared.get('materials',{}).get('files',[]) if e['status']=='included']
            if included and data.get('confirm_material_restore') is not True: raise ValueError('请明确确认将包内材料恢复到本机')
            basis = {k: data['checkpoint'][k] for k in ('annotations', 'objects')}
            if data['checkpoint'].get('business_context') is not None:
                basis['business_context'] = data['checkpoint']['business_context']
            if data['checkpoint'].get('spatial_context') is not None:
                basis['spatial_context'] = data['checkpoint']['spatial_context']
            request_sha = hashlib.sha256(encoded(basis)).hexdigest()
            request_file = folder / 'request.json'
            if request_file.exists() and read_json(request_file.read_bytes())['checkpoint_sha256'] != request_sha:
                raise RuntimeError('预检确认后的工作台已变化；请重新选择项目包，避免丢失新修改')
            # If the response was lost, return the same revision instead of importing twice.
            head_info = self.store.load()
            marker = 'bundle_import:' + token + ':'
            if head_info['document'] and head_info['document']['reason'].startswith(marker):
                return dict(**head_info, import_id=token, checkpoint_revision=head_info['document']['parent_revision'], already_committed=True)
            if (folder / 'receipt.json').exists():
                raise RuntimeError('此包已导入且项目已有后续版本；请载入已有版本，不重复导入')
            recovering = bool(head_info['document'] and head_info['document']['reason'] == 'checkpoint_before_bundle_import:' + token and data.get('expected_revision') == head_info['document']['parent_revision'])
            if type(data.get('expected_revision')) is not int or (data['expected_revision'] != head_info['head'] and not recovering):
                raise RuntimeError('项目已有新版本，请先备份并载入最新项目，再重新确认导入')
            stage = ProjectStore(folder, self.store.scene)
            doc = prepared['document']
            if 'materials' in prepared and prepared['materials']!=material_inventory(doc.get('business_context'),[e['sha256'] for e in included]): raise ValueError('Staged material inventory changed')
            snapshots=[dict(sha256=e['sha256'],bytes=e['bytes'],name=e['names'][0]) for e in included]
            for s in snapshots:
                if len(stage.material_payload(s['sha256']))!=s['bytes']: raise ValueError('Staged material size mismatch')
            if snapshots:self.store.check_material_capacity(snapshots)
            # Recheck staged data and target checkpoint before any target version is written.
            import_data = dict(expected_revision=0, annotations=doc['annotations'], objects=doc['objects'], spatial_context=doc.get('spatial_context'), business_context=doc.get('business_context'), reason=marker+prepared['summary']['bundle_sha256'])
            stage.save(import_data, validate_only=True)
            checkpoint = copy.deepcopy(data['checkpoint'])
            checkpoint['expected_revision'] = head_info['head']
            checkpoint['reason'] = 'checkpoint_before_bundle_import:' + token
            checkpoint_normalized = self.store.save(checkpoint, validate_only=True)['document']
            if not request_file.exists():
                stage.atomic(request_file, encoded(dict(checkpoint_sha256=request_sha)))
            for sha in prepared['blobs']:
                self.store.put_blob(stage.blob(sha))
            try:
                for s in snapshots:
                    self.store.archive_material(dict(scene_id=self.store.scene['scene_id'],source_hashes=self.store.scene['source_hashes'],snapshot=s,base64=base64.b64encode(stage.material_payload(s['sha256'])).decode(),confirm_local_archive=True))
            except Exception as error:
                raise RuntimeError('材料恢复未完成；可能已有文件存入本机，未自动删除。请核对后重试。'+str(error)) from error
            # Reuse a checkpoint after a failed final write; never accumulate duplicates.
            if head_info['document'] and head_info['document']['reason'] == checkpoint['reason']:
                if head_info['document']['annotations'] != checkpoint_normalized['annotations'] or head_info['document']['objects'] != checkpoint_normalized['objects'] or head_info['document'].get('spatial_context') != checkpoint_normalized.get('spatial_context') or head_info['document'].get('business_context') != checkpoint_normalized.get('business_context'):
                    raise RuntimeError('检查点之后的工作台已变化；请重新预检后导入')
                checkpoint_revision = head_info['head']
            else:
                checkpoint_revision = self.store.save(checkpoint)['head']
            import_data['expected_revision'] = checkpoint_revision
            try:
                result = self.store.save(import_data)
            except Exception as error:
                raise ImportCommitError('导入未完成，当前标签未切换；已保留导入前检查点，请载入检查点核对后重试。' + str(error), checkpoint_revision) from error
            # Version reason is the authoritative receipt if this auxiliary write fails.
            try:
                stage.atomic(folder / 'receipt.json', encoded(dict(revision=result['head'], checkpoint_revision=checkpoint_revision)))
            except OSError:
                pass
            return dict(**result, import_id=token, checkpoint_revision=checkpoint_revision, already_committed=False)
