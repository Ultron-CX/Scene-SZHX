"""Internal semantic dataset v1. Standard library only; no renderer/model required.

Integrity validation is not semantic accuracy or permission-to-publish approval.
"""
import hashlib
import json
import math
import re
import zipfile
from datetime import date
from collections import Counter

DIMENSIONS = ('category', 'identity', 'coverage', 'background', 'evidence', 'coordinate')
BASE_FILES = {'scene.json', 'taxonomy.json', 'regions.json', 'objects.json', 'quality.json', 'evidence.json'}
SHA = re.compile(r'^[0-9a-f]{64}$')


def validate_source_snapshot(s):
    if not isinstance(s,dict) or set(s)!={'sha256','bytes','name'} or not isinstance(s['sha256'],str) or not SHA.fullmatch(s['sha256']): raise ValueError('Invalid material snapshot')
    if type(s['bytes']) is not int or not 1<=s['bytes']<=8*1024*1024: raise ValueError('Material size limit: 8 MiB')
    if not isinstance(s['name'],str) or not s['name'].strip() or len(s['name'])>180 or re.search(r'[\x00-\x1f\x7f<>:"/\\|?*]',s['name']): raise ValueError('Invalid material filename')
    return s


def validate_business(value):
    if value is None:
        return None
    def need(ok):
        if not ok: raise ValueError('Invalid business context / 业务属性记录无效')
    need(isinstance(value, dict) and set(value) == {'schema', 'attributes'} and value['schema'] in ('scenesem.business-context.v1','scenesem.business-context.v2'))
    need(isinstance(value['attributes'], list) and len(value['attributes']) <= 2000)
    fields = set('attribute_id subject_kind subject_id name value basis source recorder observed_on valid_from valid_until verification'.split())
    seen = set()
    for a in value['attributes']:
        need(isinstance(a, dict) and (set(a) == fields or value['schema']=='scenesem.business-context.v2' and set(a)==fields|{'source_snapshot'}))
        if 'source_snapshot' in a: validate_source_snapshot(a['source_snapshot'])
        need(isinstance(a['attribute_id'], str) and re.fullmatch(r'attr-[0-9a-f-]{36}', a['attribute_id']) and a['attribute_id'] not in seen); seen.add(a['attribute_id'])
        need(a['subject_kind'] in ('object', 'zone'))
        need(isinstance(a['subject_id'], str) and re.fullmatch(('obj-' if a['subject_kind'] == 'object' else 'zone-') + r'[0-9a-f-]{36}', a['subject_id']))
        for key, limit in [('name',100),('value',2000),('source',1000),('recorder',100)]:
            need(isinstance(a[key], str) and len(a[key]) <= limit)
        need(bool(a['name'].strip()) and a['basis'] in ('unknown','simulation','field_report') and a['verification'] == 'unverified')
        for key in ('observed_on','valid_from','valid_until'):
            v = a[key]; need(v is None or isinstance(v, str) and re.fullmatch(r'[0-9]{4}-[0-9]{2}-[0-9]{2}',v))
            if v is not None: date.fromisoformat(v)
        need(not (a['valid_from'] and a['valid_until']) or a['valid_from'] <= a['valid_until'])
        if a['basis'] == 'unknown': need(a['value'] == '')
        else: need(bool(a['value'].strip()) and bool(a['source'].strip()))
        if a['basis'] == 'field_report': need(bool(a['recorder'].strip()) and a['observed_on'] is not None)
    return value


def check_business_at_date(value, objects, spatial, as_of, max_comparisons=10000):
    validate_business(value); validate_spatial(spatial)
    if not isinstance(as_of,str) or not re.fullmatch(r'[0-9]{4}-[0-9]{2}-[0-9]{2}',as_of): raise ValueError('Invalid check date')
    date.fromisoformat(as_of)
    if type(max_comparisons) is not int or not 1 <= max_comparisons <= 100000: raise ValueError('Invalid check budget')
    subjects={'object':{o['object_id'] for o in objects},'zone':{z['zone_id'] for z in (spatial or {}).get('zones',[])}}
    rows=[]; groups={}; names={}
    for a in (value or {}).get('attributes',[]):
        temporal='not_started' if a['valid_from'] and as_of<a['valid_from'] else 'expired' if a['valid_until'] and as_of>a['valid_until'] else 'within_declared_period' if a['valid_from'] and a['valid_until'] else 'incomplete_period'
        row=dict(attribute_id=a['attribute_id'],temporal=temporal,missing_subject=a['subject_id'] not in subjects[a['subject_kind']],unknown_value=a['basis']=='unknown',observed_after_date=bool(a['observed_on'] and a['observed_on']>as_of),mixed_basis=False,pair_notice_count=0)
        rows.append(row); name=(a['subject_kind'],a['subject_id'],a['name']);names.setdefault(name,[]).append((a,row))
        if temporal not in ('not_started','expired') and not row['observed_after_date'] and a['basis']!='unknown':groups.setdefault(name+(a['basis'],),[]).append((a,row))
    for group in names.values():
        if len({a['basis'] for a,_ in group})>1:
            for _,row in group:row['mixed_basis']=True
    total=sum(len(g)*(len(g)-1)//2 for g in groups.values());pairs=[]; comparisons=0
    for group in groups.values():
        for i,(a,ar) in enumerate(group):
            for b,br in group[i+1:]:
                if comparisons>=max_comparisons:break
                comparisons+=1;known=ar['temporal']==br['temporal']=='within_declared_period'
                kind='same_text' if a['value']==b['value'] else 'different_text' if known else 'possible_overlap_unknown_period'
                pairs.append(dict(a=a['attribute_id'],b=b['attribute_id'],kind=kind));ar['pair_notice_count']+=1;br['pair_notice_count']+=1
            if comparisons>=max_comparisons:break
        if comparisons>=max_comparisons:break
    return dict(schema='scenesem.business-date-check.v1',as_of=as_of,context_present=value is not None,scope='declared_dates_and_exact_text_not_business_truth',rows=rows,pairs=pairs,comparisons=comparisons,skipped_comparisons=total-comparisons,max_comparisons=max_comparisons,source_content_verified=False,task_qualification=False,real_robot_commands=False)


def business_quality(value, objects, spatial):
    validate_business(value)
    subjects = {'object': {o['object_id'] for o in objects}, 'zone': {z['zone_id'] for z in (spatial or {}).get('zones', [])}}
    groups = Counter((a['subject_kind'],a['subject_id'],a['name']) for a in value['attributes'])
    result = dict(scope='recorded_claims_not_business_truth', attribute_count=len(value['attributes']),
        missing_subject_attribute_ids=sorted(a['attribute_id'] for a in value['attributes'] if a['subject_id'] not in subjects[a['subject_kind']]),
        multiple_claim_attribute_ids=sorted(a['attribute_id'] for a in value['attributes'] if groups[(a['subject_kind'],a['subject_id'],a['name'])] > 1),
        basis_counts=dict(sorted(Counter(a['basis'] for a in value['attributes']).items())),
        source_documents_included=False, source_content_verified=False, current_applicability='not_evaluated', task_qualification=False)
    if value['schema']=='scenesem.business-context.v2':
        result.update(snapshot_reference_count=sum('source_snapshot' in a for a in value['attributes']),snapshot_bytes_included=False,snapshot_availability='requires_local_check')
    return result


def validate_spatial(value):
    """Hierarchy and optional reference bounds; never a physical containment certificate."""
    if value is None:
        return None
    def need(ok):
        if not ok:
            raise ValueError('Invalid spatial context / 区域层级或归属记录无效')
    need(isinstance(value, dict) and set(value) == {'schema', 'zones', 'memberships'})
    need(value['schema'] in ('scenesem.spatial-context.v1', 'scenesem.spatial-context.v2'))
    zones, members = value['zones'], value['memberships']
    need(isinstance(zones, list) and len(zones) <= 1000 and isinstance(members, list) and len(members) <= 5000)
    by_id = {}
    def annotation(r):
        need(r.get('source') in ('manual', 'automatic_candidate', 'unknown') and r.get('verification') == 'unverified')
        need(isinstance(r.get('notes'), str) and len(r['notes']) <= 1000)
    for z in zones:
        fields = {'zone_id', 'name', 'kind', 'parent_id', 'source', 'verification', 'notes'}
        need(isinstance(z, dict) and (set(z) == fields or value['schema'] == 'scenesem.spatial-context.v2' and set(z) == fields | {'boundary'}))
        if 'boundary' in z:
            b = z['boundary']
            need(isinstance(b, dict) and set(b) == {'type', 'min', 'max'} and b['type'] == 'aabb')
            need(all(isinstance(b[k], list) and len(b[k]) == 3 and all(type(v) in (int, float) and math.isfinite(v) and abs(v) < 100000 for v in b[k]) for k in ('min', 'max')))
            need(all(b['max'][i] - b['min'][i] >= .01 for i in range(3)))
        need(isinstance(z['zone_id'], str) and re.fullmatch(r'zone-[0-9a-f-]{36}', z['zone_id']) and z['zone_id'] not in by_id)
        need(isinstance(z['name'], str) and bool(z['name'].strip()) and len(z['name']) <= 100)
        need(z['kind'] in ('floor', 'room', 'functional_area'))
        need(z['parent_id'] is None or isinstance(z['parent_id'], str))
        annotation(z); by_id[z['zone_id']] = z
    for z in zones:
        seen = {z['zone_id']}; parent = z['parent_id']
        while parent is not None:
            need(parent in by_id and parent not in seen)
            seen.add(parent); parent = by_id[parent]['parent_id']
    owners = set()
    for m in members:
        need(isinstance(m, dict) and set(m) == {'object_id', 'zone_id', 'source', 'verification', 'notes'})
        need(isinstance(m['object_id'], str) and re.fullmatch(r'obj-[0-9a-f-]{36}', m['object_id']) and m['object_id'] not in owners)
        need(m['zone_id'] is None or isinstance(m['zone_id'], str) and m['zone_id'] in by_id)
        annotation(m); owners.add(m['object_id'])
    return value


def spatial_quality(value, objects):
    validate_spatial(value)
    ids = {o['object_id'] for o in objects}
    members = {m['object_id']: m for m in value['memberships']}
    result = dict(scope='logical_zones_not_geometric_containment', zone_count=len(value['zones']),
        unassigned_object_ids=sorted(i for i in ids if i not in members or members[i]['zone_id'] is None),
        missing_object_ids=sorted(set(members) - ids), verification='unverified', business_rules_status='not_implemented')
    if value['schema'] == 'scenesem.spatial-context.v2':
        result.update(scope='reference_bounds_not_physical_containment',
            bounded_zone_count=sum('boundary' in z for z in value['zones']),
            automatic_membership_currentness='not_revalidated', geometry_accuracy=None)
    return result


def encode(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def refs(value):
    found = set()
    def walk(v):
        if isinstance(v, dict):
            if isinstance(v.get('evidence_id'), str):
                found.add(v['evidence_id'])
            found.update(v.get('evidence_ids', []))
            for k, item in v.items():
                if k not in ('geometry', 'cells', 'runs', 'positive', 'negative', 'flagged'):
                    walk(item)
        elif isinstance(v, list):
            for item in v:
                walk(item)
    walk(value)
    if any(not SHA.fullmatch(e) for e in found):
        raise ValueError('Invalid evidence reference')
    return sorted(found)


def quality(regions, objects):
    by_id = {r['id']: r for r in regions}
    owners, rows = set(), []
    for o in objects:
        ids = o['region_ids']
        if owners.intersection(ids) or len(set(ids)) != len(ids):
            raise ValueError('Duplicate region ownership')
        owners.update(ids)
        missing = [r for r in ids if r not in by_id]
        basis = dict(regions=[by_id.get(r) for r in ids], evidence_ids=o['evidence_ids'], alias=o.get('alias', ''))
        if o.get('mask_corrections'):
            basis['mask_corrections'] = o['mask_corrections']
        last = o.get('reviews', [])[-1] if o.get('reviews') else None
        stale = bool(o.get('import_requires_review') or (last and (o.get('review_stale') or last.get('basis_sha256') != sha(encode(basis)))))
        checks = {d: last.get('checks', {}).get(d, 'pending') if last and not stale and not missing else 'pending' for d in DIMENSIONS}
        if any(v not in ('pending', 'pass', 'unknown', 'issue') for v in checks.values()):
            raise ValueError('Invalid review verdict')
        categories = sorted({by_id[r]['category'] for r in ids if r in by_id})
        status = 'missing' if missing else 'stale' if stale else 'unreviewed' if not last else 'issue' if 'issue' in checks.values() else 'unreviewed' if 'pending' in checks.values() else 'unknown' if 'unknown' in checks.values() else 'recorded'
        rows.append(dict(object_id=o['object_id'], status=status, checks=checks, missing_region_ids=missing,
                         categories=categories, category_conflict=len(categories) > 1,
                         evidence_ids=refs([o] + [by_id[r] for r in ids if r in by_id]), task_qualification=False))
    return dict(schema='scenesem.dataset-quality.v1', scope='record_consistency_not_accuracy',
                region_count=len(regions), object_record_count=len(objects), confirmed_physical_object_count=None,
                unregistered_region_ids=[r['id'] for r in regions if r['id'] not in owners],
                state_counts=dict(sorted(Counter(r['status'] for r in rows).items())), objects=rows,
                accuracy=None, completeness=None, sampling_protocol=None, business_rules_status='not_implemented',
                task_qualification=False, real_robot_commands=False)


def material_inventory(business, included):
    """Canonical referenced-file inventory, not a content/authority review."""
    validate_business(business)
    entries = {}
    unbound = []
    for a in (business or {}).get('attributes', []):
        s = a.get('source_snapshot')
        if s is None:
            unbound.append(a['attribute_id']); continue
        e = entries.setdefault(s['sha256'], dict(sha256=s['sha256'], bytes=s['bytes'], names=set(), attribute_ids=[]))
        if e['bytes'] != s['bytes']: raise ValueError('Conflicting sizes for the same material hash')
        e['names'].add(s['name']); e['attribute_ids'].append(a['attribute_id'])
    if not set(included) <= set(entries): raise ValueError('Selected material is not referenced by this revision')
    rows = [dict(e, names=sorted(e['names']), attribute_ids=sorted(e['attribute_ids']),
                 status='included' if h in included else 'not_included') for h,e in sorted(entries.items())]
    return dict(schema='scenesem.dataset-materials.v1', scope='referenced_snapshots_only_not_all_source_documents',
                files=rows, unbound_attribute_ids=sorted(unbound),
                all_referenced_snapshots_included=bool(rows) and all(e['status']=='included' for e in rows),
                source_content_verified=False, public_release_approved=False)


def material_quality(business_quality_value, inventory):
    q = dict(business_quality_value)
    included = sum(e['status']=='included' for e in inventory['files'])
    q.update(source_documents_included=bool(included), snapshot_bytes_included=bool(included),
             snapshot_availability='see_materials_inventory', included_snapshot_count=included,
             omitted_snapshot_count=len(inventory['files'])-included,
             all_referenced_snapshots_included=inventory['all_referenced_snapshots_included'])
    return q


def export_dataset(store, revision, stream, categories, material_hashes=None):
    doc = store.load(revision)['document']
    a = doc['annotations']
    if a['coordinate_frame'] != store.scene['coordinate_frame'] or a['T_lcc_from_las'] != store.scene['T_lcc_from_las']:
        raise ValueError('Scene coordinate contract mismatch')
    regions, objects = a['boxes'], doc['objects']
    scene = {k: a[k] for k in ('scene_id', 'source_hashes', 'coordinate_frame', 'T_lcc_from_las')}
    scene.update(schema='scenesem.dataset-scene.v1', revision=revision, project_sha256=sha(encode(doc)),
                 units='metres_as_declared_not_certified', alignment_status=store.scene.get('alignment_status', 'unknown'),
                 annotation_kind=a.get('annotation_kind', 'spatial_regions_not_instance_segmentation'),
                 source_assets_included=False, real_robot_commands=False)
    files = {k: encode(v) for k, v in {'scene.json': scene,
        'taxonomy.json': dict(schema='scenesem.dataset-taxonomy.v1', categories=sorted(categories), business_taxonomy=False),
        'regions.json': regions, 'objects.json': objects, 'quality.json': quality(regions, objects)}.items()}
    spatial = validate_spatial(doc.get('spatial_context'))
    if spatial is not None:
        files['spatial.json'] = encode(spatial)
        q = quality(regions, objects); q['spatial'] = spatial_quality(spatial, objects)
        files['quality.json'] = encode(q)
    business = validate_business(doc.get('business_context'))
    if business is not None:
        files['business.json'] = encode(business)
        q = json.loads(files['quality.json']); q['business'] = business_quality(business, objects, spatial)
        files['quality.json'] = encode(q)
    if material_hashes is not None:
        if business is None: raise ValueError('No business context for material export')
        if not isinstance(material_hashes,list) or len(material_hashes)>2000 or any(not isinstance(h,str) or not SHA.fullmatch(h) for h in material_hashes) or len(set(material_hashes))!=len(material_hashes):
            raise ValueError('Invalid material selection')
        inventory = material_inventory(business, material_hashes)
        if sum(e['bytes'] for e in inventory['files'] if e['status']=='included')>256*1024*1024:
            raise ValueError('Material selection exceeds 256 MiB')
        for e in inventory['files']:
            if e['status']=='included':
                raw=store.material_payload(e['sha256'])
                if len(raw)!=e['bytes']: raise ValueError('Material size mismatch: '+e['sha256'])
                files['materials/'+e['sha256']]=raw
        files['materials.json']=encode(inventory)
        q=json.loads(files['quality.json']);q['business']=material_quality(q['business'],inventory)
        files['quality.json']=encode(q)
    evidence = []
    for eid in refs([regions, objects]):
        entry = dict(evidence_id=eid, status='verified', files=[])
        try:
            m = store.evidence(eid)
            if m.get('coordinate_frame') != scene['coordinate_frame']:
                raise ValueError('Evidence coordinate mismatch')
            group = {eid: store.blob(eid)}
            for key in ('rgb', 'depth', 'validity', 'result'):
                if m.get(key):
                    group[m[key]] = store.blob(m[key])
            if any(not m.get(k) for k in ('rgb', 'depth', 'validity')) or (m.get('kind') == 'recognition' and not m.get('result')):
                raise ValueError('Incomplete evidence manifest')
            for name, raw in group.items():
                files['blobs/' + name] = raw
            entry['files'] = sorted('blobs/' + name for name in group)
        except (FileNotFoundError, ValueError, RuntimeError, KeyError, TypeError):
            entry['status'] = 'unavailable_or_invalid'
        evidence.append(entry)
    files['evidence.json'] = encode(evidence)
    manifest = dict(schema='scenesem.semantic-dataset.v4' if material_hashes is not None else 'scenesem.semantic-dataset.v3' if business is not None else 'scenesem.semantic-dataset.v2' if spatial is not None else 'scenesem.semantic-dataset.v1', stage='internal_draft', revision=revision,
        scene_id=scene['scene_id'], project_sha256=scene['project_sha256'], public_release_approved=False,
        source_assets_included=False, real_robot_commands=False,
        files={name: dict(sha256=sha(raw), bytes=len(raw)) for name, raw in sorted(files.items())})
    if sum(map(len,files.values()))+len(encode(manifest))>512*1024*1024: raise ValueError('Dataset exceeds 512 MiB reader limit')
    with zipfile.ZipFile(stream, 'w', compression=zipfile.ZIP_DEFLATED) as z:
        for name, raw in sorted(files.items()):
            z.writestr(name, raw)
        z.writestr('manifest.json', encode(manifest))
    return manifest


def read_dataset(source):
    """Read without extracting or accessing paths supplied by the archive."""
    def require(ok, message):
        if not ok:
            raise ValueError(message)
    with zipfile.ZipFile(source) as z:
        infos = z.infolist()
        names = [i.filename for i in infos]
        require(len(names) == len(set(names)) and len(names) <= 25000, 'Duplicate/excessive ZIP members')
        require(sum(i.file_size for i in infos) <= 512 * 1024 * 1024, 'Dataset exceeds 512 MiB reader limit')
        require(all(n in BASE_FILES | {'manifest.json', 'spatial.json', 'business.json', 'materials.json'} or re.fullmatch(r'(blobs|materials)/[0-9a-f]{64}', n) for n in names), 'Unexpected archive path')
        def pairs(items):
            out = {}
            for k, v in items:
                require(k not in out, 'Duplicate JSON key')
                out[k] = v
            return out
        def decode(raw):
            return json.loads(raw, object_pairs_hook=pairs, parse_constant=lambda x: (_ for _ in ()).throw(ValueError('Non-finite JSON')))
        m = decode(z.read('manifest.json'))
        require(m['schema'] in ('scenesem.semantic-dataset.v1', 'scenesem.semantic-dataset.v2', 'scenesem.semantic-dataset.v3','scenesem.semantic-dataset.v4') and m['stage'] == 'internal_draft', 'Unsupported dataset')
        portable = m['schema']=='scenesem.semantic-dataset.v4'
        require(('materials.json' in names)==portable, 'Materials schema/file mismatch')
        require(portable or not any(n.startswith('materials/') for n in names), 'Materials in legacy schema')
        require(('business.json' in names) == (m['schema'] in ('scenesem.semantic-dataset.v3','scenesem.semantic-dataset.v4')), 'Business schema/file mismatch')
        if m['schema'] not in ('scenesem.semantic-dataset.v3','scenesem.semantic-dataset.v4'):
            require(('spatial.json' in names) == (m['schema'] == 'scenesem.semantic-dataset.v2'), 'Spatial schema/file mismatch')
        require(m.get('public_release_approved') is False and m.get('real_robot_commands') is False and m.get('source_assets_included') is False, 'Invalid release scope')
        require(set(m['files']) == set(names) - {'manifest.json'} and BASE_FILES <= set(m['files']), 'File inventory mismatch')
        data = {}
        for name, spec in m['files'].items():
            raw = z.read(name)
            require(len(raw) == spec['bytes'] and sha(raw) == spec['sha256'], 'File integrity mismatch: ' + name)
            if name.startswith(('blobs/','materials/')):
                require(sha(raw) == name.split('/')[1], 'Invalid content address')
                if name.startswith('materials/'): require(1<=len(raw)<=8*1024*1024, 'Material size limit')
            else:
                data[name] = decode(raw)
        s, regions, objects, evidence = [data[n] for n in ('scene.json', 'regions.json', 'objects.json', 'evidence.json')]
        require(s['schema'] == 'scenesem.dataset-scene.v1' and s['scene_id'] == m['scene_id'] and s['revision'] == m['revision'] and s['project_sha256'] == m['project_sha256'], 'Scene/version mismatch')
        require(s.get('source_assets_included') is False and s.get('real_robot_commands') is False, 'Scene scope mismatch')
        require(isinstance(s['coordinate_frame'], str) and bool(s['coordinate_frame']) and isinstance(s['T_lcc_from_las'], list), 'Missing coordinate contract')
        require(isinstance(s['source_hashes'], dict) and all(isinstance(h, str) and SHA.fullmatch(h) for h in s['source_hashes'].values()), 'Invalid asset hashes')
        require(isinstance(regions, list) and isinstance(objects, list) and len(regions) <= 5000 and len(objects) <= 5000, 'Invalid record counts')
        require(all(isinstance(r.get('id'), str) and r['id'] for r in regions), 'Invalid region ID')
        require(all(isinstance(o.get('object_id'), str) and re.fullmatch(r'obj-[0-9a-f-]{36}', o['object_id']) and isinstance(o.get('region_ids'), list) and o['region_ids'] and isinstance(o.get('reviews'), list) and isinstance(o.get('evidence_ids'), list) for o in objects), 'Invalid object record')
        require(all(all(isinstance(r.get(k), list) and len(r[k]) == 3 and all(type(x) in (int, float) and math.isfinite(x) for x in r[k]) for k in ('min','max')) and all(r['min'][i] <= r['max'][i] for i in range(3)) for r in regions), 'Invalid region bounds')
        require(len({r['id'] for r in regions}) == len(regions) and len({o['object_id'] for o in objects}) == len(objects), 'Duplicate semantic IDs')
        taxonomy = data['taxonomy.json']
        require(taxonomy['schema'] == 'scenesem.dataset-taxonomy.v1' and len(set(taxonomy['categories'])) == len(taxonomy['categories']), 'Invalid taxonomy')
        require(all(r['category'] in taxonomy['categories'] for r in regions), 'Unknown category')
        require(all(o.get('task_qualification') is False for o in objects), 'Unvalidated task qualification')
        expected_quality = quality(regions, objects)
        if 'spatial.json' in data:
            require(data['spatial.json'] is not None, 'Missing spatial context')
            expected_quality['spatial'] = spatial_quality(data['spatial.json'], objects)
        if 'business.json' in data:
            require(data['business.json'] is not None, 'Missing business context')
            expected_quality['business'] = business_quality(data['business.json'], objects, data.get('spatial.json'))
        if portable:
            included = {n.split('/')[1] for n in names if n.startswith('materials/')}
            inventory = material_inventory(data['business.json'], included)
            require(data['materials.json']==inventory, 'Material inventory inconsistent with references/bytes')
            require(sum(m['files']['materials/'+h]['bytes'] for h in included)<=256*1024*1024, 'Materials exceed 256 MiB')
            require(all(e['status']!='included' or m['files']['materials/'+e['sha256']]['bytes']==e['bytes'] for e in inventory['files']), 'Material reference size mismatch')
            expected_quality['business']=material_quality(expected_quality['business'],inventory)
        require(data['quality.json'] == expected_quality, 'Quality report inconsistent with records')
        require(sorted(e['evidence_id'] for e in evidence) == refs([regions, objects]), 'Evidence reference inventory mismatch')
        referenced_blobs = set()
        for e in evidence:
            require(e['status'] in ('verified', 'unavailable_or_invalid'), 'Unknown evidence state')
            if e['status'] != 'verified':
                require(e['files'] == [], 'Unavailable evidence must not claim verified files')
                continue
            p = 'blobs/' + e['evidence_id']
            em = decode(z.read(p))
            require(em.get('schema') == 'scenesem.evidence.v1' and all(em.get(k) == s[k] for k in ('scene_id', 'source_hashes', 'coordinate_frame')), 'Evidence scene mismatch')
            require(all(em.get(k) for k in ('rgb', 'depth', 'validity')) and (em.get('kind') != 'recognition' or em.get('result')), 'Incomplete evidence')
            expected = {p} | {'blobs/' + em[k] for k in ('rgb', 'depth', 'validity', 'result') if em.get(k)}
            require(set(e['files']) == expected and expected <= set(m['files']), 'Evidence files mismatch')
            require(len(z.read('blobs/' + em['depth'])) == em['width'] * em['height'] * 4 and len(z.read('blobs/' + em['validity'])) == em['width'] * em['height'], 'Evidence dimensions mismatch')
            referenced_blobs.update(expected)
        require(referenced_blobs == {n for n in names if n.startswith('blobs/')}, 'Unreferenced blobs')
        return dict(manifest=m, **{n[:-5]: value for n, value in data.items()})


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Validate/read a SceneSem internal semantic dataset ZIP; no extraction or model required')
    parser.add_argument('dataset')
    parser.add_argument('--as-of', help='Read-only declared-date/text check, YYYY-MM-DD; not business certification')
    args = parser.parse_args()
    result = read_dataset(args.dataset)
    if args.as_of:
        print(json.dumps(check_business_at_date(result.get('business'),result['objects'],result.get('spatial'),args.as_of),ensure_ascii=False,indent=2))
        raise SystemExit(0)
    print(json.dumps(dict(valid=True, scene_id=result['scene']['scene_id'], revision=result['scene']['revision'],
        regions=len(result['regions']), object_records=len(result['objects']),
        unregistered_regions=len(result['quality']['unregistered_region_ids']), evidence_states=dict(Counter(e['status'] for e in result['evidence'])),
        materials=result.get('materials',dict(status='not_included_legacy_package',source_content_verified=False)),
        semantic_accuracy_validated=False, public_release_approved=False), ensure_ascii=False, indent=2))
