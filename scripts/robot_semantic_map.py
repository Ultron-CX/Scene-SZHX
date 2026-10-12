"""Read-only adapter from a validated SceneSem dataset to a robot-map draft.

No robot connection, embeddings, navigation graph or inferred online timestamps.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
from semantic_dataset import read_dataset, validate_spatial


def adapt(data, *, up_axis=None):
    if up_axis not in (None, 'z'):
        raise ValueError('Only explicitly declared Z-up projection is supported')
    scene = data['scene']
    fragments = {r['id']: r for r in data['regions']}
    if len(fragments) != len(data['regions']):
        raise ValueError('Duplicate source region IDs')
    spatial = validate_spatial(data.get('spatial')) or {'zones': [], 'memberships': []}
    object_ids = [o['object_id'] for o in data['objects']]
    if len(set(object_ids)) != len(object_ids):
        raise ValueError('Duplicate object IDs')
    if any(m['object_id'] not in object_ids for m in spatial['memberships']):
        raise ValueError('Dangling functional-zone membership')
    members = {m['object_id']: m for m in spatial['memberships']}
    objects, regions, used = [], [], set()
    for obj in data['objects']:
        ids = obj['region_ids']
        if not ids or len(set(ids)) != len(ids) or any(i not in fragments for i in ids):
            raise ValueError('Missing/duplicate object support references')
        if used.intersection(ids):
            raise ValueError('A source fragment belongs to multiple objects')
        used.update(ids)
        rows = [fragments[i] for i in ids]
        labels = sorted({r['category'] for r in rows})
        for row in rows:
            if any(len(row[k]) != 3 or any(type(v) not in (int, float) or not math.isfinite(v) for v in row[k]) for k in ('min', 'max')) or any(row['min'][i] > row['max'][i] for i in range(3)):
                raise ValueError('Invalid source bounds')
        low = [min(r['min'][i] for r in rows) for i in range(3)]
        high = [max(r['max'][i] for r in rows) for i in range(3)]
        membership = members.get(obj['object_id'])
        objects.append(dict(id=obj['object_id'], label=labels[0] if len(labels)==1 else None,
                            label_confidence=None, centroid=None,
                            bbox3d=dict(center=[(a+b)/2 for a,b in zip(low,high)], size=[b-a for a,b in zip(low,high)],
                                        type='aabb', frame=scene['coordinate_frame'], extent='union_of_annotation_bounds_not_full_object'),
                            embedding_ref=None, observation_count=None, last_seen=None,
                            region_id=membership['zone_id'] if membership else None,
                            source_region_ids=list(ids), evidence_ids=list(obj.get('evidence_ids', [])),
                            label_candidates=labels, membership_status='unverified' if membership else 'unknown',
                            unknown_fields=['label_confidence','centroid','embedding_ref','observation_count','last_seen']))
    zones = {z['zone_id']:z for z in spatial['zones']}
    for zone in spatial['zones']:
        boundary = zone.get('boundary')
        polygon, height = None, None
        if boundary and up_axis == 'z':
            lo, hi = boundary['min'], boundary['max']
            polygon = [[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]]
            height = [lo[2],hi[2]]
        ancestor, floor = zone, None
        while ancestor:
            if ancestor['kind']=='floor':
                floor=ancestor['zone_id']; break
            ancestor=zones.get(ancestor['parent_id'])
        regions.append(dict(id=zone['zone_id'], label=zone['name'], confidence=None,
                            kind=zone['kind'], parent_id=zone['parent_id'], floor_id=floor,
                            polygon2d=polygon, height_range=height, embedding_ref=None, places=[],
                            objects=[o['id'] for o in objects if o['region_id']==zone['zone_id']],
                            boundary_source=copy.deepcopy(boundary),
                            boundary_status='reference_aabb_projection' if polygon else 'not_projected',
                            verification='unverified'))
    return dict(schema='scenesem.robot-semantic-map.v1', adapter_version='1.0',
                scene=copy.deepcopy(scene), frame=dict(name=scene['coordinate_frame'], units=scene.get('units','unknown'),
                up_axis=up_axis, robot_map_id=None, T_robot_map_from_scene=None),
                objects=objects, places=[], regions=regions,
                limitations=dict(stage='offline_draft', robot_ready=False, semantic_accuracy_validated=False,
                embeddings='not_generated', places='not_generated_not_known_empty', online_observations='not_available',
                region_meaning='functional_zones_not_annotation_fragments', support_geometry='retained_in_source_dataset',
                unassigned_source_region_ids=sorted(set(fragments)-used)), real_robot_commands=False)


def convert(source, output, *, up_axis=None):
    source, output = Path(source).resolve(), Path(output).resolve()
    result = adapt(read_dataset(source), up_axis=up_axis)
    # Bind references to the exact original dataset; never embed private absolute paths.
    h = hashlib.sha256()
    with source.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''): h.update(chunk)
    result['source_dataset_sha256'] = h.hexdigest()
    encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False).encode('utf-8')
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as stream: stream.write(encoded)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description='Export a robot semantic-map draft, without changing the source dataset')
    parser.add_argument('dataset',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--up-axis',choices=['z'],help='Explicitly authorize XY projection of functional-zone bounds; no coordinate transform')
    args=parser.parse_args()
    result=convert(args.dataset,args.output,up_axis=args.up_axis)
    print(json.dumps(dict(output=str(args.output.resolve()),objects=len(result['objects']),regions=len(result['regions']),places=0,robot_ready=False),ensure_ascii=False))
