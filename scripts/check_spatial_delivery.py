"""Write a synthetic D4 consumer sample, never a production project revision."""
import io
import json
import tempfile
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path
from project_store import ProjectStore, CATEGORIES
from test_project_store import payload, SCENE, TMP
from test_spatial_context import fixture
from semantic_dataset import export_dataset, read_dataset, sha

root = Path(__file__).resolve().parents[1]
stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
with tempfile.TemporaryDirectory(dir=TMP) as folder:
    store = ProjectStore(Path(folder), SCENE)
    p = payload(count=3)
    spatial = fixture(p['objects'][0]['object_id'])
    spatial['zones'].append(dict(zone_id='zone-'+str(uuid.uuid4()), name='Synthetic service area — not a bank fact', kind='functional_area',
        parent_id=spatial['zones'][1]['zone_id'], source='manual', verification='unverified', notes='Contract demonstration only; no spatial boundary'))
    spatial['memberships'].append(dict(object_id=p['objects'][1]['object_id'], zone_id=spatial['zones'][2]['zone_id'], source='automatic_candidate', verification='unverified', notes='Synthetic candidate, no model inference'))
    bounded = '--boundaries' in sys.argv
    if bounded:
        spatial['schema'] = 'scenesem.spatial-context.v2'
        for z, bounds in zip(spatial['zones'], [([0, 0, 0], [10, 10, 3]), ([0, 0, 0], [5, 5, 3]), ([1, 1, 0], [3, 3, 2])]):
            z['boundary'] = dict(type='aabb', min=bounds[0], max=bounds[1])
            z['notes'] = 'Synthetic reference bounds only, not current K2 or bank geometry'
    p['spatial_context'] = spatial
    store.save(p)
    buf = io.BytesIO(); export_dataset(store, 1, buf, CATEGORIES)
    result = read_dataset(io.BytesIO(buf.getvalue()))
    assert result['spatial'] == spatial and len(result['quality']['spatial']['unassigned_object_ids']) == 1
    output = root/'artifacts/dataset-20260914'/(('spatial-bounds-synthetic-' if bounded else 'spatial-synthetic-')+stamp+'.zip')
    with output.open('xb') as f: f.write(buf.getvalue())
    report = dict(at=stamp, scope='synthetic_contract_not_current_K2_not_bank_truth', dataset=output.name, sha256=sha(buf.getvalue()),
        schema=result['manifest']['schema'], zones=len(spatial['zones']), object_records=len(result['objects']),
        spatial_quality=result['quality']['spatial'], production_modified=False, browser_gpu_tested=False, real_robot_commands=False)
    with output.with_suffix('.json').open('x', encoding='utf-8') as f: json.dump(report, f, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))
