"""Synthetic v3 consumer fixture. No production project writes or material fetches."""
import copy
import io
import json
import tempfile
import uuid
from datetime import datetime, timezone
from pathlib import Path
from project_store import ProjectStore, CATEGORIES
from test_project_store import SCENE, TMP, payload
from test_business_context import fixture
from test_spatial_context import fixture as spatial_fixture
from semantic_dataset import export_dataset, read_dataset, sha

root=Path(__file__).resolve().parents[1]
stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
with tempfile.TemporaryDirectory(dir=TMP) as folder:
    store=ProjectStore(Path(folder),SCENE); p=payload(count=2)
    p['business_context']=fixture(p['objects'][0]['object_id']); p['spatial_context']=spatial_fixture(p['objects'][0]['object_id'])
    field=copy.deepcopy(p['business_context']['attributes'][0]);field.update(attribute_id='attr-'+str(uuid.uuid4()),subject_kind='zone',subject_id=p['spatial_context']['zones'][0]['zone_id'],basis='field_report',value='Synthetic field statement, not bank truth',source='Fictitious source for contract tests',recorder='Synthetic witness',observed_on='2026-09-14')
    unknown=copy.deepcopy(field);unknown.update(attribute_id='attr-'+str(uuid.uuid4()),name='待补业务信息',basis='unknown',value='',source='',recorder='',observed_on=None,valid_from=None,valid_until=None)
    p['business_context']['attributes'] += [field,unknown];store.save(p)
    out=io.BytesIO();export_dataset(store,1,out,CATEGORIES);read=read_dataset(io.BytesIO(out.getvalue()))
    assert read['business']==p['business_context'] and read['spatial']==p['spatial_context']
    path=root/'artifacts/dataset-20260914'/('business-synthetic-'+stamp+'.zip')
    with path.open('xb') as f:f.write(out.getvalue())
    report=dict(scope='synthetic_contract_not_K2_or_bank_truth',dataset=path.name,sha256=sha(out.getvalue()),schema=read['manifest']['schema'],quality=read['quality']['business'],production_modified=False,browser_gpu_tested=False,real_robot_commands=False)
    with path.with_suffix('.json').open('x',encoding='utf8') as f:json.dump(report,f,ensure_ascii=False,indent=2)
    print(json.dumps(report,ensure_ascii=False,indent=2))
