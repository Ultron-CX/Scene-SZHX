"""Create strictly synthetic v4 delivery samples, never from the live project."""
import io
import json
from datetime import datetime, timezone
from pathlib import Path
from test_material_delivery import DeliveryTests
from semantic_dataset import read_dataset, sha

if __name__=='__main__':
    fixture=DeliveryTests();fixture.setUp()
    try:
        stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        target=Path(__file__).resolve().parents[1]/'artifacts'/('dataset-'+stamp[:8])
        target.mkdir(parents=True,exist_ok=True)
        rows=[]
        for mode,selected in [('partial',fixture.hashes[:1]),('all',fixture.hashes)]:
            raw=fixture.package(selected);result=read_dataset(io.BytesIO(raw))
            path=target/('materials-synthetic-'+mode+'-'+stamp+'.zip')
            with path.open('xb') as f:f.write(raw)
            rows.append(dict(path=str(path),sha256=sha(raw),bytes=len(raw),materials=result['materials']))
    finally:fixture.tearDown()
    # Read again after the isolated source store has been removed: package-only consumption.
    for row in rows:read_dataset(row['path'])
    report=dict(synthetic_only=True,source_store_present_during_final_read=False,samples=rows,
                live_user_labels_modified=False,browser_gpu_tested=False,source_content_verified=False,real_robot_commands=False)
    path=target/('materials-delivery-'+stamp+'.json')
    with path.open('x',encoding='utf8') as f:json.dump(report,f,ensure_ascii=False,indent=2)
    print(json.dumps(dict(report=str(path),samples=[r['path'] for r in rows]),ensure_ascii=False))
