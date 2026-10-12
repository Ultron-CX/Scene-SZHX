"""Local-only smoke check: files/import ownership; optional actual instance inference."""
import argparse
import hashlib
import importlib.metadata
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from PIL import Image
import torch
import transformers
from scenesem_config import ROOT, SOURCE, SCENE, POSES, VISUALS, MODELS, ENGINE, check_inputs
from instance_semantics import InstanceSemantics

parser = argparse.ArgumentParser()
parser.add_argument('--infer', action='store_true')
parser.add_argument('--record', action='store_true')
args = parser.parse_args()
check_inputs()
assert Path(sys.executable).resolve().is_relative_to(ROOT)
assert Path(torch.__file__).resolve().is_relative_to(ROOT)
assert Path(transformers.__file__).resolve().is_relative_to(ROOT)
for path in [SCENE, MODELS, ENGINE]:
    assert path.is_relative_to(ROOT), f'Runtime dependency outside SceneSem: {path}'
packages = {name: importlib.metadata.version(name) for name in ('numpy', 'Pillow', 'scipy', 'torch', 'transformers', 'safetensors')}
report = dict(project='SceneSem', at=datetime.now(timezone.utc).isoformat(), python=sys.executable,
              package_versions=packages, source_root=str(SOURCE), raw_sources_read_only=True,
              poses=str(POSES), visual_assets={k: {'path':str(p),'bytes':p.stat().st_size} for k,p in VISUALS.items()},
              live_browser_gpu_tested=False, current_chair_024_accuracy_validated=False, real_robot_commands=False)
if args.infer:
    image_file = ROOT / 'artifacts/k2-object-semantics-eval-v1/view-04.png'
    result = InstanceSemantics().infer(Image.open(image_file), .6, 'target', [178,204,352,349])
    report['inference'] = dict(image=str(image_file.relative_to(ROOT)), sha256=hashlib.sha256(image_file.read_bytes()).hexdigest(),
        model=result['model'], model_sha256=result['model_sha256'], components=len(result['components']),
        crop_count=result['crop_count'], seconds=result['seconds'], scope='historical RGB fixture, not current object or 3D accuracy')
    assert result['crop_count']==2 and len(result['components'])>0
if args.record:
    with (ROOT / 'artifacts/migration-20260911/runtime.json').open('x', encoding='utf-8') as f:
        json.dump(report,f,ensure_ascii=False,indent=2)
        f.write('\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
