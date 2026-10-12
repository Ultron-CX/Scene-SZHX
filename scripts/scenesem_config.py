"""Explicit local asset references. No runtime imports or fallback to Sim-XGRIDS."""
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HOST, PORT = '127.0.0.1', 18876
CONFIG_FILE = Path(os.environ.get('SCENESEM_CONFIG', str(ROOT / 'config/local.json'))).resolve()
if not CONFIG_FILE.is_file():
    raise RuntimeError('Missing SceneSem config/local.json. Copy local.example.json and set read-only asset paths.')
CONFIG = json.loads(CONFIG_FILE.read_text(encoding='utf-8-sig'))
if CONFIG.get('schema') != 'scenesem.local-config.v1':
    raise ValueError('Unsupported SceneSem local configuration')


def project_path(key):
    path = Path(CONFIG[key])
    return (path if path.is_absolute() else ROOT / path).resolve()


SOURCE = Path(CONFIG['source_root']).resolve()


def source_path(relative):
    path = (SOURCE / relative).resolve()
    if not path.is_relative_to(SOURCE):
        raise ValueError('Configured raw asset must remain inside source_root')
    return path


SCENE = project_path('scene_dir')
FIXTURE = project_path('fixture_dir')
ENGINE = project_path('engine_dir')
MODELS = project_path('models_dir')
PROPOSALS = project_path('proposals')
LEGACY_PROPOSALS = project_path('legacy_proposals')
POSES = source_path(CONFIG['panoramic_poses'])
VISUALS = {name: source_path(CONFIG['visual_files'][name]) for name in ('low', 'medium', 'full')}


def check_inputs():
    required = [SCENE / name for name in ('scene.json', 'points.bin', 'mesh-vertices.bin', 'mesh-indices.bin')]
    required += [FIXTURE / 'report.json', FIXTURE / 'annotation-fixture.npz', ENGINE / 'build/playcanvas.js',
                 ENGINE / 'LICENSE', PROPOSALS, LEGACY_PROPOSALS, POSES, *VISUALS.values()]
    missing = [str(p) for p in required if not p.is_file()]
    if missing:
        raise FileNotFoundError('SceneSem input files missing:\n' + '\n'.join(missing))
