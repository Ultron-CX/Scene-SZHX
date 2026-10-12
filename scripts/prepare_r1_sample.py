"""Prepare an empty, disk-isolated R1 sample; never activates a server/browser."""
import hashlib
import json
import tempfile
from pathlib import Path
from project_store import ProjectStore

ROOT = Path(__file__).resolve().parents[1]


def prepare(scene_file, parent):
    scene_file, parent = Path(scene_file).resolve(), Path(parent).resolve()
    if not parent.is_relative_to((ROOT / '.tmp').resolve()):
        raise ValueError('Sample must remain inside SceneSem/.tmp')
    raw = scene_file.read_bytes()
    scene = json.loads(raw)
    for field in ('scene_id', 'source_hashes', 'coordinate_frame', 'T_lcc_from_las'):
        if field not in scene:
            raise ValueError('Missing scene identity: ' + field)
    parent.mkdir(parents=True, exist_ok=True)
    target = Path(tempfile.mkdtemp(prefix='r1-sample-', dir=parent)).resolve()
    store = ProjectStore(target / 'store', scene)
    annotations = {k: scene[k] for k in ('scene_id', 'source_hashes', 'coordinate_frame', 'T_lcc_from_las')}
    annotations.update(schema='digit.regions.v5', boxes=[], real_robot_commands=False)
    saved = store.save(dict(expected_revision=0, annotations=annotations, objects=[]))
    manifest = dict(schema='scenesem.r1-isolated-sample.v1', sample_id=target.name,
                    scene_id=scene['scene_id'], scene_metadata_sha256=hashlib.sha256(raw).hexdigest(),
                    source_metadata=str(scene_file), store=str(target / 'store'), head=saved['head'],
                    initial_regions=0, initial_objects=0, real_robot_commands=False,
                    activated=False, browser_isolation_verified=False,
                    note='Preparation only. Do not import into the live project. Browser draft/API routing isolation is not yet implemented.')
    with (target / 'manifest.json').open('x', encoding='utf-8') as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
    return target


if __name__ == '__main__':
    from scenesem_config import SCENE
    print(prepare(SCENE / 'scene.json', ROOT / '.tmp' / 'r1-samples'))
