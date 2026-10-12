"""Test-only bridge: real store/export in a strictly isolated R1 directory."""
import io
import json
import sys
from pathlib import Path
from project_store import ProjectStore, CATEGORIES
from semantic_dataset import export_dataset, read_dataset

ROOT = Path(__file__).resolve().parents[1]
TMP = (ROOT / '.tmp').resolve()
SCENE = dict(scene_id='synthetic-r1-flow-not-a-bank', source_hashes={'test': 'a' * 64}, coordinate_frame='TEST', T_lcc_from_las=[])


def main():
    target = Path(sys.argv[1]).resolve()
    if target.parent != TMP or not target.name.startswith('r1-flow-') or not target.is_dir():
        raise ValueError('Only a pre-created direct .tmp/r1-flow-* directory is allowed')
    request = json.load(sys.stdin)
    store = ProjectStore(target / 'store', SCENE)
    action = request['action']
    if action == 'save':
        return store.save(request['data'])
    if action == 'load':
        return store.load(request.get('revision'))
    if action == 'export':
        stream = io.BytesIO()
        export_dataset(store, request['revision'], stream, CATEGORIES)
        archive = target / ('semantic-v%d.zip' % request['revision'])
        if archive.exists():
            raise ValueError('Test export already exists; no overwrite')
        archive.write_bytes(stream.getvalue())
        # Independent reader, not a reflection of input payload.
        return read_dataset(archive)
    raise ValueError('Unknown R1 test action')


if __name__ == '__main__':
    try:
        print(json.dumps({'ok': True, 'data': main()}, ensure_ascii=False))
    except Exception as exc:
        print(json.dumps({'ok': False, 'error': str(exc)}, ensure_ascii=False))
