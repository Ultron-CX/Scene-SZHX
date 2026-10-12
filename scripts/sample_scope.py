"""Explicit sample routing. Missing/invalid samples never fall back to production."""
import hashlib
import json
import re
import threading
from pathlib import Path
from project_store import ProjectStore

_stores = {}
_lock = threading.Lock()


def resolve_sample(root, sample_id, scene_file):
    if not re.fullmatch(r'r1-sample-[a-z0-9_]{8}', sample_id):
        raise ValueError('Invalid sample ID')
    base = (Path(root) / '.tmp' / 'r1-samples').resolve()
    folder = base / sample_id
    if folder.resolve() != folder or not folder.is_dir():
        raise ValueError('Sample missing or redirected')
    manifest_file, store_path = folder / 'manifest.json', folder / 'store'
    if manifest_file.resolve() != manifest_file or store_path.resolve() != store_path:
        raise ValueError('Sample paths redirected')
    manifest = json.loads(manifest_file.read_text(encoding='utf-8'))
    raw = Path(scene_file).read_bytes()
    scene = json.loads(raw)
    if (manifest.get('schema') != 'scenesem.r1-isolated-sample.v1' or
            manifest.get('sample_id') != sample_id or manifest.get('scene_id') != scene['scene_id'] or
            manifest.get('scene_metadata_sha256') != hashlib.sha256(raw).hexdigest() or
            Path(manifest.get('store', '')).resolve() != store_path):
        raise ValueError('Sample scene identity mismatch')
    with _lock:
        key = (str(store_path), manifest['scene_metadata_sha256'])
        if key not in _stores:
            store = ProjectStore(store_path, scene)
            store.load()  # Require a previously prepared valid store.
            _stores[key] = store
        return _stores[key]
