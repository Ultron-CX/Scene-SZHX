"""Explicit version export. Never saves/adopts annotations or overwrites output."""
import argparse
import io
import json
from pathlib import Path
from scenesem_config import ROOT, SCENE
from project_store import ProjectStore, CATEGORIES
from semantic_dataset import export_dataset, read_dataset

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--revision', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--material-sha', action='append', default=[], help='Explicit referenced material SHA-256 to include; repeat for each file')
    parser.add_argument('--confirm-material-export', action='store_true', help='Confirm permission to copy selected materials into this internal dataset')
    args = parser.parse_args()
    store = ProjectStore(ROOT / 'data/projects/k2', json.loads((SCENE / 'scene.json').read_bytes()))
    stream = io.BytesIO()
    if args.material_sha and not args.confirm_material_export: parser.error('--confirm-material-export required with --material-sha')
    export_dataset(store, args.revision, stream, CATEGORIES, material_hashes=args.material_sha if args.confirm_material_export else None)
    stream.seek(0)
    result = read_dataset(stream)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('xb') as f:
        f.write(stream.getvalue())
    print(json.dumps(dict(output=str(args.output.resolve()), revision=args.revision,
        object_records=len(result['objects']), regions=len(result['regions']),
        stage='internal_draft', semantic_accuracy_validated=False, real_robot_commands=False), ensure_ascii=False))
