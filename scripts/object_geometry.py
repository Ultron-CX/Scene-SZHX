"""Export one saved region's observed support cells as a standalone point set.

Only the surface-voxels already stored in the region are written. The export
never fills volume, infers hidden surfaces, splits or merges regions, or turns a
spatial fragment into a complete instance or a robot-usable asset.
"""
import json

import numpy as np

from region_validation import geometry_valid

# One saved region never exceeds this; see region_validation.geometry_valid.
MAX_CELLS = 50000


def _text(value):
    return ' '.join(str(value).split())[:300]


def cell_centers(scene, region):
    """Return (origin, cell, cell_indices, centers) for a valid surface-voxels region."""
    if not isinstance(region, dict):
        raise ValueError('区域记录无效，不能导出单体几何')
    geometry = region.get('geometry')
    if not isinstance(geometry, dict) or geometry.get('type') != 'surface-voxels':
        raise ValueError('该区域几何不是已观测表面体素（surface-voxels）；外框、掩膜和轮廓不会被当成体素导出')
    if not geometry_valid(region):
        raise ValueError('区域几何、边界或支持单元无效，未导出')
    cell = float(geometry['cell'])
    origin = np.asarray(geometry['origin'], dtype=float)
    cells = np.asarray(geometry['cells'], dtype=np.int64)
    centers = origin + (cells + .5) * cell
    return origin, cell, cells, centers


def region_ply(scene, region):
    """ASCII PLY point cloud of the region's observed support cells."""
    origin, cell, cells, centers = cell_centers(scene, region)
    if len(centers) > MAX_CELLS:
        raise ValueError('支持单元超过限制，未导出')
    # Same lattice arithmetic as the workbench bounds, so no float drift is introduced.
    low, high = origin + cells.min(axis=0) * cell, origin + (cells.max(axis=0) + 1) * cell
    header = [
        'ply',
        'format ascii 1.0',
        'comment SceneSem observed-surface geometry export',
        'comment schema scenesem.object-geometry.v1',
        'comment scene_id ' + _text(scene.get('scene_id')),
        'comment coordinate_frame ' + _text(scene.get('coordinate_frame')),
        'comment source_hashes ' + _text(json.dumps(scene.get('source_hashes') or {}, sort_keys=True, separators=(',', ':'))),
        'comment region_id ' + _text(region['id']),
        'comment category ' + _text(region['category']),
        'comment sequence ' + _text(region['sequence']),
        'comment region_status ' + _text(region['status']),
        'comment geometry_type surface-voxels',
        'comment support_cell_count ' + str(len(centers)),
        'comment voxel_cell_size_m ' + _text(cell),
        'comment lattice_origin_m ' + _text(json.dumps([float(v) for v in origin])),
        'comment observed_bounds_min_m ' + _text(json.dumps([float(v) for v in low])),
        'comment observed_bounds_max_m ' + _text(json.dumps([float(v) for v in high])),
        'comment units meters',
        'comment coordinate_frame_source scene_document_not_recalibrated',
        'comment coverage observed_surface_only_completeness_unchecked',
        'comment instance_kind spatial_fragments_not_instance_segmentation',
        'comment hidden_surface_inferred false',
        'comment volume_filled false',
        'comment semantic_accuracy_validated false',
        'comment task_qualification false',
        'comment real_robot_commands false',
        'element vertex ' + str(len(centers)),
        'property float x',
        'property float y',
        'property float z',
        'property float voxel_size',
        'end_header',
    ]
    body = ['%.6f %.6f %.6f %.6f' % (p[0], p[1], p[2], cell) for p in centers]
    return ('\n'.join(header + body) + '\n').encode('utf-8')


def geometry_filename(region, revision):
    return f"SceneSem-object-{_text(region['category'])}-{_text(region['sequence'])}-v{revision}.ply"
