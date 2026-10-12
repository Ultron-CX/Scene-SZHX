"""Synthetic tests for the single-object geometry export; no live project writes."""
import copy
import tempfile
import unittest
from pathlib import Path

import numpy as np

from project_store import ProjectStore
from object_geometry import cell_centers, geometry_filename, region_ply
from test_project_store import SCENE, TMP, payload


def voxel_region(cells=None, cell=.05, origin=(1.0, 2.0, 3.0), region_id='region-chair-1'):
    cells = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]] if cells is None else cells
    points = np.asarray(cells)
    low = np.asarray(origin) + points.min(axis=0) * cell
    high = np.asarray(origin) + (points.max(axis=0) + 1) * cell
    return dict(id=region_id, category='chair', sequence=4, status='draft',
                min=[float(v) for v in low], max=[float(v) for v in high],
                geometry=dict(type='surface-voxels', origin=list(origin), cell=cell, cells=[list(c) for c in cells]))


def read_ply(payload_bytes):
    text = payload_bytes.decode('utf-8')
    lines = text.split('\n')
    end = lines.index('end_header')
    comments, elements = {}, []
    for line in lines[1:end]:
        if line.startswith('comment '):
            key, _, value = line[8:].partition(' ')
            comments[key] = value
        elif line.startswith('element '):
            elements.append(tuple(line.split()[1:3]))
    vertices = [[float(v) for v in line.split()] for line in lines[end + 1:] if line.strip()]
    return comments, elements, vertices


class GeometryExportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=TMP)
        self.store = ProjectStore(Path(self.temp.name), SCENE)

    def tearDown(self):
        self.temp.cleanup()

    def test_ply_matches_stored_cells_without_touching_the_region(self):
        region = voxel_region()
        original = copy.deepcopy(region)
        comments, elements, vertices = read_ply(region_ply(SCENE, region))
        self.assertEqual(elements, [('vertex', '4')])
        self.assertEqual(len(vertices), 4)
        origin, cell, _, centers = cell_centers(SCENE, region)
        self.assertEqual(vertices, [[*map(float, center), cell] for center in centers])
        self.assertEqual(comments['geometry_type'], 'surface-voxels')
        self.assertEqual(comments['voxel_cell_size_m'], '0.05')
        self.assertEqual(comments['support_cell_count'], '4')
        self.assertEqual(comments['region_id'], 'region-chair-1')
        self.assertEqual(comments['category'], 'chair')
        self.assertEqual(comments['units'], 'meters')
        self.assertEqual(region, original)

    def test_header_keeps_the_limits_explicit(self):
        comments, _, _ = read_ply(region_ply(SCENE, voxel_region()))
        self.assertEqual(comments['coverage'], 'observed_surface_only_completeness_unchecked')
        self.assertEqual(comments['instance_kind'], 'spatial_fragments_not_instance_segmentation')
        self.assertEqual(comments['hidden_surface_inferred'], 'false')
        self.assertEqual(comments['volume_filled'], 'false')
        self.assertEqual(comments['semantic_accuracy_validated'], 'false')
        self.assertEqual(comments['task_qualification'], 'false')
        self.assertEqual(comments['real_robot_commands'], 'false')
        self.assertEqual(comments['scene_id'], SCENE['scene_id'])
        self.assertEqual(comments['coordinate_frame'], SCENE['coordinate_frame'])
        self.assertEqual(comments['lattice_origin_m'], '[1.0, 2.0, 3.0]')
        self.assertEqual(comments['observed_bounds_min_m'], '[1.0, 2.0, 3.0]')
        self.assertEqual(comments['observed_bounds_max_m'], '[1.1, 2.1, 3.05]')

    def test_only_observed_surface_voxels_are_exported(self):
        for geometry in (None, dict(type='surface-mask', origin=[0, 0, 0], u=[1, 0, 0], v=[0, 1, 0], n=[0, 0, 1], low=0, high=.3, uv0=[0, 0], cell=.05, width=2, height=2, runs=[[0, 0, 2]], edges=[])):
            with self.assertRaises(ValueError):
                region_ply(SCENE, dict(voxel_region(), geometry=geometry))
        with self.assertRaises(ValueError):
            region_ply(SCENE, voxel_region(cells=[[0, 0, 0], [0, 0, 0]]))

    def test_bounds_must_agree_with_the_support_cells(self):
        region = voxel_region()
        region['max'] = [v + 1 for v in region['max']]
        with self.assertRaises(ValueError):
            region_ply(SCENE, region)

    def test_saved_revision_exports_the_same_points(self):
        document = payload(count=0)
        region = voxel_region()
        document['annotations']['boxes'] = [region]
        self.store.save(document)
        saved = ProjectStore(self.store.root, SCENE).load(1)['document']['annotations']['boxes'][0]
        self.assertEqual(saved, region)
        self.assertEqual(region_ply(SCENE, saved), region_ply(SCENE, region))
        self.assertEqual(geometry_filename(saved, 1), 'SceneSem-object-chair-4-v1.ply')

    def test_comment_values_cannot_break_the_header(self):
        region = voxel_region(region_id='region\nwith-newline')
        comments, elements, _ = read_ply(region_ply(SCENE, region))
        self.assertEqual(comments['region_id'], 'region with-newline')
        self.assertEqual(elements, [('vertex', '4')])


if __name__ == '__main__':
    unittest.main()
