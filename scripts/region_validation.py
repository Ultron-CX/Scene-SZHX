"""Validate imported v5 geometry before it can reach persistent project state.

Matches the workbench's supported boxes/prisms/masks/visible voxels, not a
claim of scene calibration or semantic correctness.
"""
import math
import numpy as np


def vector(value, length=3):
    return isinstance(value, list) and len(value) == length and all(type(v) in (float, int) and math.isfinite(v) and abs(v) < 100000 for v in value)


def polygon_valid(p):
    if not isinstance(p, list) or not 3 <= len(p) <= 64 or not all(vector(v, 2) for v in p):
        return False
    cross = lambda a, b, c: (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    area = 0
    for i, a in enumerate(p):
        b = p[(i+1) % len(p)]
        if math.dist(a, b) < .001:
            return False
        area += a[0]*b[1]-b[0]*a[1]
        for j in range(i+1, len(p)):
            if j == i+1 or (i == 0 and j == len(p)-1):
                continue
            c, d = p[j], p[(j+1) % len(p)]
            if cross(a, b, c)*cross(a, b, d) <= 0 and cross(c, d, a)*cross(c, d, b) <= 0 and all(max(min(a[k], b[k]), min(c[k], d[k])) <= min(max(a[k], b[k]), max(c[k], d[k])) for k in (0, 1)):
                return False
    return abs(area) > .0002


def geometry_valid(region):
    if not all(vector(region.get(k)) for k in ('min', 'max')):
        return False
    g = region.get('geometry')
    if g is None:
        return True
    if not isinstance(g, dict) or not vector(g.get('origin')):
        return False
    if g.get('type') == 'surface-voxels':
        cell, cells = g.get('cell'), g.get('cells')
        if type(cell) not in (float, int) or not .02 <= cell <= .5 or not isinstance(cells, list) or not 1 <= len(cells) <= 50000 or not all(vector(c) and all(type(n) is int for n in c) for c in cells):
            return False
        if len(set(map(tuple, cells))) != len(cells):
            return False
        p = np.asarray(cells)
        low = np.asarray(g['origin']) + p.min(axis=0)*cell
        high = np.asarray(g['origin']) + (p.max(axis=0)+1)*cell
    elif g.get('type') in ('prism', 'surface-mask'):
        if not all(vector(g.get(k)) for k in ('u', 'v', 'n')) or not all(type(g.get(k)) in (float, int) and math.isfinite(g[k]) and abs(g[k]) <= 100000 for k in ('low', 'high')) or g['high']-g['low'] < .01:
            return False
        axes = np.asarray([g['u'], g['v'], g['n']])
        if any(abs(np.dot(a, a)-1) > 1e-4 for a in axes) or sum(abs(np.dot(axes[a], axes[b])) for a, b in ((0, 1), (0, 2), (1, 2))) > 1e-4:
            return False
        if g['type'] == 'prism':
            poly = g.get('polygon')
            if not polygon_valid(poly):
                return False
        else:
            w, h, cell = g.get('width'), g.get('height'), g.get('cell')
            if not vector(g.get('uv0'), 2) or not all(type(n) is int and 0 < n <= 10000 for n in (w, h)) or w*h > 4000000 or type(cell) not in (float, int) or not .01 <= cell <= 1:
                return False
            runs, edges = g.get('runs'), g.get('edges')
            if not isinstance(runs, list) or not 1 <= len(runs) <= 200000 or not isinstance(edges, list) or len(edges) > 100000:
                return False
            last_y, last_end = -1, -1
            for run in runs:
                if not isinstance(run, list) or len(run) != 3 or not all(type(n) is int for n in run):
                    return False
                y, a, b = run
                if not (0 <= y < h and 0 <= a < b <= w) or y < last_y or (y == last_y and a < last_end):
                    return False
                last_y, last_end = y, b
            if not all(isinstance(e, list) and len(e) == 2 and all(vector(v, 2) for v in e) for e in edges):
                return False
            x, y = g['uv0']
            poly = [[x, y], [x+w*cell, y], [x+w*cell, y+h*cell], [x, y+h*cell]]
        p = np.asarray([np.asarray(g['origin'])+x*axes[0]+y*axes[1]+z*axes[2] for z in (g['low'], g['high']) for x, y in poly])
        low, high = p.min(axis=0), p.max(axis=0)
    else:
        return False
    return bool(np.all(np.abs(low-region['min']) < .002) and np.all(np.abs(high-region['max']) < .002))
