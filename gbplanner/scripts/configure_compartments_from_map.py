#!/usr/bin/env python3
"""Estimate a straight row of compartments from a PCD; explicitly apply with --apply.

Works in the map's existing world frame. This is a geometric heuristic, not a
semantic compartment or opening detector. See README for assumptions and limits.
"""
import argparse
import copy
import datetime
import json
import os
from pathlib import Path
import shutil
import tempfile

import numpy as np
from scipy.signal import find_peaks
from scipy.spatial import cKDTree
import yaml


def read_pcd(path):
    """Read XYZ and optional normals from ASCII or uncompressed binary PCD."""
    with open(path, 'rb') as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD has no DATA header')
            words = line.decode('ascii').strip().split()
            if not words or words[0].startswith('#'):
                continue
            header[words[0]] = words[1:]
            if words[0] == 'DATA':
                break
        names = header['FIELDS']
        counts = list(map(int, header.get('COUNT', ['1'] * len(names))))
        sizes = list(map(int, header['SIZE']))
        types = header['TYPE']
        if not len(names) == len(counts) == len(sizes) == len(types):
            raise ValueError('Inconsistent PCD field metadata')
        mode = header['DATA'][0]
        if mode == 'ascii':
            data = np.loadtxt(stream, ndmin=2)
            if data.shape[1] != sum(counts):
                raise ValueError('PCD ASCII column count does not match header')
            starts = np.cumsum([0] + counts[:-1])
            columns = {name: data[:, i] for name, i in zip(names, starts)}
        elif mode == 'binary':
            kinds = {'F': 'f', 'I': 'i', 'U': 'u'}
            dtype = np.dtype([(name, '<' + kinds[kind] + str(size), (count,))
                              for name, kind, size, count in zip(names, types, sizes, counts)])
            data = np.fromfile(stream, dtype=dtype)
            columns = {name: data[name][:, 0] for name in names}
        else:
            raise ValueError('Use ASCII or binary PCD; binary_compressed is not supported')
        if 'POINTS' in header and len(data) != int(header['POINTS'][0]):
            raise ValueError('PCD point count does not match header')
        for name in ('x', 'y', 'z'):
            if name not in columns or counts[names.index(name)] != 1:
                raise ValueError('PCD requires scalar x, y, z fields')
        xyz = np.column_stack([columns[k] for k in ('x', 'y', 'z')]).astype(float)
        normals = (np.column_stack([columns[k] for k in ('normal_x', 'normal_y', 'normal_z')])
                   if all(k in columns for k in ('normal_x', 'normal_y', 'normal_z')) else None)
        valid = np.isfinite(xyz).all(axis=1)
        return xyz[valid], normals[valid] if normals is not None else None


def estimate_normals(points):
    """Local PCA; batching bounds memory for large clouds."""
    tree = cKDTree(points)
    result = np.empty_like(points)
    for start in range(0, len(points), 4000):
        _, indices = tree.query(points[start:start + 4000], k=min(20, len(points)))
        neighbors = points[indices]
        centered = neighbors - neighbors.mean(axis=1, keepdims=True)
        _, vectors = np.linalg.eigh(np.einsum('nki,nkj->nij', centered, centered))
        result[start:start + len(indices)] = vectors[:, :, 0]
    return result


def wall_chain(offsets, min_width, max_width, expected=None):
    """Choose an ordered chain spanning the first/last detected transverse walls.

    Prefer more rooms, then more regular spacing; expected count constrains this.
    Retain all candidates in the report so rejected baffles can be reviewed.
    """
    pitch = float(np.median(np.diff(offsets)))
    states = {(0, 0): (0.0, [0])}
    for j in range(1, len(offsets)):
        for (i, count), (cost, chain) in list(states.items()):
            gap = offsets[j] - offsets[i]
            if i >= j or not min_width <= gap <= max_width:
                continue
            key = (j, count + 1)
            candidate = (cost + (gap - pitch) ** 2, chain + [j])
            if key not in states or candidate[0] < states[key][0]:
                states[key] = candidate
    choices = [(count, cost, chain) for (end, count), (cost, chain) in states.items()
               if end == len(offsets) - 1 and count > 0 and (expected is None or count == expected)]
    if not choices:
        raise ValueError('No wall sequence fits the room width/count constraints; inspect ROI and candidates: ' + str(offsets))
    choices.sort(key=lambda item: (-item[0], item[1]))
    return choices[0][2]


def infer_geometry(points, normals, axis_yaw, min_width, max_width, expected,
                   margin=0.3, bin_size=0.08, wall_offsets=None):
    if len(points) < 200:
        raise ValueError('Too few finite points in the selected region')
    if normals is None:
        normals = estimate_normals(points)
    norms = np.linalg.norm(normals, axis=1)
    valid = np.isfinite(normals).all(axis=1) & (norms > 1e-6)
    normals = normals[valid] / norms[valid, None]
    normal_points = points[valid]
    seed = np.array([np.cos(axis_yaw), np.sin(axis_yaw), 0.0])
    transverse = (np.abs(normals @ seed) > 0.9) & (np.abs(normals[:, 2]) < 0.2)
    if transverse.sum() < 100:
        raise ValueError('Too few transverse wall normals; check axis yaw / ROI')
    oriented = normals[transverse] * np.sign(normals[transverse] @ seed)[:, None]
    direction = np.median(oriented[:, :2], axis=0)
    direction /= np.linalg.norm(direction)
    u = np.r_[direction, 0.0]
    v = np.array([-u[1], u[0], 0.0])
    projected = points @ u
    bins = np.arange(projected.min() - bin_size, projected.max() + 2 * bin_size, bin_size)
    histogram, edges = np.histogram(normal_points[transverse] @ u, bins=bins)
    peaks, _ = find_peaks(histogram, prominence=max(20, 0.04 * histogram.max()),
                         distance=max(1, int(0.4 / bin_size)))
    candidates = []
    for index in peaks:
        rough = (edges[index] + edges[index + 1]) / 2
        values = normal_points[transverse] @ u
        support = np.abs(values - rough) < 2 * bin_size
        candidates.append(float(np.median(values[support])))
    if wall_offsets is not None:
        walls = np.array(wall_offsets, dtype=float)
        if len(walls) < 2 or not np.all(np.diff(walls) > 0):
            raise ValueError('Wall offsets must be strictly increasing, including both end walls')
        if expected is not None and len(walls) != expected + 1:
            raise ValueError('Explicit walls do not match expected compartment count')
    else:
        if len(candidates) < 2:
            raise ValueError('Could not detect two end walls')
        chosen = wall_chain(candidates, min_width, max_width, expected)
        walls = np.array(candidates)[chosen]
    rooms = []
    for lo, hi in zip(walls[:-1], walls[1:]):
        room = points[(projected >= lo) & (projected <= hi)]
        if len(room) < 100:
            raise ValueError('Insufficient points between walls %.3f and %.3f' % (lo, hi))
        lateral = room @ v
        side_lo, side_hi = np.quantile(lateral, [0.01, 0.99])
        floor, ceiling = np.quantile(room[:, 2], [0.01, 0.99])
        center = u * ((lo + hi) / 2) + v * ((side_lo + side_hi) / 2)
        center[2] = (floor + ceiling) / 2
        corners = np.array([u * x + v * y + np.array([0, 0, z])
                            for x in (lo, hi) for y in (side_lo, side_hi)
                            for z in (floor, ceiling)])
        rooms.append({'center': center.tolist(), 'min': corners.min(axis=0).tolist(),
                      'max': corners.max(axis=0).tolist(), 'points': len(room)})
    centers = np.array([r['center'] for r in rooms])
    # The planner supports a single shared axis-aligned size, not one size/room.
    half_size = np.max([(np.array(r['max']) - np.array(r['min'])) / 2 for r in rooms], axis=0) + margin
    global_min = np.min(centers - half_size, axis=0)
    global_max = np.max(centers + half_size, axis=0)
    return {'axis_yaw_deg': float(np.degrees(np.arctan2(u[1], u[0]))),
            'wall_candidates': candidates, 'selected_walls': walls.tolist(), 'rooms': rooms,
            'compartment_centers': centers.reshape(-1).tolist(),
            'compartment_dimensions': {'type': 'kCuboid', 'min_val': (-half_size).tolist(), 'max_val': half_size.tolist()},
            'global_bounds': {'type': 'kCuboid', 'min_val': global_min.tolist(), 'max_val': global_max.tolist()}}


def updated_config(original, geometry):
    result = copy.deepcopy(original)
    result['PlanningParams']['compartment_centers'] = geometry['compartment_centers']
    result['PlanningParams']['compartment_dimensions'].update(geometry['compartment_dimensions'])
    result['BoundedSpaceParams']['Global'].update(geometry['global_bounds'])
    return result


def write_config(path, config, apply=False):
    """Backup and atomic replace on the same filesystem."""
    path = Path(path)
    if path.exists() and not apply:
        raise ValueError('Output exists; choose a new output path')
    backup = None
    if apply:
        backup = path.with_name(path.name + '.bak.' + datetime.datetime.now().strftime('%Y%m%dT%H%M%S%f'))
        shutil.copy2(path, backup)
    fd, temporary = tempfile.mkstemp(dir=str(path.parent), prefix='.' + path.name)
    try:
        with os.fdopen(fd, 'w') as stream:
            yaml.safe_dump(config, stream, sort_keys=False)
        if apply:
            shutil.copymode(path, temporary)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return backup


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, type=Path, help='Downloaded PCD in the planner world frame')
    parser.add_argument('--config', required=True, type=Path, help='Existing planner YAML')
    region = parser.add_mutually_exclusive_group(required=True)
    region.add_argument('--roi', nargs=6, type=float, metavar=('XMIN', 'YMIN', 'ZMIN', 'XMAX', 'YMAX', 'ZMAX'))
    region.add_argument('--whole-map', action='store_true', help='Assert that the map contains only the tank of interest')
    parser.add_argument('--axis-yaw-deg', type=float, default=0, help='Approximate direction of compartment progression')
    parser.add_argument('--min-width', type=float, default=1.5)
    parser.add_argument('--max-width', type=float, default=3.5)
    parser.add_argument('--expected-compartments', type=int)
    parser.add_argument('--margin', type=float, default=0.3, help='Sampling envelope margin in meters (not collision clearance)')
    parser.add_argument('--wall-offsets', nargs='+', type=float, help='Override wall offsets along the fitted axis, including end walls')
    parser.add_argument('--report-prefix', type=Path, default=Path('compartment_map_report'))
    output = parser.add_mutually_exclusive_group()
    output.add_argument('--apply', action='store_true', help='Update --config atomically after making a backup')
    output.add_argument('--output', type=Path, help='Write a new complete planner YAML')
    args = parser.parse_args()
    try:
        if not 0 < args.min_width <= args.max_width or not np.isfinite([args.min_width, args.max_width, args.margin, args.axis_yaw_deg]).all() or args.margin < 0:
            raise ValueError('Invalid widths, margin or yaw')
        if args.expected_compartments is not None and args.expected_compartments < 1:
            raise ValueError('Expected compartment count must be positive')
        points, normals = read_pcd(args.map.expanduser())
        if args.roi:
            low, high = np.array(args.roi[:3]), np.array(args.roi[3:])
            if not np.isfinite(args.roi).all() or np.any(low >= high):
                raise ValueError('ROI must have finite minimum < maximum on every axis')
            mask = ((points >= low) & (points <= high)).all(axis=1)
            points = points[mask]
            normals = normals[mask] if normals is not None else None
        geometry = infer_geometry(points, normals, np.radians(args.axis_yaw_deg), args.min_width,
                                  args.max_width, args.expected_compartments, args.margin,
                                  wall_offsets=args.wall_offsets)
        geometry['source_map'] = str(args.map.resolve())
        geometry['roi'] = args.roi
        geometry['assumptions'] = 'Straight row, parallel transverse walls, both end walls present; spacing heuristic may reject baffles. Review preview.'
        prefix = args.report_prefix.expanduser()
        prefix.parent.mkdir(parents=True, exist_ok=True)
        Path(str(prefix) + '.json').write_text(json.dumps(geometry, indent=2) + '\n')
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        fig, ax = plt.subplots(figsize=(12, 5))
        subset = points[::max(1, len(points) // 25000)]
        ax.scatter(subset[:, 0], subset[:, 1], s=0.3, color='gray')
        for i, room in enumerate(geometry['rooms']):
            c, lo, hi = np.array(room['center']), np.array(room['min']), np.array(room['max'])
            ax.add_patch(Rectangle(lo[:2], hi[0]-lo[0], hi[1]-lo[1], fill=False, edgecolor='tab:blue'))
            ax.plot(c[0], c[1], 'rx'); ax.annotate(str(i + 1), c[:2])
        ax.set(xlabel='World X (m)', ylabel='World Y (m)', title='Estimated compartments — review before flight', aspect='equal')
        fig.tight_layout(); fig.savefig(str(prefix) + '.png', dpi=160); plt.close(fig)
        print('Centers:', np.array(geometry['compartment_centers']).reshape(-1, 3).round(3).tolist())
        print('Shared dimensions:', geometry['compartment_dimensions'])
        print('Global bounds:', geometry['global_bounds'])
        print('Wall candidates:', np.round(geometry['wall_candidates'], 3).tolist())
        print('Selected walls:', np.round(geometry['selected_walls'], 3).tolist())
        print('Report:', str(prefix) + '.json', str(prefix) + '.png')
        if args.apply or args.output:
            config = yaml.safe_load(args.config.read_text())
            new_config = updated_config(config, geometry)
            destination = args.config if args.apply else args.output
            backup = write_config(destination, new_config, args.apply)
            print('Wrote:', destination)
            if backup:
                print('Backup:', backup)
            print('Restart with autoassess_set_global_bound:=false so the bridge does not replace these bounds.')
    except (ValueError, KeyError, OSError) as error:
        parser.exit(2, 'Error: %s\n' % error)


if __name__ == '__main__':
    main()
