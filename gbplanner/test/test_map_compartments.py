import copy
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from configure_compartments_from_map import (
    infer_geometry, read_pcd, updated_config, wall_chain, write_config,
)


def test_rotated_tank_with_baffle():
    # Six transverse walls, plus an extra nearby plate inside the first room.
    yz = np.array([(y, z) for y in np.linspace(-1, 1, 32)
                   for z in np.linspace(0, 3, 32)])
    walls = [0, 0.8, 2.4, 4.8, 7.2, 9.6, 12]
    cloud = np.concatenate([np.column_stack([np.full(len(yz), x), yz]) for x in walls])
    yaw = 0.2
    rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                         [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    cloud = cloud @ rotation.T
    normals = np.tile(rotation[:, 0], (len(cloud), 1))
    result = infer_geometry(cloud, normals, 0, 1.5, 3.5, 5)
    assert result['axis_yaw_deg'] == pytest.approx(np.degrees(yaw))
    assert result['selected_walls'] == pytest.approx([0, 2.4, 4.8, 7.2, 9.6, 12])
    centers = np.array(result['compartment_centers']).reshape(-1, 3)
    assert centers[2] == pytest.approx(rotation @ [6, 0, 1.5], abs=0.03)
    half = np.array(result['compartment_dimensions']['max_val'])
    low, high = [np.array(result['global_bounds'][key]) for key in ('min_val', 'max_val')]
    assert np.all(centers - half >= low - 1e-9)
    assert np.all(centers + half <= high + 1e-9)


def test_bad_layout_fails_instead_of_inventing_rooms():
    with pytest.raises(ValueError, match='No wall sequence'):
        wall_chain([0, 10], 1.5, 3.5, 5)
    with pytest.raises(ValueError, match='Too few'):
        infer_geometry(np.zeros((10, 3)), None, 0, 1.5, 3.5, None)


@pytest.mark.parametrize('mode', ['ascii', 'binary'])
def test_read_xyz_pcd_without_normals(tmp_path, mode):
    path = tmp_path / 'test.pcd'
    header = ('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              'WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ' + mode + '\n').encode()
    points = np.array([[1, 2, 3], [4, 5, 6]], dtype='<f4')
    path.write_bytes(header + (points.tobytes() if mode == 'binary' else b'1 2 3\n4 5 6\n'))
    loaded, normals = read_pcd(path)
    np.testing.assert_array_equal(loaded, points)
    assert normals is None


def test_apply_preserves_unrelated_values_and_backup(tmp_path):
    original = {'PlanningParams': {'compartment_centers': [0, 0, 0],
                'compartment_dimensions': {'type': 'kCuboid', 'rotation': [0, 0, 0]},
                'max_opening_attempts': 3}, 'BoundedSpaceParams': {'Global': {
                'type': 'kCuboid', 'min_val': [-1]*3, 'max_val': [1]*3}},
                'RobotParams': {'size': [0.4]*3}}
    saved = copy.deepcopy(original)
    geometry = {'compartment_centers': [2, 0, 1], 'compartment_dimensions': {
                'type': 'kCuboid', 'min_val': [-2]*3, 'max_val': [2]*3},
                'global_bounds': {'type': 'kCuboid', 'min_val': [-3]*3, 'max_val': [3]*3}}
    new = updated_config(original, geometry)
    assert original == saved
    assert new['RobotParams'] == original['RobotParams']
    assert new['PlanningParams']['max_opening_attempts'] == 3
    assert new['PlanningParams']['compartment_dimensions']['rotation'] == [0, 0, 0]
    path = tmp_path / 'config.yaml'
    path.write_text(yaml.safe_dump(original))
    previous = path.read_bytes()
    backup = write_config(path, new, apply=True)
    assert backup.read_bytes() == previous
    assert yaml.safe_load(path.read_text()) == new
    with pytest.raises(ValueError, match='Output exists'):
        write_config(path, original)
