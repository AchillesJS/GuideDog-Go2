# SPDX-License-Identifier: Apache-2.0
"""Check full-pose geometry, mesh provenance and conservative point exclusion."""

import hashlib
import json

from guidedog_perception.robot_body import RobotBody
import numpy as np
import pytest


def robot(shape, origin='', visual=None):
    """Build a minimal complete geometry fixture without simulator assumptions."""
    return ('<robot name="test"><link name="part"><collision>'+origin+'<geometry>'
            + shape+'</geometry></collision>'+(visual or '')+'</link></robot>')


def pose():
    """Use an identity cloud-to-link pose for single-link geometry fixtures."""
    return {'part': ([0, 0, 0], [0, 0, 0, 1])}


@pytest.mark.parametrize('shape', ['<box size=".2 .2 .2"/>', '<sphere radius=".1"/>',
                                   '<cylinder radius=".1" length=".2"/>'])
def test_padded_surfaces_are_removed_but_floor_below_body_survives(tmp_path, shape):
    """A 3D filter must distinguish nearby floor from the robot's elevated volume."""
    body = RobotBody(robot(shape, '<origin xyz="0 0 .3"/>'), tmp_path)
    points = [[0, 0, .3], [.12, 0, .3], [.121, 0, .3], [0, 0, 0]]
    assert body.mask(points, pose()).tolist() == [True, True, False, False]


def test_rotated_shape_and_nonidentity_cloud_transform(tmp_path):
    """Both URDF origin rotation and current link rotation must affect containment."""
    body = RobotBody(robot('<box size=".4 .02 .02"/>',
                           '<origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/>'), tmp_path)
    assert body.mask([[1, .15, 0], [1.15, 0, 0]], pose()).tolist() == [True, False]
    tf = {'part': ([1, 0, 0], [0, 0, 2**-.5, 2**-.5])}
    assert body.mask([[.15, 0, 0], [0, -.15, 0]], tf).tolist() == [True, False]


def test_visual_geometry_is_included_even_outside_collision_primitive(tmp_path):
    """Gazebo renders visual surfaces; collision-only masking would miss these hits."""
    visual = '<visual><origin xyz=".5 0 0"/><geometry><sphere radius=".1"/></geometry></visual>'
    body = RobotBody(robot('<box size=".1 .1 .1"/>', visual=visual), tmp_path)
    assert body.mask([[.5, 0, 0], [.7, 0, 0]], pose()).tolist() == [True, False]


def test_missing_link_transform_never_becomes_partial_filter(tmp_path):
    """Without one exact pose, no point may be accepted under an incomplete model."""
    body = RobotBody(robot('<sphere radius=".1"/>'), tmp_path)
    with pytest.raises(ValueError, match='incomplete_self_transforms'):
        body.mask([[1, 0, 0]], {})


def test_catalog_bounds_include_mesh_origin_scale_and_hash(tmp_path):
    """Use native-converted noncentered bounds, not a guessed sphere or raw DAE units."""
    path = tmp_path/'fixture.dae'
    path.write_bytes(b'catalogued mesh contents')
    known = {'fixture.dae': {'min': [0, 0, 0], 'max': [.1, .2, .3],
                             'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}}
    shape = f'<mesh filename="{path.as_uri()}" scale="2 1 1"/>'
    body = RobotBody(robot(shape), tmp_path, catalog=known)
    assert body.mask([[.19, .1, .2], [-.03, .1, .2]], pose()).tolist() == [True, False]
    path.write_bytes(b'different mesh')
    with pytest.raises(ValueError, match='mesh_hash_mismatch'):
        RobotBody(robot(shape), tmp_path, catalog=known)


def test_individually_symlinked_installed_mesh_is_verified(tmp_path):
    """Verify the expected asset across individual ament symlink-install file links."""
    original = tmp_path/'source'/'fixture.dae'
    original.parent.mkdir()
    original.write_bytes(b'original mesh')
    installed = tmp_path/'install'
    installed.mkdir()
    (installed/'fixture.dae').symlink_to(original)
    known = {'fixture.dae': {'min': [0, 0, 0], 'max': [.1, .1, .1],
                             'sha256': hashlib.sha256(original.read_bytes()).hexdigest()}}
    body = RobotBody(robot(f'<mesh filename="{(installed / "fixture.dae").as_uri()}"/>'),
                     installed, catalog=known)
    assert body.mask([[.05, .05, .05]], pose())[0]
    other = tmp_path/'other'/'fixture.dae'
    other.parent.mkdir()
    other.write_bytes(b'original mesh')
    with pytest.raises(ValueError, match='mesh_outside_catalog_root'):
        RobotBody(robot(f'<mesh filename="{other.as_uri()}"/>'), installed, catalog=known)


@pytest.mark.parametrize('shape', ['<box size="-1 1 1"/>', '<sphere radius="nan"/>',
                                   '<capsule radius="1"/>', '<mesh filename="https://x/a"/>'])
def test_unknown_or_invalid_geometry_is_rejected(tmp_path, shape):
    """Unsupported assets must withhold observations rather than silently disappearing."""
    with pytest.raises(ValueError):
        RobotBody(robot(shape), tmp_path)


def test_packaged_catalog_has_six_native_meshes_and_pinned_revision():
    """Keep native bound provenance small, complete and tied to the fixed dependency."""
    from importlib.resources import files
    catalog = json.loads(files('guidedog_perception').joinpath('mesh_bounds.json').read_text())
    assert len(catalog['meshes']) == 6 and catalog['loader'] == 'gz::common::MeshManager'
    assert catalog['dependency_revision'] == '55ff151632a0220bf3ae0c0772392615d86da7a8'
    for mesh in catalog['meshes'].values():
        assert len(mesh['sha256']) == 64 and mesh['vertices'] > 0
        assert (np.array(mesh['max']) > np.array(mesh['min'])).all()
