import subprocess
import sys

import numpy as np
import pytest
import trimesh

from g1_wrist_camera.config import ROOT, load_mounts
from g1_wrist_camera.model.collision import (
    collision_path_for_visual_mesh,
    generate_collision_mesh,
    generate_convex_hull,
    load_triangle_mesh,
    mesh_statistics,
)
from g1_wrist_camera.model.urdf import load_model


def test_hull_preserves_translated_rotated_scaled_concave_geometry():
    # Extruded L cross-section: a closed, concave solid.
    points = np.array([[0, 0], [2, 0], [2, 1], [1, 1], [1, 2], [0, 2]])
    mesh = trimesh.creation.extrude_triangulation(
        points, [[0, 1, 2], [0, 2, 3], [0, 3, 5], [3, 4, 5]], height=0.7)
    mesh.apply_scale([2, 3, 4])
    mesh.apply_transform(trimesh.transformations.euler_matrix(.2, .4, .8))
    mesh.apply_translation([13, -7, 29])
    before = mesh.vertices.copy()
    assert not mesh.is_convex
    hull = generate_convex_hull(mesh)
    assert len(hull.vertices) > 0 and len(hull.faces) > 0
    assert hull.is_convex and hull.is_volume
    assert hull.volume > mesh.volume
    np.testing.assert_array_equal(mesh.vertices, before)
    np.testing.assert_allclose(hull.bounds, mesh.bounds)
    # Every hull vertex is an original vertex in the unchanged CAD frame.
    for point in hull.vertices:
        assert np.min(np.linalg.norm(before - point, axis=1)) < 1e-10


def test_file_generation_and_overwrite(tmp_path):
    source = tmp_path / 'left.stl'
    trimesh.creation.box().export(source)
    output, _, _ = generate_collision_mesh(source)
    assert output == collision_path_for_visual_mesh(source)
    assert load_triangle_mesh(output).is_convex
    with pytest.raises(FileExistsError, match='--force'):
        generate_collision_mesh(source)
    generate_collision_mesh(source, force=True)
    explicit = tmp_path / 'new/subdir/custom.stl'
    generate_collision_mesh(source, explicit)
    assert explicit.is_file()
    with pytest.raises(ValueError, match='differ'):
        generate_collision_mesh(source, source, force=True)


def test_invalid_and_unreliable_geometry(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_triangle_mesh(tmp_path / 'absent.stl')
    broken = tmp_path / 'bad.stl'
    broken.write_text('not an STL')
    with pytest.raises(ValueError):
        load_triangle_mesh(broken)
    with pytest.raises(ValueError, match='vertices and faces'):
        generate_convex_hull(trimesh.Trimesh())
    planar = trimesh.Trimesh(vertices=[[0, 0, 0], [1, 0, 0], [0, 1, 0]], faces=[[0, 1, 2]])
    assert mesh_statistics(planar)['volume'] is None
    with pytest.raises(ValueError, match='valid convex hull'):
        generate_convex_hull(planar)


def test_scene_keeps_instance_transforms(tmp_path, monkeypatch):
    path = tmp_path / 'scene.glb'
    path.touch()
    scene = trimesh.Scene()
    for x in (10, 15):
        scene.add_geometry(trimesh.creation.box(), transform=trimesh.transformations.translation_matrix([x, 2, 3]))
    monkeypatch.setattr(trimesh, 'load_mesh', lambda _: scene)
    mesh = load_triangle_mesh(path)
    np.testing.assert_allclose(mesh.bounds, scene.bounds)
    assert len(mesh.faces) == 24


@pytest.mark.parametrize('side', ['left', 'right', 'both'])
def test_repository_collision_assets_and_urdf(side):
    model = load_model(load_mounts(), side)
    for s in (('left', 'right') if side == 'both' else (side,)):
        link = model.link_map[f'{s}_camera_mount_link']
        visual = link.visuals[0].geometry.mesh
        collision = link.collisions[0].geometry.mesh
        assert collision.filename == str(collision_path_for_visual_mesh(visual.filename))
        source, hull = load_triangle_mesh(visual.filename), load_triangle_mesh(collision.filename)
        assert hull.is_convex and hull.is_volume
        assert len(hull.faces) < len(source.faces)
        np.testing.assert_allclose(hull.bounds, source.bounds)
        assert model.link_map[f'{s}_d405_link'].collisions[0].geometry.box is not None


def test_cli(tmp_path):
    source = tmp_path / 'mount.stl'
    trimesh.creation.box().export(source)
    command = [sys.executable, str(ROOT / 'tools/generate_collision_mesh.py'), str(source)]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    assert 'Volume ratio:' in result.stdout
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    assert result.returncode == 1 and '--force' in result.stderr
