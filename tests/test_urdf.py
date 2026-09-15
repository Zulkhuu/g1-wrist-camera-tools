from dataclasses import replace
import hashlib
import json
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yourdfpy
from g1_wrist_camera.config import BASE_URDF, load_mounts
from g1_wrist_camera.model.urdf import load_model, augmented_xml, update_camera, load_base
from g1_wrist_camera.model.transforms import Transform, D405_OPTICAL


def test_upstream_loads_and_checksums():
    model = yourdfpy.URDF.load(str(BASE_URDF), mesh_dir=str(BASE_URDF.parent))
    assert {'left_wrist_yaw_link', 'right_wrist_yaw_link'} <= model.link_map.keys()
    assert len(model.scene.geometry) >= 93
    checksums = json.loads((BASE_URDF.parent / 'checksums.json').read_text())
    for name, digest in checksums.items():
        assert hashlib.sha256((BASE_URDF.parent / name).read_bytes()).hexdigest() == digest


@pytest.mark.parametrize('side', ['left', 'right', 'both'])
def test_augmented_tree(side):
    config = load_mounts()
    model = load_model(config, side)
    selected = ('left', 'right') if side == 'both' else (side,)
    for s in selected:
        for link in ('camera_mount_link', 'd405_link', 'd405_color_frame', 'd405_depth_frame',
                     'd405_color_optical_frame', 'd405_depth_optical_frame'):
            assert f'{s}_{link}' in model.link_map
        np.testing.assert_allclose(
            model.get_transform(f'{s}_d405_link', f'{s}_camera_mount_link'),
            config[s].camera.matrix(), atol=1e-14)
        for sensor in ('depth', 'color'):
            relative = model.get_transform(f'{s}_d405_{sensor}_optical_frame', f'{s}_d405_link')
            np.testing.assert_allclose(relative[:3, :3] @ [0, 0, 1], [0, 1, 0], atol=1e-14)
        expected = config[s].mount.matrix() @ config[s].camera.matrix() @ D405_OPTICAL.matrix()
        np.testing.assert_allclose(model.get_transform(f'{s}_d405_depth_optical_frame', config[s].parent_link), expected, atol=1e-14)
    if side != 'both':
        other = 'right' if side == 'left' else 'left'
        assert f'{other}_d405_link' not in model.link_map


def test_moving_camera_and_robot():
    config = load_mounts()
    model = load_model(config, 'left')
    config['left'] = replace(config['left'], mount=Transform((.12,.03,-.02), (.3,.2,.1)))
    update_camera(model, 'left', config['left'])
    model.update_cfg({'left_wrist_yaw_joint': .4})
    np.testing.assert_allclose(model.get_transform('left_d405_link', 'left_camera_mount_link'), config['left'].camera.matrix(), atol=1e-14)
    expected = model.get_transform(config['left'].parent_link) @ config['left'].mount.matrix() @ config['left'].camera.matrix() @ D405_OPTICAL.matrix()
    np.testing.assert_allclose(model.get_transform('left_d405_depth_optical_frame'), expected, atol=1e-14)


def test_bad_parent_and_missing_custom_assets(tmp_path):
    config = load_mounts()
    xml = ET.fromstring(augmented_xml(config, assets=tmp_path))
    assert xml.find("link[@name='left_camera_mount_link']/visual") is None
    assert xml.find("link[@name='left_d405_link']/visual/geometry/box") is not None
    config['left'] = replace(config['left'], parent_link='nonexistent')
    with pytest.raises(ValueError, match='parent_link'):
        augmented_xml(config)


def test_single_mesh_and_scale(tmp_path):
    mesh = tmp_path / 'wrist_camera_mount/left/mount.stl'
    mesh.parent.mkdir(parents=True)
    import trimesh
    trimesh.creation.box().export(mesh)
    trimesh.creation.box().export(mesh.with_name("mount_collision.stl"))
    config = load_mounts()
    config['left'] = replace(config['left'], mesh_scale=(.001,.001,.001))
    xml = ET.fromstring(augmented_xml(config, 'left', assets=tmp_path))
    mount = xml.find("link[@name='left_camera_mount_link']")
    assert len(mount.findall('visual')) == 1
    visual = mount.find('visual/geometry/mesh')
    collision = mount.find('collision/geometry/mesh')
    assert visual.get('scale') == collision.get('scale') == '0.001 0.001 0.001'
    assert visual.get('filename') == str(mesh)
    assert collision.get('filename') == str(mesh.with_name('mount_collision.stl'))
    assert mount.find('visual/origin') is None
    assert mount.find('collision/origin') is None


def test_chained_thumb_mimic():
    model = load_base()
    mimic = model.joint_map['left_thumb_4_joint'].mimic
    assert mimic.joint == 'left_thumb_2_joint'
    assert mimic.multiplier == pytest.approx(.8024 * .9487)
    model.update_cfg({'left_thumb_2_joint': .5})
    joint = model.joint_map['left_thumb_4_joint']
    actual = model.get_transform(joint.child, joint.parent)
    expected = joint.origin @ Transform(rpy=(0, 0, -.5 * .8024 * .9487)).matrix()
    np.testing.assert_allclose(actual, expected, atol=1e-14)


def test_missing_collision_is_actionable(tmp_path):
    import trimesh
    mesh = tmp_path / 'wrist_camera_mount/right/mount.stl'
    mesh.parent.mkdir(parents=True)
    trimesh.creation.box().export(mesh)
    with pytest.raises(FileNotFoundError, match='Collision mesh not found') as error:
        augmented_xml(load_mounts(), 'right', assets=tmp_path)
    assert 'generate_collision_mesh.py' in str(error.value)
    assert 'mount_collision.stl' in str(error.value)
