import numpy as np
import pytest
from g1_wrist_camera.model.transforms import Transform, BODY_OPTICAL
from g1_wrist_camera.config import load_mounts, save_mounts, Intrinsics, sides


def test_composition_and_roundtrip():
    a = Transform((1, 2, 3), (0, 0, np.pi/2)).matrix()
    b = Transform((1, 0, 0), (.2, -.3, .4)).matrix()
    combined = a @ b
    np.testing.assert_allclose(combined[:3, 3], [1, 3, 3])
    np.testing.assert_allclose(Transform.from_matrix(combined).matrix(), combined, atol=1e-14)
    q = Transform.from_matrix(combined).viser()['wxyz']
    assert np.linalg.norm(q) == pytest.approx(1)


def test_optical_axes():
    r = BODY_OPTICAL.matrix()[:3, :3]
    np.testing.assert_allclose(r @ [0,0,1], [1,0,0], atol=1e-14)
    np.testing.assert_allclose(r @ [1,0,0], [0,-1,0], atol=1e-14)
    np.testing.assert_allclose(r @ [0,1,0], [0,0,-1], atol=1e-14)


def test_yaml_roundtrip(tmp_path):
    config = load_mounts()
    path = tmp_path / 'mounts.yaml'
    save_mounts(config, path)
    assert load_mounts(path) == config


@pytest.mark.parametrize('xyz', [[1,2], [0,0,float('nan')], [0,0,float('inf')]])
def test_bad_vectors(xyz):
    with pytest.raises(ValueError, match='three finite'):
        Transform(xyz=xyz)


def test_invalid_config(tmp_path):
    p = tmp_path / 'invalid.yaml'
    p.write_text('left: {}')
    with pytest.raises(ValueError, match='configuration'):
        load_mounts(p)
    with pytest.raises(ValueError):
        sides('other')
    with pytest.raises(ValueError):
        Intrinsics(640, 480, 0, 430, 320, 240)
    with pytest.raises(ValueError):
        Transform.from_matrix(np.zeros((4,4)))
