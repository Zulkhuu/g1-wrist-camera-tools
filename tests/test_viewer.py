"""Real Viser server integration, without a robot, camera, or browser."""
from dataclasses import replace
import numpy as np
from g1_wrist_camera.config import load_mounts, save_mounts
from g1_wrist_camera.model.transforms import Transform
from g1_wrist_camera.viewer.app import ViewerApp


def test_viewer_updates_and_persistence(tmp_path):
    path = tmp_path / 'mounts.yaml'
    save_mounts(load_mounts(), path)
    app = ViewerApp('both', path, port=0)
    try:
        assert len(app.sliders) == 41  # 29 robot + 6 independent joints per hand.
        # Exercise actual GUI callbacks across every subgroup, including non-final sliders.
        for name, slider in app.sliders.items():
            joint = app.model.joint_map[name]
            assert (slider.min, slider.max) == (joint.limit.lower, joint.limit.upper)
            previous_joints = app.joints.copy()
            before = app.model.get_transform(joint.child, joint.parent).copy()
            value = round(slider.min + .6 * (slider.max - slider.min), 3)
            slider.value = value
            assert app.joints == {**previous_joints, name: value}
            after = app.model.get_transform(joint.child, joint.parent)
            assert not np.allclose(before, after), name
            np.testing.assert_allclose(
                app.robot.visual._urdf.get_transform(joint.child, joint.parent), after)
        assert len(app.camera.geometry) >= 2  # D405 bodies plus provided mount meshes.
        pose = Transform((.15,.02,.03), (.1,.2,.3))
        app.set_transform('left', 'mount', pose)
        app.joints['left_wrist_yaw_joint'] = .4
        app.update()
        frame = app.camera.frames['left_d405_depth_optical_frame']
        expected = app.model.get_transform('left_d405_depth_optical_frame')
        np.testing.assert_allclose(frame.position, expected[:3,3])
        np.testing.assert_allclose(app.camera.frustums['left_d405_depth_optical_frame'].position, expected[:3,3])
        assert len(app.camera.collision_geometry) == 4
        for visual, collision in ((True, False), (False, True), (True, True)):
            app.camera.visibility(False, visual, False, False, collision)
            assert all(h.visible == visual for _, h in app.camera.geometry)
            assert all(h.visible == collision for _, h in app.camera.collision_geometry)
        for name, handle in app.camera.collision_geometry:
            matrix = app.model.get_transform(name, collision_geometry=True)
            np.testing.assert_allclose(handle.position, matrix[:3, 3])
        app.save()
        app.set_transform('left', 'mount', Transform())
        app.reload()
        assert app.config['left'].mount == pose
        np.testing.assert_allclose(frame.position, expected[:3,3])
        app.camera.visibility(False, False, False, False)
        assert not frame.visible
        assert not app.camera.geometry[0][1].visible
        assert load_mounts(path)['right'] == load_mounts()['right']
    finally:
        app.server.stop()
