"""Application state and synchronized updates; no DDS or camera hardware."""
from dataclasses import replace
from pathlib import Path
from threading import RLock
import logging
import time
import viser
from ..config import DEFAULT_MOUNTS, load_mounts, load_intrinsics, save_mounts, sides
from ..model.urdf import load_model, update_camera
from ..model.poses import reset_pose
from ..model.transforms import Transform
from .robot_view import RobotView
from .camera_view import CameraView


class ViewerApp:
    def __init__(self, side="both", config: Path = DEFAULT_MOUNTS, host="127.0.0.1", port=8080):
        self.lock = RLock()
        self.path = config
        self.selected = sides(side)
        self.config = load_mounts(config)
        self.model = load_model(self.config, side)
        intrinsics = load_intrinsics()
        self.joints = reset_pose(self.model)
        self.server = viser.ViserServer(host=host, port=port, label="G1 Wrist Camera Viewer")
        self.server.scene.set_up_direction("+z")
        self.robot = RobotView(self.server)
        self.camera = CameraView(self.server, self.model, self.selected, intrinsics)
        from .gui import build_gui
        build_gui(self, side)
        self.update()
        logging.info("G1 Wrist Camera Viewer: http://%s:%s", host, self.server.get_port())

    def update(self):
        with self.lock, self.server.atomic():
            self.model.update_cfg(self.joints)
            self.robot.update(self.joints)
            self.camera.update(self.model)

    def set_transform(self, side: str, part: str, pose: Transform):
        with self.lock:
            self.config[side] = replace(self.config[side], **{part: pose})
            update_camera(self.model, side, self.config[side])
            self.update()

    def save(self):
        with self.lock:
            save_mounts(self.config, self.path)

    def reload(self):
        with self.lock:
            config = load_mounts(self.path)
            # Parent/mesh changes require rebuilding geometry; avoid silently ignoring them.
            for s in self.selected:
                if (config[s].parent_link != self.config[s].parent_link or
                    config[s].mesh_scale != self.config[s].mesh_scale):
                    raise ValueError("Parent link or mesh scale changed: restart the viewer to apply")
            self.config = config
            for s in self.selected:
                update_camera(self.model, s, self.config[s])
            self.update()

    def run(self):
        try:
            while True:
                time.sleep(.5)
        except KeyboardInterrupt:
            pass
        finally:
            self.server.stop()
