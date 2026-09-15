"""Camera geometry and overlays consume the augmented model's forward kinematics."""
import numpy as np
import trimesh
from ..model.transforms import Transform


def set_pose(handle, matrix):
    pose = Transform.from_matrix(matrix).viser()
    handle.position = pose["position"]
    handle.wxyz = pose["wxyz"]


class CameraView:
    def __init__(self, server, model, selected, intrinsics):
        self.frames = {}
        self.geometry = []
        self.collision_geometry = []
        self.frustums = {}
        self.axes = {}
        names = [model.scene.graph.base_frame, "left_wrist_yaw_link", "right_wrist_yaw_link"]
        camera_links = set()
        for side in selected:
            names.extend([f"{side}_camera_mount_link", f"{side}_d405_link"])
            names.extend(f"{side}_d405_{sensor}{suffix}" for sensor in ("color", "depth")
                         for suffix in ("_frame", "_optical_frame"))
            camera_links.update((f"{side}_camera_mount_link", f"{side}_d405_link"))
            optical = f"{side}_d405_depth_optical_frame"
            self.axes[optical] = server.scene.add_line_segments(
                f"/cameras/{side}/optical_axis", points=np.array([[[0., 0., 0.], [0., 0., .08]]]),
                colors=(255, 0, 0), line_width=0.4)
            z = .3  # Frustum depth in metres.
            corners = np.array([[(u-intrinsics.cx)/intrinsics.fx*z,
                                 (v-intrinsics.cy)/intrinsics.fy*z, z]
                                for u,v in ((0,0),(intrinsics.width,0),(intrinsics.width,intrinsics.height),(0,intrinsics.height))])
            # Eight edges only, using the exact intrinsic corner rays.
            segments = np.array([[np.zeros(3), p] for p in corners] +
                                [[corners[i], corners[(i+1)%4]] for i in range(4)])
            # Match the coordinate axes' world-space thickness, slightly slimmer.
            # Split edges into 12 mm dashes separated by 8 mm gaps.
            dashes = []
            for start, end in segments:
                length = np.linalg.norm(end - start)
                direction = (end - start) / length
                for offset in np.arange(0., length, .010):
                    dashes.append([start + direction * offset,
                                   start + direction * min(offset + .006, length)])
            edges = trimesh.util.concatenate([
                trimesh.creation.cylinder(radius=.0005, segment=edge, sections=8)
                for edge in dashes
            ])
            self.frustums[optical] = server.scene.add_mesh_simple(
                f"/cameras/{side}/frustum", vertices=edges.vertices,
                faces=edges.faces, color=(255, 0, 0))
        for name in names:
            self.frames[name] = server.scene.add_frame(f"/frames/{name}", axes_length=.055, axes_radius=.0015)
        for name, mesh in model.scene.geometry.items():
            parent = model.scene.graph.transforms.parents[name]
            if parent in camera_links:
                handle = server.scene.add_mesh_trimesh(f"/camera_geometry/{name}", mesh)
                self.geometry.append((name, handle))
        for name, mesh in model.collision_scene.geometry.items():
            parent = model.collision_scene.graph.transforms.parents[name]
            if parent in camera_links:
                handle = server.scene.add_mesh_simple(
                    f"/camera_collision/{name}", vertices=mesh.vertices, faces=mesh.faces,
                    color=(255, 160, 30), opacity=0.4, visible=False)
                self.collision_geometry.append((name, handle))
        self.update(model)

    def update(self, model):
        for name, handle in self.frames.items():
            set_pose(handle, model.get_transform(name))
        for name, handle in self.geometry:
            set_pose(handle, model.get_transform(name))
        for name, handle in self.collision_geometry:
            set_pose(handle, model.get_transform(name, collision_geometry=True))
        for mapping in (self.frustums, self.axes):
            for name, handle in mapping.items():
                set_pose(handle, model.get_transform(name.split(":")[0]))

    def visibility(self, frames: bool, camera: bool, axes: bool, frustum: bool, collision: bool = False):
        for h in self.frames.values():
            h.visible = frames
        for _, h in self.geometry:
            h.visible = camera
        for _, h in self.collision_geometry:
            h.visible = collision
        for h in self.axes.values():
            h.visible = axes
        for h in self.frustums.values():
            h.visible = frustum
