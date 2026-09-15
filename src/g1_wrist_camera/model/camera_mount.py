"""Single rigid mount and camera description, independent of Viser."""
import logging
from pathlib import Path
import xml.etree.ElementTree as ET
from ..config import WristCamera
from .transforms import D405_OPTICAL, Transform
from .collision import collision_path_for_visual_mesh, load_triangle_mesh

logger = logging.getLogger(__name__)


def fixed(root, parent: str, child: str, pose: Transform):
    link = ET.SubElement(root, "link", name=child)
    joint = ET.SubElement(root, "joint", name=f"{child}_joint", type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", **pose.urdf())
    return link


def visual(link, mesh: Path, scale=(1., 1., 1.), placeholder=False, rgba="0.2 0.5 0.8 1"):
    if not mesh.exists() and not placeholder:
        logger.warning("Missing %s: mount will have coordinate frames only", mesh)
        return
    v = ET.SubElement(link, "visual")
    geometry = ET.SubElement(v, "geometry")
    if mesh.exists():
        ET.SubElement(geometry, "mesh", filename=str(mesh.resolve()), scale=" ".join(map(str, scale)))
    else:
        logger.warning("Missing %s: using a D405 visualization cuboid placeholder", mesh)
        ET.SubElement(geometry, "box", size="0.042 0.023 0.042")
    material = ET.SubElement(v, "material", name=f"{link.get('name')}_material")
    ET.SubElement(material, "color", rgba=rgba)


def add_camera(root, side: str, config: WristCamera, assets: Path):
    mount = f"{side}_camera_mount_link"
    body = f"{side}_d405_link"
    mount_mesh = assets / f"wrist_camera_mount/g1_d405_wrist_mount_{side}.stl"
    # Also accept the organized per-side layout used by the repository.
    if not mount_mesh.exists():
        mount_mesh = assets / f"wrist_camera_mount/{side}/g1_d405_wrist_mount_{side}.stl"
    # Keep the old generic name as a development fallback for compatibility.
    if not mount_mesh.exists():
        legacy = assets / f"wrist_camera_mount/{side}/mount.stl"
        if legacy.exists():
            mount_mesh = legacy
    mount_link = fixed(root, config.parent_link, mount, config.mount)
    if mount_mesh.exists():
        collision_mesh = collision_path_for_visual_mesh(mount_mesh)
        if not collision_mesh.is_file():
            raise FileNotFoundError(
                f"Collision mesh not found: {collision_mesh}\n\nGenerate it with:\n"
                f'uv run python tools/generate_collision_mesh.py "{mount_mesh}"')
        geometry = ET.SubElement(ET.SubElement(mount_link, "collision"), "geometry")
        ET.SubElement(geometry, "mesh", filename=str(collision_mesh.resolve()),
                      scale=" ".join(map(str, config.mesh_scale)))
    visual(mount_link, mount_mesh, config.mesh_scale)
    # Accept the supplied CAD export and the original lowercase filename.
    d405_mesh = assets / "realsense_d405/D405.stl"
    if not d405_mesh.exists():
        d405_mesh = assets / "realsense_d405/d405.stl"
    body_link = fixed(root, mount, body, config.camera)
    visual(body_link, d405_mesh, placeholder=True, rgba="0 0 0 1")
    # Separate camera-body approximation; never include it in the mount hull.
    collision = ET.SubElement(body_link, "collision")
    size = (0.042, 0.023, 0.042)
    if d405_mesh.is_file():
        mesh = load_triangle_mesh(d405_mesh)
        size = mesh.extents
        ET.SubElement(collision, "origin", **Transform(xyz=mesh.bounds.mean(axis=0)).urdf())
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "box", size=" ".join(map(str, size)))
    for sensor in ("color", "depth"):
        frame = f"{side}_d405_{sensor}_frame"
        fixed(root, body, frame, Transform())
        fixed(root, frame, f"{side}_d405_{sensor}_optical_frame", D405_OPTICAL)
