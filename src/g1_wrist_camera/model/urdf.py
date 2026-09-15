"""Generate an augmented description without writing to the upstream URDF."""
from io import BytesIO
from pathlib import Path
import xml.etree.ElementTree as ET
import yourdfpy
from ..config import BASE_URDF, ROOT, WristCamera, sides
from .camera_mount import add_camera


def augmented_xml(config: dict[str, WristCamera], side: str = "both",
                  base: Path = BASE_URDF, assets: Path = ROOT / "assets") -> bytes:
    root = ET.parse(base).getroot()
    links = {x.attrib["name"] for x in root.findall("link")}
    for s in sides(side):
        if s not in config or config[s].parent_link not in links:
            raise ValueError(f"{s}.parent_link must name a link in {base}")
    for mesh in root.findall(".//mesh"):
        path = (base.parent / mesh.attrib["filename"]).resolve()
        if not path.is_file():
            raise FileNotFoundError(f"Missing upstream mesh: {path}")
        mesh.set("filename", str(path))
    flatten_mimics(root)
    for s in sides(side):
        add_camera(root, s, config[s], assets)
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def load_model(config: dict[str, WristCamera], side: str = "both", **kwargs) -> yourdfpy.URDF:
    return yourdfpy.URDF.load(BytesIO(augmented_xml(config, side, **kwargs)),
                              build_collision_scene_graph=True, load_collision_meshes=True)


def update_camera(model: yourdfpy.URDF, side: str, config: WristCamera) -> None:
    """Refresh fixed edges explicitly: yourdfpy's actuated update skips fixed joints."""
    for child, pose in ((f"{side}_camera_mount_link", config.mount), (f"{side}_d405_link", config.camera)):
        joint = model.joint_map[f"{child}_joint"]
        joint.origin = pose.matrix()
        for scene in (model.scene, model.collision_scene):
            if scene is not None:
                scene.graph.update(frame_from=joint.parent, frame_to=joint.child, matrix=joint.origin)


def flatten_mimics(root) -> None:
    """Resolve chained mimic equations for yourdfpy, preserving their exact meaning."""
    joints = {j.attrib["name"]: j for j in root.findall("joint")}
    for joint in joints.values():
        mimic = joint.find("mimic")
        if mimic is None:
            continue
        target = mimic.attrib["joint"]
        multiplier = float(mimic.get("multiplier", "1"))
        offset = float(mimic.get("offset", "0"))
        visited = {joint.attrib["name"]}
        while joints[target].find("mimic") is not None:
            if target in visited:
                raise ValueError("Cycle in mimic joints")
            visited.add(target)
            parent = joints[target].find("mimic")
            offset += multiplier * float(parent.get("offset", "0"))
            multiplier *= float(parent.get("multiplier", "1"))
            target = parent.attrib["joint"]
        mimic.attrib.update(joint=target, multiplier=str(multiplier), offset=str(offset))


def load_base() -> yourdfpy.URDF:
    root = ET.parse(BASE_URDF).getroot()
    for mesh in root.findall(".//mesh"):
        mesh.set("filename", str((BASE_URDF.parent / mesh.attrib["filename"]).resolve()))
    flatten_mimics(root)
    return yourdfpy.URDF.load(BytesIO(ET.tostring(root)), build_collision_scene_graph=True,
                              load_collision_meshes=True)
