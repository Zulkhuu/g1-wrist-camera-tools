"""Validated, user-editable configuration; no viewer dependencies."""
from dataclasses import dataclass
from pathlib import Path
import math
import os
import tempfile
import yaml
from .model.transforms import Transform, vector3

ROOT = Path(__file__).resolve().parents[2]
BASE_URDF = ROOT / "assets/unitree/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf"
DEFAULT_MOUNTS = ROOT / "config/camera_mounts.yaml"
DEFAULT_INTRINSICS = ROOT / "config/camera_intrinsics.yaml"


def sides(side: str) -> tuple[str, ...]:
    if side not in ("left", "right", "both"):
        raise ValueError(f"Unsupported side {side!r}; expected left, right, or both")
    return ("left", "right") if side == "both" else (side,)


@dataclass(frozen=True)
class WristCamera:
    parent_link: str
    mount: Transform
    camera: Transform
    mesh_scale: tuple[float, float, float] = (1., 1., 1.)

    def __post_init__(self):
        if not isinstance(self.parent_link, str) or not self.parent_link:
            raise ValueError("parent_link must be a nonempty link name")
        object.__setattr__(self, "mesh_scale", vector3(self.mesh_scale, "mesh_scale"))
        if min(self.mesh_scale) <= 0:
            raise ValueError("mesh_scale must be positive")


def load_mounts(path: Path = DEFAULT_MOUNTS) -> dict[str, WristCamera]:
    try:
        data = yaml.safe_load(path.read_text())
        if not isinstance(data, dict) or set(data) != {"left", "right"}:
            raise ValueError("Expected left and right mappings")
        result = {}
        for side, entry in data.items():
            result[side] = WristCamera(entry["parent_link"], Transform(**entry["mount"]),
                                       Transform(**entry["camera"]), entry.get("mesh_scale", (1., 1., 1.)))
        return result
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise ValueError(f"Invalid mount configuration {path}: {exc}") from exc


def save_mounts(config: dict[str, WristCamera], path: Path = DEFAULT_MOUNTS) -> None:
    """Atomically save both sides, retaining exact float values; comments are normalized."""
    data = {side: {"parent_link": c.parent_link, "mount": c.mount.to_dict(),
                   "camera": c.camera.to_dict(), "mesh_scale": list(c.mesh_scale)}
            for side, c in config.items()}
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as f:
            temporary = Path(f.name)
            yaml.safe_dump(data, f, sort_keys=False)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self):
        for name in ("width", "height", "fx", "fy", "cx", "cy"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"d405.{name} must be finite numeric")
        if any(getattr(self, x) <= 0 for x in ("width", "height", "fx", "fy")):
            raise ValueError("Image dimensions and focal lengths must be positive")
        if int(self.width) != self.width or int(self.height) != self.height:
            raise ValueError("Image dimensions must be integers")
        if not (0 <= self.cx <= self.width and 0 <= self.cy <= self.height):
            raise ValueError("Principal point must be within the image")


def load_intrinsics(path: Path = DEFAULT_INTRINSICS) -> Intrinsics:
    try:
        return Intrinsics(**yaml.safe_load(path.read_text())["d405"])
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise ValueError(f"Invalid intrinsics configuration {path}: {exc}") from exc
