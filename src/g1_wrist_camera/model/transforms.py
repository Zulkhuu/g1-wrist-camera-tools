"""T_parent_child maps child coordinates into parent coordinates (column vectors)."""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation


def vector3(value, name: str) -> tuple[float, float, float]:
    try:
        a = np.asarray(value, dtype=float)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{name} must contain three finite numbers") from exc
    if a.shape != (3,) or not np.isfinite(a).all():
        raise ValueError(f"{name} must contain three finite numbers")
    return tuple(float(x) for x in a)


@dataclass(frozen=True)
class Transform:
    """Translation in metres; fixed-axis roll, pitch, yaw in radians, Rz Ry Rx."""
    xyz: tuple[float, float, float] = (0., 0., 0.)
    rpy: tuple[float, float, float] = (0., 0., 0.)

    def __post_init__(self):
        object.__setattr__(self, "xyz", vector3(self.xyz, "xyz"))
        object.__setattr__(self, "rpy", vector3(self.rpy, "rpy"))

    def matrix(self) -> np.ndarray:
        t = np.eye(4)
        t[:3, :3] = Rotation.from_euler("xyz", self.rpy).as_matrix()
        t[:3, 3] = self.xyz
        return t

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> "Transform":
        t = np.asarray(matrix, dtype=float)
        if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1])
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3))
            or not np.isclose(np.linalg.det(t[:3, :3]), 1)):
            raise ValueError("Expected a finite rigid homogeneous 4x4 transform")
        return cls(tuple(t[:3, 3]), tuple(Rotation.from_matrix(t[:3, :3]).as_euler("xyz")))

    def viser(self) -> dict:
        xyzw = Rotation.from_euler("xyz", self.rpy).as_quat()
        return {"position": self.xyz, "wxyz": xyzw[[3, 0, 1, 2]]}

    def urdf(self) -> dict[str, str]:
        return {"xyz": " ".join(map(str, self.xyz)), "rpy": " ".join(map(str, self.rpy))}

    def to_dict(self) -> dict:
        return {"xyz": list(self.xyz), "rpy": list(self.rpy)}


# ROS body (+X forward, +Y left, +Z up) -> optical (+Z forward, +X right, +Y down).
BODY_OPTICAL = Transform(rpy=(-np.pi / 2, 0., -np.pi / 2))

# Supplied D405 CAD: front is +Y; optical +X is CAD +X, +Y is CAD -Z.
# Apply to optical children only, preserving the configured mesh placement.
D405_OPTICAL = Transform(rpy=(-np.pi / 2, 0., 0.))
