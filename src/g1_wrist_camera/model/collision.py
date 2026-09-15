"""Single-hull collision assets in the source mesh coordinate system.

TODO: highly concave mounts may benefit from approximate convex decomposition.
"""
from pathlib import Path

import numpy as np
import trimesh


def collision_path_for_visual_mesh(path: Path) -> Path:
    path = Path(path)
    return path.with_name(f"{path.stem}_collision.stl")


def _validate_mesh(mesh: trimesh.Trimesh) -> None:
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError("Input must be a triangle mesh")
    if len(mesh.vertices) == 0 or len(mesh.faces) == 0:
        raise ValueError("Triangle mesh must contain vertices and faces")
    if mesh.faces.ndim != 2 or mesh.faces.shape[1] != 3:
        raise ValueError("Mesh faces must be triangles")
    if not np.isfinite(mesh.vertices).all():
        raise ValueError("Mesh vertices must be finite")
    if mesh.faces.min() < 0 or mesh.faces.max() >= len(mesh.vertices):
        raise ValueError("Mesh has invalid face indices")


def load_triangle_mesh(path: Path) -> trimesh.Trimesh:
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"Input mesh not found: {path}")
    try:
        mesh = trimesh.load_mesh(path)
        if isinstance(mesh, trimesh.Scene):
            if any(not isinstance(g, trimesh.Trimesh) for g in mesh.geometry.values()):
                raise ValueError("Scene contains non-triangle geometry")
            # Bake existing scene placements, preserving the scene coordinate frame.
            mesh = mesh.to_mesh()
        _validate_mesh(mesh)
        return mesh
    except Exception as exc:
        raise ValueError(f"Cannot load triangle mesh {path}: {exc}") from exc


def generate_convex_hull(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Return the exact convex hull without recentering, scaling, or rotating."""
    _validate_mesh(mesh)
    try:
        hull = mesh.convex_hull
        _validate_mesh(hull)
        if not hull.is_volume or not hull.is_convex or not np.isfinite(hull.volume):
            raise ValueError("Hull must be a finite, watertight convex solid")
        return hull
    except Exception as exc:
        raise ValueError(f"Cannot generate valid convex hull: {exc}") from exc


def mesh_statistics(mesh: trimesh.Trimesh) -> dict:
    """Volume is unavailable when topology/winding cannot support a solid."""
    volume = float(mesh.volume) if mesh.is_volume else None
    if volume is not None and not np.isfinite(volume):
        volume = None
    return {"vertices": len(mesh.vertices), "triangles": len(mesh.faces),
            "watertight": bool(mesh.is_watertight), "bounds": mesh.bounds.tolist(),
            "volume": volume}


def generate_collision_mesh(input_path: Path, output_path: Path | None = None,
                            *, force: bool = False, method: str = "convex-hull"):
    if method != "convex-hull":
        raise ValueError(f"Unsupported collision method: {method}")
    input_path = Path(input_path)
    output_path = Path(output_path) if output_path is not None else collision_path_for_visual_mesh(input_path)
    if input_path.resolve() == output_path.resolve():
        raise ValueError("Collision output must differ from the detailed input mesh")
    if output_path.suffix.lower() != ".stl":
        raise ValueError("Collision output must have an .stl extension")
    if output_path.exists() and not force:
        raise FileExistsError(f"Collision mesh already exists: {output_path}; use --force to overwrite")
    mesh = load_triangle_mesh(input_path)
    hull = generate_convex_hull(mesh)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    payload = hull.export(file_type="stl")
    # Exclusive creation also protects against another process creating the file.
    with output_path.open("wb" if force else "xb") as output:
        output.write(payload)
    return output_path, mesh_statistics(mesh), mesh_statistics(hull)
