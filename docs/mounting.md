# Installing custom mount assets

Copy the exported meshes to either the flat paths or the organized per-side paths:

* `assets/wrist_camera_mount/g1_d405_wrist_mount_left.stl`
* `assets/wrist_camera_mount/g1_d405_wrist_mount_right.stl`
* `assets/wrist_camera_mount/left/g1_d405_wrist_mount_left.stl`
* `assets/wrist_camera_mount/right/g1_d405_wrist_mount_right.stl`

The matching STEP files are useful source CAD and may remain beside the STL. The current viewer uses STL because `yourdfpy`/`trimesh` can load it directly. The supplied `assets/realsense_d405/D405_Solid.SLDPRT` is also retained as source CAD; SolidWorks Part files are not directly renderable by the Python mesh loader, the viewer loads the supplied `assets/realsense_d405/D405.stl` (also accepting lowercase `d405.stl`), with a cuboid fallback if neither exists.

Mount placement is edited in the Viser GUI and saved to `config/camera_mounts.yaml`. RPY values are radians internally.

## Collision asset workflow

```text
CAD
 ↓
export mount STL
 ↓
assets/wrist_camera_mount/<side>/<name>.stl
 ↓
uv run python tools/generate_collision_mesh.py <path-to-STL>
 ↓
assets/wrist_camera_mount/<side>/<name>_collision.stl
 ↓
URDF mount link
 ├── visual    -> detailed STL
 └── collision -> convex STL
```

Both existing wrist mounts have generated collision assets next to their detailed STLs. Generate the corresponding hull whenever adding or replacing either side's CAD. Use `--force` for regeneration after CAD changes and commit the resulting STL with its source asset. The generic `mount.stl` convention remains supported and derives `mount_collision.stl`; no additional YAML paths are needed.

V0.1 generates one exact convex hull of the mount alone, preserving every input coordinate convention. It does not recenter, normalize, rotate, or convert STL units. Visual and collision meshes share the same mount link, wrist-to-mount transform, and `mesh_scale`; they must remain in exactly the same local coordinate frame. Do not independently reposition either STL. Scene inputs retain their existing instance transforms when combined.

A hull fills holes and concave regions and can substantially overestimate occupied volume. The generator prints statistics and the hull/original volume ratio; unavailable source volumes are explicitly reported. Future convex decomposition could improve highly concave mounts, but V0.1 does not implement it or collision queries/self-collision checking.

The D405 remains a separate link and geometry with a bounds-based box collision approximation, never part of the mount hull. To compare alignment, enable **Display → Collision geometry** in Viser and toggle **Wrist camera visual geometry**; both can be displayed together. Both representations follow live mount transforms and joint updates.

If a detailed mount exists without its collision file, URDF augmentation fails with the command needed to generate it. An absent custom mount still permits the frames-only development fallback.
