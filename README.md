# G1 Wrist Camera Tools

Tools for integrating and visualizing wrist-mounted cameras on the Unitree G1, currently based on the G1 29-DOF model with Inspire FTP hands and Intel RealSense D405 cameras. The project combines robot descriptions, custom mount assets, and an interactive viewer, with camera calibration and related tooling planned for future development.

## Current features

- Unitree G1 29-DOF + Inspire FTP URDF integration; upstream files remain unchanged.
- Custom single-piece D405 wrist mounts with independent left, right, or both camera configurations.
- Detailed visual mount STLs and generated convex-hull collision STLs in separate URDF `<visual>` and `<collision>` elements.
- Reusable collision-mesh generation script with validation, mesh statistics, and overwrite protection.
- Viser robot visualization with joint controls and independent visual/collision display.
- Coordinate frames, camera optical axes, and approximate camera frustums.
- Configurable wrist-to-mount and mount-to-camera transforms, with YAML save/reload.

## Installation

Requires **Python 3.10 or newer**, [uv](https://docs.astral.sh/uv/), and a browser. Run from a checkout of this repository:

```bash
cd g1-wrist-camera-tools
uv sync
```

`uv sync` installs the project and development tools using `uv.lock`. Runtime dependencies are NumPy, SciPy, PyYAML, yourdfpy, trimesh, and Viser; pytest and Ruff are development dependencies. ROS, the Unitree SDK, RealSense drivers, and connected hardware are not required for the offline viewer.

Keep the checkout's `assets/` and `config/` directories alongside the source. The current application uses repository assets and an editable installation; installing the Python wheel alone does not provide a standalone viewer with assets.

## Basic usage

```bash
uv run python scripts/view_model.py --side right
uv run python scripts/view_model.py --side left
uv run python scripts/view_model.py --side both
```

Run one viewer at a time, then open the printed URL (normally `http://127.0.0.1:8080`). Use `--port 8081` for another port or `--config config/camera_mounts.yaml` to select a mount configuration. Save and reload operate on that file. Mount/camera GUI rotation controls use degrees; rotational joint sliders and YAML rotations use radians.

Under **Display**, toggle **Wrist camera visual geometry** and **Collision geometry** to inspect either representation or both together. Disable **Robot visual geometry** as well for a collision-only view. Joint and mount-pose edits update both representations.

Validation:

```bash
uv run pytest
uv run python scripts/view_model.py --side both --smoke-test
```

The smoke test starts the viewer, exercises transforms, and exits. Tests cover transforms, upstream asset checksums, left/right/both URDF loading, mesh resolution, collision generation, and viewer updates. In a sourced ROS shell, unrelated pytest plugins may fail before collection; use `uv run pytest --disable-plugin-autoload` in that case.

## Repository structure

```text
g1-wrist-camera-tools/
├── assets/
│   ├── unitree/                 # Pinned upstream URDF, meshes, license, checksums
│   ├── realsense_d405/          # Camera visual STL and source CAD
│   └── wrist_camera_mount/
│       ├── left/               # Detailed STL, collision STL, source STEP
│       └── right/              # Detailed STL, collision STL, source STEP
├── config/                     # Mount transforms and visualization intrinsics
├── docs/                       # Mounting and coordinate-frame conventions
├── scripts/view_model.py
├── tools/generate_collision_mesh.py
├── src/g1_wrist_camera/
│   ├── model/                  # URDF augmentation, transforms, collision utilities
│   └── viewer/                 # Viser geometry and controls
├── tests/
├── pyproject.toml
└── uv.lock
```

## Custom wrist-mount assets

Each printed mount is one rigid STL part. Both sides are included under `assets/wrist_camera_mount/<side>/`:

```text
g1_d405_wrist_mount_<side>.stl            -> URDF visual geometry
g1_d405_wrist_mount_<side>_collision.stl  -> URDF collision geometry
```

STEP files are retained as source CAD. The loader also supports the existing flat mount layout and per-side `mount.stl` naming. Collision filenames are derived from the selected visual filename, so no duplicate YAML path configuration is needed.

An existing mount STL requires its collision STL; missing collision geometry produces an error with the generation command. If the mount itself is absent, the viewer retains coordinate frames. The D405 body stays on its own link, with a separate visual STL and a bounds-based collision box; an absent D405 mesh uses a box placeholder. See [mounting](docs/mounting.md).

## Collision mesh generation

```bash
uv run python tools/generate_collision_mesh.py \
    assets/wrist_camera_mount/right/g1_d405_wrist_mount_right.stl
```

This creates the adjacent `g1_d405_wrist_mount_right_collision.stl`. Both current collision files are already included, so add **`--force`** to regenerate them after changing CAD. Use the corresponding left path for the other wrist. The utility also accepts `--output path/to/custom_collision.stl` and `--method convex-hull`; URDF integration expects the default `<visual-stem>_collision.stl` name.

The generator uses one exact convex hull. It does **not recenter, rescale, rotate, or convert units**. Both meshes share the mount link transform and YAML `mesh_scale`. Commit generated collision STLs alongside the detailed assets whenever CAD changes.

Counts, bounds, watertightness, volumes, and the hull/original volume ratio are reported; unreliable source volumes are marked unavailable. A single hull is a conservative approximation that fills holes and concavities. Collision geometry can be displayed, but collision queries and avoidance are not implemented.

## Coordinate-frame conventions

- `T_parent_child` maps points from child coordinates into parent coordinates.
- Translations use metres; YAML rotations use fixed-axis XYZ roll, pitch, yaw in radians (`Rz @ Ry @ Rx`).
- The chain is `wrist_yaw_link → camera_mount_link → d405_link → sensor/optical frames`.
- Optical frames use +Z forward, +X image-right, and +Y image-down.
- The current D405 optical transform maps optical +Z to the CAD body's local +Y.

Frustum intrinsics in `config/camera_intrinsics.yaml` are visualization defaults, not factory or calibrated intrinsics. Mount poses are editable placements, not calibration results. See [coordinate frames](docs/coordinate_frames.md).

## Current hardware support

Currently tested with the **Unitree G1 29-DOF, Inspire FTP hands, and Intel RealSense D405** model/assets. Validation currently covers offline descriptions and visualization, not live robot or camera operation. Other hands and cameras are possible future extensions; they are not yet provided as selectable configurations.

## TODO / roadmap

- Live Unitree joint states via `unitree_sdk2_python`.
- Live RealSense streams and factory intrinsics.
- ChArUco target detection and hand-eye calibration.
- Calibration dataset capture and calibration-quality visualization.
- Calibrated transform export.
- Improved collision approximation through convex decomposition, such as CoACD or V-HACD.
- Self-collision checking, with potential Pinocchio and HPP-FCL integration.

## Upstream attribution and licensing

The G1 URDF and meshes in `assets/unitree/` originate from [Unitree Robotics' unitree_ros repository](https://github.com/unitreerobotics/unitree_ros). The exact URDF source is [`robots/g1_description/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf`](https://github.com/unitreerobotics/unitree_ros/blob/7d6075f7f58588b189b940130e3edab3c839b2df/robots/g1_description/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf), pinned to commit `7d6075f7f58588b189b940130e3edab3c839b2df`.

The original **BSD-3-Clause** license is retained in [assets/unitree/LICENSE](assets/unitree/LICENSE), with provenance in [assets/unitree/README.md](assets/unitree/README.md). These are upstream assets, not original project work. URDF augmentation happens in memory; the vendored source stays unchanged.

No root project license has been selected yet. The Unitree license applies to its vendored content and does not assign a license to this project's code or custom assets. Before publication, select a project license and record the ownership, provenance, and redistribution terms for the custom mount and D405 CAD assets.
