# Coordinate frames

The project uses `T_parent_child` to mean a homogeneous transform that maps a point expressed in `child` coordinates into `parent` coordinates. URDF origins and Viser poses follow this convention. Mount configuration values are metres and radians, with fixed-axis XYZ roll, pitch, yaw.

Each wrist has `wrist_yaw_link → camera_mount_link → d405_link`. The D405 body frame is the physical camera frame. Separate `color_frame` and `depth_frame` are fixed to that body. Their optical frames use the ROS convention: +Z points along the optical axis, +X points right in the image, and +Y points down. The viewer draws that +Z axis and an approximate frustum.

Frustum intrinsics in `config/camera_intrinsics.yaml` are visualization defaults, not factory calibration. Real intrinsics will come from `pyrealsense2` in a later version.

The current D405 optical transform uses a fixed −90° roll on each optical child, mapping optical +Z to the CAD body’s local +Y, optical +X to local +X, and optical +Y to local −Z. The configured mount-to-camera pose places the CAD body directly; the optical child transform does not rotate the mesh.
