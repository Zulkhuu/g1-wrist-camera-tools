"""GUI binding only; poses, configuration and kinematics live elsewhere."""
import logging
import math
from ..model.poses import joint_limits, reset_pose
from ..model.transforms import Transform


def build_gui(app, side):
    gui = app.server.gui
    # Viser's shared input layout reserves a narrow label column even for
    # checkboxes. Let their labels fill the row, keeping only the box at right.
    # Scope to checkbox rows so sliders and numeric inputs keep their layout.
    gui.add_html("""
    <style>
      .mantine-Flex-root:has(> div > .mantine-Checkbox-root) > :first-child {
        width: auto !important;
        flex: 1 1 0 !important;
        min-width: 0;
      }
      .mantine-Flex-root:has(> div > .mantine-Checkbox-root) > :last-child {
        flex: 0 0 auto !important;
      }
    </style>
    """)
    gui.add_markdown(f"## G1 Wrist Camera Viewer\nG1 29DOF + Inspire FTP · **{side}**")
    with gui.add_folder("Display"):
        robot = gui.add_checkbox("Robot visual geometry", True)
        collision = gui.add_checkbox("Collision geometry", False)
        frames = gui.add_checkbox("Coordinate frames", True)
        camera = gui.add_checkbox("Wrist camera visual geometry", True)
        axis = gui.add_checkbox("Camera optical axis (+Z)", True)
        frustum = gui.add_checkbox("Camera frustum", True)

        def display(_):
            with app.lock:
                app.robot.visual.show_visual = robot.value
                app.robot.visual.show_collision = collision.value
                app.camera.visibility(frames.value, camera.value, axis.value, frustum.value, collision.value)
        for handle in (robot, collision, frames, camera, axis, frustum):
            handle.on_update(display)
    sliders = {}
    with gui.add_folder("Robot joints (radians; prismatic in metres)"):
        reset = gui.add_button("Reset Pose")
        limits = joint_limits(app.model)
        groups = {"Waist": [], "Left arm": [], "Right arm": [], "Left hand": [],
                  "Right hand": [], "Left leg": [], "Right leg": [], "Other": []}
        for name in limits:
            low = name.lower()
            if low.startswith("waist"): group = "Waist"
            elif low.startswith("left_shoulder") or low.startswith("left_elbow") or low.startswith("left_wrist"): group = "Left arm"
            elif low.startswith("right_shoulder") or low.startswith("right_elbow") or low.startswith("right_wrist"): group = "Right arm"
            elif low.startswith("left_hip") or low.startswith("left_knee") or low.startswith("left_ankle"): group = "Left leg"
            elif low.startswith("right_hip") or low.startswith("right_knee") or low.startswith("right_ankle"): group = "Right leg"
            elif low.startswith("left_"): group = "Left hand"
            elif low.startswith("right_"): group = "Right hand"
            else: group = "Other"
            groups[group].append(name)
        for group, names in groups.items():
            if not names: continue
            with gui.add_folder(group):
                for name in names:
                    lo, hi = limits[name]
                    slider = gui.add_slider(name, min=lo, max=hi, step=.001, initial_value=app.joints[name])
                    sliders[name] = slider

                    def joint_changed(event, name=name):
                        with app.lock:
                            app.joints[name] = event.target.value
                            app.update()
                    slider.on_update(joint_changed)

        @reset.on_click
        def reset_clicked(_):
            with app.lock:
                app.joints = reset_pose(app.model)
                for name, value in app.joints.items():
                    sliders[name].value = value
                app.update()
    controls = {}
    for s in app.selected:
        with gui.add_folder(f"{s.title()} camera transforms"):
            for part, title in (("mount", "Wrist → mount"), ("camera", "Mount → D405")):
                with gui.add_folder(title):
                    pose = getattr(app.config[s], part)
                    numbers = [gui.add_number(label, initial_value=value, step=.001)
                               for label, value in zip(("X (m)", "Y (m)", "Z (m)"), pose.xyz)]
                    numbers += [gui.add_number(label, initial_value=math.degrees(value) % 360,
                                               min=0., max=360., step=.1)
                                for label, value in zip(("Roll (deg)", "Pitch (deg)", "Yaw (deg)"), pose.rpy)]
                    controls[s, part] = numbers

                    def changed(_, s=s, part=part, numbers=numbers):
                        with app.lock:
                            if app.syncing_controls:
                                return
                            try:
                                app.set_transform(s, part, Transform(tuple(h.value for h in numbers[:3]), tuple(math.radians(h.value) for h in numbers[3:])))
                                status.content = "Transform updated (unsaved)."
                            except ValueError as exc:
                                status.content = f"**Error:** {exc}"
                    for number in numbers:
                        number.on_update(changed)
    app.syncing_controls = False
    save = gui.add_button("Save transforms")
    reload = gui.add_button("Reload transforms")
    status = gui.add_markdown(f"Configuration: `{app.path.name}`")

    @save.on_click
    def save_clicked(_):
        try:
            app.save()
            status.content = f"Saved to `{app.path}`"
        except (OSError, ValueError) as exc:
            logging.exception("Save failed")
            status.content = f"**Save failed:** {exc}"

    @reload.on_click
    def reload_clicked(_):
        try:
            with app.lock:
                app.reload()
                app.syncing_controls = True
                try:
                    for (s, part), numbers in controls.items():
                        pose = getattr(app.config[s], part)
                        for h, value in zip(numbers, pose.xyz + tuple(math.degrees(v) % 360 for v in pose.rpy)):
                            h.value = value
                finally:
                    app.syncing_controls = False
                status.content = "Reloaded transforms."
        except (OSError, ValueError) as exc:
            logging.exception("Reload failed")
            status.content = f"**Reload failed:** {exc}"
    app.sliders = sliders
    app.transform_controls = controls
