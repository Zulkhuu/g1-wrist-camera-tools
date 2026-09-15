"""Robot mesh rendering accepts joint values without knowledge of their source."""
from viser.extras import ViserUrdf
from ..model.urdf import load_base


class RobotView:
    def __init__(self, server):
        base = load_base()
        self.visual = ViserUrdf(server, base, root_node_name="/robot", load_collision_meshes=True)
        self.visual.show_collision = False

    def update(self, joints: dict[str, float]) -> None:
        self.visual.update_cfg(joints)
