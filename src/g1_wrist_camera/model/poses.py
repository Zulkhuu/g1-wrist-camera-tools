"""Offline named poses, independently usable by future state adapters."""
import math
import numpy as np


def joint_limits(model) -> dict[str, tuple[float, float]]:
    limits = {}
    for joint in model.actuated_joints:
        lo = joint.limit.lower if joint.limit else None
        hi = joint.limit.upper if joint.limit else None
        if lo is None or hi is None or not np.isfinite([lo, hi]).all() or lo >= hi:
            lo, hi = (-0.1, 0.1) if joint.type == "prismatic" else (-math.pi, math.pi)
        limits[joint.name] = (float(lo), float(hi))
    return limits


def reset_pose(model) -> dict[str, float]:
    return {name: float(np.clip(0., lo, hi)) for name, (lo, hi) in joint_limits(model).items()}
