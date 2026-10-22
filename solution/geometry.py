"""Correct closed-loop geometry for the planar four-bar linkage."""

from __future__ import annotations

import math


LINK_NAMES = ("ground", "crank", "coupler", "rocker")


def shortest_link(mechanism: dict) -> str:
    """Return the link with the smallest length."""
    lengths = {
        "ground": float(mechanism["ground_m"]),
        "crank": float(mechanism["crank_m"]),
        "coupler": float(mechanism["coupler_m"]),
        "rocker": float(mechanism["rocker_m"]),
    }
    return min(LINK_NAMES, key=lambda name: lengths[name])


def classify_grashof(mechanism: dict) -> str:
    """Classify the mechanism using the Grashof inequality."""
    lengths = {
        "ground": float(mechanism["ground_m"]),
        "crank": float(mechanism["crank_m"]),
        "coupler": float(mechanism["coupler_m"]),
        "rocker": float(mechanism["rocker_m"]),
    }
    ordered = sorted(lengths.values())
    grashof = ordered[0] + ordered[3] <= ordered[1] + ordered[2] + 1e-12
    if grashof and min(lengths, key=lengths.get) == "crank":
        return "crank-rocker"
    return "other"


def solve_closure(mechanism: dict, crank_angle_rad: float) -> dict:
    """Return both moving joints for the selected assembly branch."""
    ground = float(mechanism["ground_m"])
    crank = float(mechanism["crank_m"])
    coupler = float(mechanism["coupler_m"])
    rocker = float(mechanism["rocker_m"])
    branch = int(mechanism["branch"])

    ax = crank * math.cos(crank_angle_rad)
    ay = crank * math.sin(crank_angle_rad)
    dx = ground - ax
    dy = -ay
    distance = math.hypot(dx, dy)
    if distance == 0.0:
        raise ValueError("crank joint coincides with the fixed pivot")
    ux, uy = dx / distance, dy / distance
    along = (coupler * coupler - rocker * rocker + distance * distance) / (
        2.0 * distance
    )
    height_sq = coupler * coupler - along * along
    scale = max(coupler * coupler, 1.0)
    if height_sq < -1e-12 * scale:
        raise ValueError("coupler and rocker circles do not intersect")
    height = math.sqrt(max(0.0, height_sq))
    perp_x, perp_y = -uy, ux
    bx = ax + along * ux + branch * height * perp_x
    by = ay + along * uy + branch * height * perp_y

    return {
        "crank_angle_rad": crank_angle_rad,
        "coupler_angle_rad": math.atan2(by - ay, bx - ax),
        "rocker_angle_rad": math.atan2(by, bx - ground),
        "coupler_point": (bx, by),
        "rocker_point": (bx, by),
        "ground": ground,
        "crank": crank,
        "coupler": coupler,
        "rocker": rocker,
    }
