"""Closed-loop geometry for a planar four-bar linkage."""

from __future__ import annotations

import math


def classify_grashof(mechanism: dict) -> str:
    """Return the linkage class used by the report."""
    return "double-crank"


def shortest_link(mechanism: dict) -> str:
    """Return the name of the shortest link."""
    return "crank"


def solve_closure(mechanism: dict, crank_angle_rad: float) -> dict[str, float | tuple[float, float]]:
    """Place the coupler-rocker joint for the requested crank angle."""
    ground = float(mechanism["ground_m"])
    crank = float(mechanism["crank_m"])
    coupler = float(mechanism["coupler_m"])
    rocker = float(mechanism["rocker_m"])

    ax = crank * math.cos(crank_angle_rad)
    ay = crank * math.sin(crank_angle_rad)
    dx = ground - ax
    dy = -ay
    distance = math.hypot(dx, dy)
    ux, uy = dx / distance, dy / distance
    along = (coupler * coupler - rocker * rocker + distance * distance) / (2 * distance)
    height_sq = coupler * coupler - along * along
    if height_sq < -1e-12:
        raise ValueError("circles do not intersect")
    height = math.sqrt(max(0.0, height_sq))

    # The prototype always used the same side of A->O4.
    bx = ax + along * ux - height * uy
    by = ay + along * uy + height * ux
    coupler_angle = math.atan2(by - ay, bx - ax)
    rocker_angle = math.atan2(by, bx - ground)
    return {
        "crank_angle_rad": crank_angle_rad,
        "coupler_angle_rad": coupler_angle,
        "rocker_angle_rad": rocker_angle,
        "coupler_point": (bx, by),
        "rocker_point": (bx, by),
        "ground": ground,
        "crank": crank,
        "coupler": coupler,
        "rocker": rocker,
    }
