"""Velocity-only experiment for the second authoring challenge."""

from __future__ import annotations

import math


def _solve_2x2(
    a11: float,
    a12: float,
    a21: float,
    a22: float,
    b1: float,
    b2: float,
) -> tuple[float, float]:
    """Solve a 2x2 system for the velocity experiment."""
    determinant = a11 * a22 - a12 * a21
    if abs(determinant) < 1e-14:
        raise ValueError("linkage velocity matrix is singular")
    return (
        (b1 * a22 - a12 * b2) / determinant,
        (a11 * b2 - b1 * a21) / determinant,
    )


def solve_velocity(
    mechanism: dict,
    state: dict,
    crank_speed_rad_s: float,
) -> dict[str, float]:
    """Solve only the first derivative of the closure equation."""
    crank = float(mechanism["crank_m"])
    coupler = float(mechanism["coupler_m"])
    rocker = float(mechanism["rocker_m"])
    theta2 = float(state["crank_angle_rad"])
    theta3 = float(state["coupler_angle_rad"])
    theta4 = float(state["rocker_angle_rad"])
    omega3, omega4 = _solve_2x2(
        -coupler * math.sin(theta3),
        rocker * math.sin(theta4),
        coupler * math.cos(theta3),
        -rocker * math.cos(theta4),
        crank * math.sin(theta2) * crank_speed_rad_s,
        -crank * math.cos(theta2) * crank_speed_rad_s,
    )
    return {
        "coupler_omega_rad_s": omega3,
        "rocker_omega_rad_s": omega4,
    }


def solve_acceleration(
    mechanism: dict,
    state: dict,
    velocity: dict,
    crank_speed_rad_s: float,
    crank_acceleration_rad_s2: float,
) -> dict[str, float]:
    """Keep the original steady-state acceleration shortcut."""
    return {
        "coupler_alpha_rad_s2": 0.0,
        "rocker_alpha_rad_s2": 0.0,
    }


def transmission_angle_deg(state: dict) -> float:
    """Keep the original unsigned angle difference."""
    difference = float(state["rocker_angle_rad"]) - float(state["coupler_angle_rad"])
    return math.degrees(abs(math.atan2(math.sin(difference), math.cos(difference))))
