"""Velocity and acceleration analysis for a planar four-bar linkage."""

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
    """Solve a 2x2 linear system when its determinant is nonzero."""
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
    """Solve the first derivative of the closure equation."""
    crank = float(mechanism["crank_m"])
    coupler = float(mechanism["coupler_m"])
    rocker = float(mechanism["rocker_m"])
    theta2 = float(state["crank_angle_rad"])
    theta3 = float(state["coupler_angle_rad"])
    theta4 = float(state["rocker_angle_rad"])
    omega2 = float(crank_speed_rad_s)

    omega3, omega4 = _solve_2x2(
        -coupler * math.sin(theta3),
        rocker * math.sin(theta4),
        coupler * math.cos(theta3),
        -rocker * math.cos(theta4),
        crank * math.sin(theta2) * omega2,
        -crank * math.cos(theta2) * omega2,
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
    """Solve the second derivative of the closure equation."""
    crank = float(mechanism["crank_m"])
    coupler = float(mechanism["coupler_m"])
    rocker = float(mechanism["rocker_m"])
    theta2 = float(state["crank_angle_rad"])
    theta3 = float(state["coupler_angle_rad"])
    theta4 = float(state["rocker_angle_rad"])
    omega2 = float(crank_speed_rad_s)
    omega3 = float(velocity["coupler_omega_rad_s"])
    omega4 = float(velocity["rocker_omega_rad_s"])
    alpha2 = float(crank_acceleration_rad_s2)

    s2, c2 = math.sin(theta2), math.cos(theta2)
    s3, c3 = math.sin(theta3), math.cos(theta3)
    s4, c4 = math.sin(theta4), math.cos(theta4)
    alpha3, alpha4 = _solve_2x2(
        -coupler * s3,
        rocker * s4,
        coupler * c3,
        -rocker * c4,
        crank * s2 * alpha2
        + crank * c2 * omega2 * omega2
        + coupler * c3 * omega3 * omega3
        - rocker * c4 * omega4 * omega4,
        -crank * c2 * alpha2
        + crank * s2 * omega2 * omega2
        + coupler * s3 * omega3 * omega3
        - rocker * s4 * omega4 * omega4,
    )
    return {
        "coupler_alpha_rad_s2": alpha3,
        "rocker_alpha_rad_s2": alpha4,
    }


def transmission_angle_deg(state: dict) -> float:
    """Return the acute angle between coupler and rocker lines."""
    difference = float(state["rocker_angle_rad"]) - float(state["coupler_angle_rad"])
    wrapped = abs(math.atan2(math.sin(difference), math.cos(difference)))
    return math.degrees(min(wrapped, math.pi - wrapped))
