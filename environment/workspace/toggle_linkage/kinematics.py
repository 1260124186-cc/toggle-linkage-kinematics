"""Velocity and acceleration helpers for the linkage."""

from __future__ import annotations

import math


def solve_velocity(
    mechanism: dict,
    state: dict,
    crank_speed_rad_s: float,
) -> dict[str, float]:
    """Estimate link speeds from the crank projection."""
    crank = float(mechanism["crank_m"])
    coupler = float(mechanism["coupler_m"])
    rocker = float(mechanism["rocker_m"])
    theta2 = float(state["crank_angle_rad"])
    theta3 = float(state["coupler_angle_rad"])
    theta4 = float(state["rocker_angle_rad"])
    coupler_speed = crank * crank_speed_rad_s * math.cos(theta2 - theta3) / coupler
    rocker_speed = crank * crank_speed_rad_s * math.sin(theta2 - theta4) / rocker
    return {
        "coupler_omega_rad_s": coupler_speed,
        "rocker_omega_rad_s": rocker_speed,
    }


def solve_acceleration(
    mechanism: dict,
    state: dict,
    velocity: dict,
    crank_speed_rad_s: float,
    crank_acceleration_rad_s2: float,
) -> dict[str, float]:
    """Return link accelerations for a steady operating condition."""
    return {
        "coupler_alpha_rad_s2": 0.0,
        "rocker_alpha_rad_s2": 0.0,
    }


def transmission_angle_deg(state: dict) -> float:
    """Return the angle between the coupler and rocker directions."""
    difference = float(state["rocker_angle_rad"]) - float(state["coupler_angle_rad"])
    return math.degrees(abs(math.atan2(math.sin(difference), math.cos(difference))))
