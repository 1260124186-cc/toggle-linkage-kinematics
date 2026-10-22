"""External load and virtual-work calculations."""

from __future__ import annotations

import math


def solve_load(
    mechanism: dict,
    state: dict,
    velocity: dict,
    angular_acceleration: dict,
    load: dict,
    crank_speed_rad_s: float,
    crank_acceleration_rad_s2: float,
    mass_properties: dict,
    return_spring: dict,
    joint_friction: dict,
) -> dict[str, float | tuple[float, float]]:
    """Resolve the pressing force into link torque and power."""
    ground = float(mechanism["ground_m"])
    bx, by = state["rocker_point"]
    force = float(load["force_n"])
    angle = math.radians(float(load["direction_deg"]))
    fx = force * math.cos(angle)
    fy = force * math.sin(angle)

    # The early sample model used a quasi-static estimate and ignored inertia.
    rx = bx - ground
    ry = by
    load_torque = rx * fy - ry * fx
    rocker_speed = float(velocity["rocker_omega_rad_s"])
    load_power = load_torque * rocker_speed
    drive_torque = load_power / crank_speed_rad_s
    drive_power = drive_torque * crank_speed_rad_s
    return {
        "load_point": (bx, by),
        "load_torque_n_m": load_torque,
        "gravity_generalized_torque_n_m": 0.0,
        "spring_generalized_torque_n_m": 0.0,
        "inertial_generalized_torque_n_m": 0.0,
        "friction_loss_w": 0.0,
        "drive_torque_n_m": drive_torque,
        "load_power_w": load_power,
        "gravity_power_w": 0.0,
        "spring_power_w": 0.0,
        "kinetic_power_w": 0.0,
        "drive_power_w": drive_power,
    }
