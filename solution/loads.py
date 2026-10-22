"""Load, gravity, spring, friction, and rigid-body inertia calculations."""

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
    """Resolve the dynamic generalized force about the crank."""
    ground = float(mechanism["ground_m"])
    crank = float(mechanism["crank_m"])
    rocker = float(mechanism["rocker_m"])
    ratio = float(load["application_ratio"])
    bx, by = state["rocker_point"]
    rx = ratio * (bx - ground)
    ry = ratio * by
    px, py = ground + rx, ry

    force = float(load["force_n"])
    direction = math.radians(float(load["direction_deg"]))
    fx = force * math.cos(direction)
    fy = force * math.sin(direction)
    load_torque = rx * fy - ry * fx
    omega2 = float(crank_speed_rad_s)
    omega3 = float(velocity["coupler_omega_rad_s"])
    omega4 = float(velocity["rocker_omega_rad_s"])
    alpha2 = float(crank_acceleration_rad_s2)
    alpha3 = float(angular_acceleration["coupler_alpha_rad_s2"])
    alpha4 = float(angular_acceleration["rocker_alpha_rad_s2"])
    load_generalized = load_torque * omega4 / omega2
    load_power = load_generalized * omega2

    theta2 = float(state["crank_angle_rad"])
    theta4 = float(state["rocker_angle_rad"])
    dax = -crank * math.sin(theta2)
    day = crank * math.cos(theta2)
    dbx = rocker * (omega4 / omega2) * (-math.sin(theta4))
    dby = rocker * (omega4 / omega2) * math.cos(theta4)

    crank_props = mass_properties["crank"]
    coupler_props = mass_properties["coupler"]
    rocker_props = mass_properties["rocker"]
    gravity = float(mass_properties["gravity_m_s2"])
    crank_ratio = float(crank_props["com_ratio"])
    coupler_ratio = float(coupler_props["com_ratio"])
    rocker_ratio = float(rocker_props["com_ratio"])
    c2x, c2y = crank_ratio * dax, crank_ratio * day
    c3x = (1.0 - coupler_ratio) * dax + coupler_ratio * dbx
    c3y = (1.0 - coupler_ratio) * day + coupler_ratio * dby
    c4x, c4y = rocker_ratio * dbx, rocker_ratio * dby
    gravity_generalized = (
        -float(crank_props["mass_kg"]) * gravity * c2y
        - float(coupler_props["mass_kg"]) * gravity * c3y
        - float(rocker_props["mass_kg"]) * gravity * c4y
    )
    gravity_power = gravity_generalized * omega2

    zero_angle = math.radians(float(return_spring["zero_angle_deg"]))
    spring_torque = -float(return_spring["stiffness_n_m_per_rad"]) * (
        theta4 - zero_angle
    )
    spring_generalized = spring_torque * omega4 / omega2
    spring_power = spring_generalized * omega2

    ax_acc = crank * (-alpha2 * math.sin(theta2) - omega2 * omega2 * math.cos(theta2))
    ay_acc = crank * (alpha2 * math.cos(theta2) - omega2 * omega2 * math.sin(theta2))
    bx_acc = rocker * (
        -alpha4 * math.sin(theta4) - omega4 * omega4 * math.cos(theta4)
    )
    by_acc = rocker * (
        alpha4 * math.cos(theta4) - omega4 * omega4 * math.sin(theta4)
    )
    c2ax, c2ay = crank_ratio * ax_acc, crank_ratio * ay_acc
    c3ax = (1.0 - coupler_ratio) * ax_acc + coupler_ratio * bx_acc
    c3ay = (1.0 - coupler_ratio) * ay_acc + coupler_ratio * by_acc
    c4ax, c4ay = rocker_ratio * bx_acc, rocker_ratio * by_acc
    inertial_generalized = (
        float(crank_props["mass_kg"]) * (c2ax * c2x + c2ay * c2y)
        + float(crank_props["inertia_kg_m2"]) * alpha2
        + float(coupler_props["mass_kg"]) * (c3ax * c3x + c3ay * c3y)
        + float(coupler_props["inertia_kg_m2"])
        * alpha3
        * (omega3 / omega2)
        + float(rocker_props["mass_kg"]) * (c4ax * c4x + c4ay * c4y)
        + float(rocker_props["inertia_kg_m2"])
        * alpha4
        * (omega4 / omega2)
    )
    kinetic_power = inertial_generalized * omega2

    friction_loss = (
        abs(omega2) * float(joint_friction["o2_n_m"])
        + abs(omega3 - omega4) * float(joint_friction["b_n_m"])
        + abs(omega4) * float(joint_friction["o4_n_m"])
    )
    drive_torque = (
        inertial_generalized
        - load_generalized
        - gravity_generalized
        - spring_generalized
        + friction_loss / omega2
    )
    drive_power = drive_torque * omega2
    return {
        "load_point": (px, py),
        "load_torque_n_m": load_torque,
        "gravity_generalized_torque_n_m": gravity_generalized,
        "spring_generalized_torque_n_m": spring_generalized,
        "inertial_generalized_torque_n_m": inertial_generalized,
        "friction_loss_w": friction_loss,
        "drive_torque_n_m": drive_torque,
        "load_power_w": load_power,
        "gravity_power_w": gravity_power,
        "spring_power_w": spring_power,
        "kinetic_power_w": kinetic_power,
        "drive_power_w": drive_power,
    }
