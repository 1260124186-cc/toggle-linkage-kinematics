"""Command-line entry point for the linkage report."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

from .geometry import classify_grashof, shortest_link, solve_closure
from .kinematics import solve_acceleration, solve_velocity, transmission_angle_deg
from .loads import solve_load


def build_report(case: dict) -> dict:
    """Evaluate all requested crank angles."""
    mechanism = case["mechanism"]
    settings = case["input"]
    load = case["load"]
    mass_properties = case["mass_properties"]
    return_spring = case["return_spring"]
    joint_friction = case["joint_friction"]
    speed = float(settings["crank_speed_rad_s"])
    acceleration = float(settings["crank_acceleration_rad_s2"])
    samples = []
    for angle_deg in settings["crank_angles_deg"]:
        theta2 = math.radians(float(angle_deg))
        position = solve_closure(mechanism, theta2)
        velocity = solve_velocity(mechanism, position, speed)
        angular_acceleration = solve_acceleration(
            mechanism, position, velocity, speed, acceleration
        )
        load_result = solve_load(
            mechanism,
            position,
            velocity,
            angular_acceleration,
            load,
            speed,
            acceleration,
            mass_properties,
            return_spring,
            joint_friction,
        )
        bx, by = position["coupler_point"]
        px, py = load_result["load_point"]
        samples.append(
            {
                "crank_angle_deg": float(angle_deg),
                "crank_angle_rad": theta2,
                "coupler_angle_deg": math.degrees(position["coupler_angle_rad"]),
                "rocker_angle_deg": math.degrees(position["rocker_angle_rad"]),
                "coupler_point_m": [bx, by],
                "load_point_m": [px, py],
                "coupler_omega_rad_s": velocity["coupler_omega_rad_s"],
                "rocker_omega_rad_s": velocity["rocker_omega_rad_s"],
                "coupler_alpha_rad_s2": angular_acceleration["coupler_alpha_rad_s2"],
                "rocker_alpha_rad_s2": angular_acceleration["rocker_alpha_rad_s2"],
                "transmission_angle_deg": transmission_angle_deg(position),
                "load_torque_n_m": load_result["load_torque_n_m"],
                "gravity_generalized_torque_n_m": load_result[
                    "gravity_generalized_torque_n_m"
                ],
                "spring_generalized_torque_n_m": load_result[
                    "spring_generalized_torque_n_m"
                ],
                "inertial_generalized_torque_n_m": load_result[
                    "inertial_generalized_torque_n_m"
                ],
                "friction_loss_w": load_result["friction_loss_w"],
                "drive_torque_n_m": load_result["drive_torque_n_m"],
                "load_power_w": load_result["load_power_w"],
                "gravity_power_w": load_result["gravity_power_w"],
                "spring_power_w": load_result["spring_power_w"],
                "kinetic_power_w": load_result["kinetic_power_w"],
                "drive_power_w": load_result["drive_power_w"],
            }
        )
    return {
        "mechanism": {
            "grashof_class": classify_grashof(mechanism),
            "shortest_link": shortest_link(mechanism),
            "branch": int(mechanism["branch"]),
        },
        "samples": samples,
        "summary": {
            "minimum_transmission_angle_deg": min(
                sample["transmission_angle_deg"] for sample in samples
            ),
            "maximum_abs_coupler_speed_rad_s": max(
                abs(sample["coupler_omega_rad_s"]) for sample in samples
            ),
            "maximum_abs_rocker_speed_rad_s": max(
                abs(sample["rocker_omega_rad_s"]) for sample in samples
            ),
            "maximum_abs_drive_torque_n_m": max(
                abs(sample["drive_torque_n_m"]) for sample in samples
            ),
            "maximum_friction_loss_w": max(
                sample["friction_loss_w"] for sample in samples
            ),
            "maximum_abs_kinetic_power_w": max(
                abs(sample["kinetic_power_w"]) for sample in samples
            ),
        },
    }


def main(argv: list[str] | None = None) -> int:
    """Read one case and write the report."""
    arguments = sys.argv[1:] if argv is None else argv
    if len(arguments) != 2:
        print("usage: python -m toggle_linkage.cli CASE.json REPORT.json", file=sys.stderr)
        return 2
    source = Path(arguments[0])
    target = Path(arguments[1])
    try:
        case = json.loads(source.read_text(encoding="utf-8"))
        report = build_report(case)
        target.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return 0
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError, ZeroDivisionError) as exc:
        print(f"calculation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
