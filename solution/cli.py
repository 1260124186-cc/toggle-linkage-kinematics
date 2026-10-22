"""Command-line entry point with validation and atomic output."""

from __future__ import annotations

import json
import math
import os
import sys
import tempfile
from pathlib import Path

from .geometry import classify_grashof, shortest_link, solve_closure
from .kinematics import solve_acceleration, solve_velocity, transmission_angle_deg
from .loads import solve_load


def _finite_number(value, label: str) -> float:
    """Return a finite float or reject the input."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def _validate_case(case: dict) -> tuple[dict, dict, dict, dict, dict, dict]:
    """Validate the complete case before calculations start."""
    if not isinstance(case, dict):
        raise ValueError("case must be an object")
    try:
        mechanism = case["mechanism"]
        settings = case["input"]
        load = case["load"]
        mass_properties = case["mass_properties"]
        return_spring = case["return_spring"]
        joint_friction = case["joint_friction"]
    except KeyError as exc:
        raise ValueError(f"missing root field: {exc.args[0]}") from exc
    objects = (
        mechanism,
        settings,
        load,
        mass_properties,
        return_spring,
        joint_friction,
    )
    if not all(isinstance(item, dict) for item in objects):
        raise ValueError("all root sections must be objects")

    for key in ("ground_m", "crank_m", "coupler_m", "rocker_m"):
        if _finite_number(mechanism.get(key), f"mechanism.{key}") <= 0:
            raise ValueError(f"mechanism.{key} must be positive")
    branch = mechanism.get("branch")
    if isinstance(branch, bool) or branch not in (-1, 1):
        raise ValueError("mechanism.branch must be 1 or -1")

    angles = settings.get("crank_angles_deg")
    if not isinstance(angles, list) or not angles:
        raise ValueError("input.crank_angles_deg must be a nonempty array")
    for index, value in enumerate(angles):
        angle = _finite_number(value, f"input.crank_angles_deg[{index}]")
        if not 0 <= angle < 360:
            raise ValueError("crank angle must be in [0, 360)")
    speed = _finite_number(settings.get("crank_speed_rad_s"), "input.crank_speed_rad_s")
    if speed == 0:
        raise ValueError("crank speed must be nonzero")
    _finite_number(
        settings.get("crank_acceleration_rad_s2"),
        "input.crank_acceleration_rad_s2",
    )

    force = _finite_number(load.get("force_n"), "load.force_n")
    if force < 0:
        raise ValueError("load.force_n must be nonnegative")
    _finite_number(load.get("direction_deg"), "load.direction_deg")
    ratio = _finite_number(load.get("application_ratio"), "load.application_ratio")
    if not 0 < ratio <= 1:
        raise ValueError("load.application_ratio must be in (0, 1]")

    for link in ("crank", "coupler", "rocker"):
        properties = mass_properties.get(link)
        if not isinstance(properties, dict):
            raise ValueError(f"mass_properties.{link} must be an object")
        if _finite_number(
            properties.get("mass_kg"),
            f"mass_properties.{link}.mass_kg",
        ) <= 0:
            raise ValueError(f"mass_properties.{link}.mass_kg must be positive")
        if _finite_number(
            properties.get("inertia_kg_m2"),
            f"mass_properties.{link}.inertia_kg_m2",
        ) <= 0:
            raise ValueError(
                f"mass_properties.{link}.inertia_kg_m2 must be positive"
            )
        com_ratio = _finite_number(
            properties.get("com_ratio"),
            f"mass_properties.{link}.com_ratio",
        )
        if not 0 < com_ratio <= 1:
            raise ValueError(
                f"mass_properties.{link}.com_ratio must be in (0, 1]"
            )
    if _finite_number(
        mass_properties.get("gravity_m_s2"),
        "mass_properties.gravity_m_s2",
    ) <= 0:
        raise ValueError("mass_properties.gravity_m_s2 must be positive")

    stiffness = _finite_number(
        return_spring.get("stiffness_n_m_per_rad"),
        "return_spring.stiffness_n_m_per_rad",
    )
    if stiffness < 0:
        raise ValueError("return_spring.stiffness_n_m_per_rad must be nonnegative")
    _finite_number(
        return_spring.get("zero_angle_deg"),
        "return_spring.zero_angle_deg",
    )
    for key in ("o2_n_m", "b_n_m", "o4_n_m"):
        if _finite_number(joint_friction.get(key), f"joint_friction.{key}") < 0:
            raise ValueError(f"joint_friction.{key} must be nonnegative")
    return (
        mechanism,
        settings,
        load,
        mass_properties,
        return_spring,
        joint_friction,
    )


def build_report(case: dict) -> dict:
    """Evaluate a validated mechanism over the requested angles."""
    (
        mechanism,
        settings,
        load,
        mass_properties,
        return_spring,
        joint_friction,
    ) = _validate_case(case)
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


def _write_atomic(target: Path, report: dict) -> None:
    """Write the finished report through a temporary sibling file."""
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary_name = ""
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=target.parent,
            prefix=f".{target.name}.",
            suffix=".tmp",
            delete=False,
        ) as handle:
            temporary_name = handle.name
            json.dump(report, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, target)
    finally:
        if temporary_name and os.path.exists(temporary_name):
            os.unlink(temporary_name)


def main(argv: list[str] | None = None) -> int:
    """Read one case and write the report atomically."""
    arguments = sys.argv[1:] if argv is None else argv
    if len(arguments) != 2:
        print("usage: python -m toggle_linkage.cli CASE.json REPORT.json", file=sys.stderr)
        return 2
    source = Path(arguments[0])
    target = Path(arguments[1])
    try:
        case = json.loads(source.read_text(encoding="utf-8"))
        report = build_report(case)
        _write_atomic(target, report)
        return 0
    except (
        OSError,
        json.JSONDecodeError,
        KeyError,
        TypeError,
        ValueError,
        ZeroDivisionError,
    ) as exc:
        print(f"calculation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
