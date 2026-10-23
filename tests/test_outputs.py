"""Independent checks for four-bar geometry, motion, and load analysis."""

from __future__ import annotations

import copy
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, "/workspace")


BASE_MECHANISM = {
    "ground_m": 0.55,
    "crank_m": 0.18,
    "coupler_m": 0.62,
    "rocker_m": 0.48,
    "branch": 1,
}
BASE_LOAD = {
    "force_n": 240.0,
    "direction_deg": -90.0,
    "application_ratio": 0.72,
}
BASE_MASS_PROPERTIES = {
    "crank": {"mass_kg": 1.2, "com_ratio": 0.45, "inertia_kg_m2": 0.0108},
    "coupler": {"mass_kg": 0.9, "com_ratio": 0.55, "inertia_kg_m2": 0.0302},
    "rocker": {"mass_kg": 1.8, "com_ratio": 0.62, "inertia_kg_m2": 0.0213},
    "gravity_m_s2": 9.80665,
}
BASE_RETURN_SPRING = {
    "stiffness_n_m_per_rad": 8.5,
    "zero_angle_deg": 25.0,
}
BASE_JOINT_FRICTION = {
    "o2_n_m": 0.6,
    "b_n_m": 0.4,
    "o4_n_m": 0.5,
}


def case_contract(
    case_id: str,
    requirement_id: str,
    test_id: str,
    instruction_quote: str,
):
    """Attach auditable题面 and design-gate metadata to one test."""

    def attach(function):
        """Attach unique behavior metadata without changing test execution."""
        function.case_id = case_id
        function.requirement_id = requirement_id
        function.test_id = test_id
        function.instruction_quote = instruction_quote
        return function

    return attach


def _mechanism(branch: int = 1) -> dict:
    """Return a fresh mechanism dictionary."""
    result = dict(BASE_MECHANISM)
    result["branch"] = branch
    return result


def _solve_2x2(
    a11: float,
    a12: float,
    a21: float,
    a22: float,
    b1: float,
    b2: float,
) -> tuple[float, float]:
    """Solve a nonsingular 2x2 system without candidate code."""
    determinant = a11 * a22 - a12 * a21
    return (
        (b1 * a22 - a12 * b2) / determinant,
        (a11 * b2 - b1 * a21) / determinant,
    )


def _closure_reference(mechanism: dict, angle_deg: float) -> dict:
    """Compute closure geometry independently from the package."""
    a = mechanism["crank_m"]
    b = mechanism["coupler_m"]
    c = mechanism["rocker_m"]
    d = mechanism["ground_m"]
    branch = mechanism["branch"]
    theta2 = math.radians(angle_deg)
    ax = a * math.cos(theta2)
    ay = a * math.sin(theta2)
    dx = d - ax
    dy = -ay
    distance = math.hypot(dx, dy)
    ux = dx / distance
    uy = dy / distance
    along = (b * b - c * c + distance * distance) / (2.0 * distance)
    height = math.sqrt(max(0.0, b * b - along * along))
    bx = ax + along * ux + branch * height * (-uy)
    by = ay + along * uy + branch * height * ux
    theta3 = math.atan2(by - ay, bx - ax)
    theta4 = math.atan2(by, bx - d)
    return {
        "theta2": theta2,
        "theta3": theta3,
        "theta4": theta4,
        "B": (bx, by),
        "crank_angle_rad": theta2,
        "coupler_angle_rad": theta3,
        "rocker_angle_rad": theta4,
        "coupler_point": (bx, by),
        "rocker_point": (bx, by),
    }


def _velocity_reference(
    mechanism: dict,
    state: dict,
    crank_speed: float,
) -> dict[str, float]:
    """Compute link speeds from the differentiated closure equation."""
    a = mechanism["crank_m"]
    b = mechanism["coupler_m"]
    c = mechanism["rocker_m"]
    theta2 = state["theta2"]
    theta3 = state["theta3"]
    theta4 = state["theta4"]
    omega3, omega4 = _solve_2x2(
        -b * math.sin(theta3),
        c * math.sin(theta4),
        b * math.cos(theta3),
        -c * math.cos(theta4),
        a * math.sin(theta2) * crank_speed,
        -a * math.cos(theta2) * crank_speed,
    )
    return {
        "omega3": omega3,
        "omega4": omega4,
        "coupler_omega_rad_s": omega3,
        "rocker_omega_rad_s": omega4,
    }


def _acceleration_reference(
    mechanism: dict,
    state: dict,
    velocity: dict,
    crank_speed: float,
    crank_acceleration: float,
) -> dict[str, float]:
    """Compute link accelerations from the second closure derivative."""
    a = mechanism["crank_m"]
    b = mechanism["coupler_m"]
    c = mechanism["rocker_m"]
    theta2 = state["theta2"]
    theta3 = state["theta3"]
    theta4 = state["theta4"]
    omega2 = crank_speed
    omega3 = velocity["omega3"]
    omega4 = velocity["omega4"]
    alpha2 = crank_acceleration
    s2, c2 = math.sin(theta2), math.cos(theta2)
    s3, c3 = math.sin(theta3), math.cos(theta3)
    s4, c4 = math.sin(theta4), math.cos(theta4)
    alpha3, alpha4 = _solve_2x2(
        -b * s3,
        c * s4,
        b * c3,
        -c * c4,
        a * s2 * alpha2
        + a * c2 * omega2 * omega2
        + b * c3 * omega3 * omega3
        - c * c4 * omega4 * omega4,
        -a * c2 * alpha2
        + a * s2 * omega2 * omega2
        + b * s3 * omega3 * omega3
        - c * s4 * omega4 * omega4,
    )
    return {
        "alpha3": alpha3,
        "alpha4": alpha4,
        "coupler_alpha_rad_s2": alpha3,
        "rocker_alpha_rad_s2": alpha4,
    }


def _load_reference(
    mechanism: dict,
    state: dict,
    velocity: dict,
    angular_acceleration: dict,
    load: dict,
    crank_speed: float,
    crank_acceleration: float,
    mass_properties: dict,
    return_spring: dict,
    joint_friction: dict,
) -> dict[str, float | tuple[float, float]]:
    """Resolve load, gravity, spring, and friction by virtual work."""
    d = mechanism["ground_m"]
    bx, by = state["B"]
    ratio = load["application_ratio"]
    rx = ratio * (bx - d)
    ry = ratio * by
    direction = math.radians(load["direction_deg"])
    fx = load["force_n"] * math.cos(direction)
    fy = load["force_n"] * math.sin(direction)
    load_torque = rx * fy - ry * fx
    load_power = load_torque * velocity["omega4"]
    load_generalized = load_power / crank_speed

    dax = -mechanism["crank_m"] * math.sin(state["theta2"])
    day = mechanism["crank_m"] * math.cos(state["theta2"])
    dbx = (
        mechanism["rocker_m"]
        * velocity["omega4"]
        / crank_speed
        * (-math.sin(state["theta4"]))
    )
    dby = (
        mechanism["rocker_m"]
        * velocity["omega4"]
        / crank_speed
        * math.cos(state["theta4"])
    )
    crank_ratio = mass_properties["crank"]["com_ratio"]
    coupler_ratio = mass_properties["coupler"]["com_ratio"]
    rocker_ratio = mass_properties["rocker"]["com_ratio"]
    gravity = mass_properties["gravity_m_s2"]
    c2y = crank_ratio * day
    c3y = (1.0 - coupler_ratio) * day + coupler_ratio * dby
    c4y = rocker_ratio * dby
    gravity_generalized = -(
        mass_properties["crank"]["mass_kg"] * gravity * c2y
        + mass_properties["coupler"]["mass_kg"] * gravity * c3y
        + mass_properties["rocker"]["mass_kg"] * gravity * c4y
    )
    gravity_power = gravity_generalized * crank_speed

    zero_angle = math.radians(return_spring["zero_angle_deg"])
    spring_torque = -return_spring["stiffness_n_m_per_rad"] * (
        state["theta4"] - zero_angle
    )
    spring_generalized = spring_torque * velocity["omega4"] / crank_speed
    spring_power = spring_generalized * crank_speed

    friction_loss = (
        abs(crank_speed) * joint_friction["o2_n_m"]
        + abs(velocity["omega3"] - velocity["omega4"]) * joint_friction["b_n_m"]
        + abs(velocity["omega4"]) * joint_friction["o4_n_m"]
    )
    alpha2 = crank_acceleration
    alpha3 = angular_acceleration["alpha3"]
    alpha4 = angular_acceleration["alpha4"]
    ax_acc = mechanism["crank_m"] * (
        -alpha2 * math.sin(state["theta2"])
        - crank_speed * crank_speed * math.cos(state["theta2"])
    )
    ay_acc = mechanism["crank_m"] * (
        alpha2 * math.cos(state["theta2"])
        - crank_speed * crank_speed * math.sin(state["theta2"])
    )
    bx_acc = mechanism["rocker_m"] * (
        -alpha4 * math.sin(state["theta4"])
        - velocity["omega4"] * velocity["omega4"] * math.cos(state["theta4"])
    )
    by_acc = mechanism["rocker_m"] * (
        alpha4 * math.cos(state["theta4"])
        - velocity["omega4"] * velocity["omega4"] * math.sin(state["theta4"])
    )
    c2ax = crank_ratio * ax_acc
    c2ay = crank_ratio * ay_acc
    c3ax = (1.0 - coupler_ratio) * ax_acc + coupler_ratio * bx_acc
    c3ay = (1.0 - coupler_ratio) * ay_acc + coupler_ratio * by_acc
    c4ax = rocker_ratio * bx_acc
    c4ay = rocker_ratio * by_acc
    dax = -mechanism["crank_m"] * math.sin(state["theta2"])
    day = mechanism["crank_m"] * math.cos(state["theta2"])
    dbx = (
        mechanism["rocker_m"]
        * velocity["omega4"]
        / crank_speed
        * (-math.sin(state["theta4"]))
    )
    dby = (
        mechanism["rocker_m"]
        * velocity["omega4"]
        / crank_speed
        * math.cos(state["theta4"])
    )
    c2x, c2y = crank_ratio * dax, crank_ratio * day
    c3x = (1.0 - coupler_ratio) * dax + coupler_ratio * dbx
    c3y = (1.0 - coupler_ratio) * day + coupler_ratio * dby
    c4x, c4y = rocker_ratio * dbx, rocker_ratio * dby
    inertial_generalized = (
        mass_properties["crank"]["mass_kg"] * (c2ax * c2x + c2ay * c2y)
        + mass_properties["crank"]["inertia_kg_m2"] * alpha2
        + mass_properties["coupler"]["mass_kg"] * (c3ax * c3x + c3ay * c3y)
        + mass_properties["coupler"]["inertia_kg_m2"]
        * alpha3
        * velocity["omega3"]
        / crank_speed
        + mass_properties["rocker"]["mass_kg"] * (c4ax * c4x + c4ay * c4y)
        + mass_properties["rocker"]["inertia_kg_m2"]
        * alpha4
        * velocity["omega4"]
        / crank_speed
    )
    kinetic_power = inertial_generalized * crank_speed
    drive_torque = -(
        load_generalized + gravity_generalized + spring_generalized
    ) + friction_loss / crank_speed + inertial_generalized
    return {
        "point": (d + rx, ry),
        "load_torque": load_torque,
        "load_generalized": load_generalized,
        "gravity_generalized": gravity_generalized,
        "spring_generalized": spring_generalized,
        "inertial_generalized": inertial_generalized,
        "friction_loss": friction_loss,
        "drive_torque": drive_torque,
        "load_power": load_power,
        "gravity_power": gravity_power,
        "spring_power": spring_power,
        "kinetic_power": kinetic_power,
        "drive_power": drive_torque * crank_speed,
    }


def _transmission_reference(state: dict) -> float:
    """Return the acute angle between the two moving-link lines."""
    difference = state["theta4"] - state["theta3"]
    wrapped = abs(math.atan2(math.sin(difference), math.cos(difference)))
    return math.degrees(min(wrapped, math.pi - wrapped))


def _reference_sample(
    mechanism: dict,
    angle_deg: float,
    speed: float,
    acceleration: float,
    load: dict,
    mass_properties: dict,
    return_spring: dict,
    joint_friction: dict,
) -> dict:
    """Build one complete expected sample."""
    position = _closure_reference(mechanism, angle_deg)
    velocity = _velocity_reference(mechanism, position, speed)
    angular = _acceleration_reference(
        mechanism, position, velocity, speed, acceleration
    )
    force = _load_reference(
        mechanism,
        position,
        velocity,
        angular,
        load,
        speed,
        acceleration,
        mass_properties,
        return_spring,
        joint_friction,
    )
    return {
        "position": position,
        "velocity": velocity,
        "acceleration": angular,
        "load": force,
        "transmission": _transmission_reference(position),
    }


def _run_cli(case: dict, existing: str | None = None) -> tuple[int, str | None, str]:
    """Run the package command in a temporary directory."""
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        source = root / "case.json"
        target = root / "report.json"
        source.write_text(json.dumps(case), encoding="utf-8")
        if existing is not None:
            target.write_text(existing, encoding="utf-8")
        completed = subprocess.run(
            [sys.executable, "-m", "toggle_linkage.cli", str(source), str(target)],
            cwd="/workspace",
            capture_output=True,
            text=True,
            timeout=20,
        )
        payload = target.read_text(encoding="utf-8") if target.exists() else None
        return completed.returncode, payload, completed.stderr


def _run_cli_with_blocked_parent(case: dict) -> tuple[int, str, str]:
    """Exercise write failure when the report parent is an unwritable file."""
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        source = root / "case.json"
        blocked_parent = root / "blocked-parent"
        target = blocked_parent / "report.json"
        source.write_text(json.dumps(case), encoding="utf-8")
        blocked_parent.write_text("parent sentinel", encoding="utf-8")
        completed = subprocess.run(
            [sys.executable, "-m", "toggle_linkage.cli", str(source), str(target)],
            cwd="/workspace",
            capture_output=True,
            text=True,
            timeout=20,
        )
        return (
            completed.returncode,
            blocked_parent.read_text(encoding="utf-8"),
            completed.stderr,
        )


def _base_case() -> dict:
    """Return a valid three-sample case."""
    return {
        "mechanism": _mechanism(),
        "mass_properties": copy.deepcopy(BASE_MASS_PROPERTIES),
        "return_spring": dict(BASE_RETURN_SPRING),
        "joint_friction": dict(BASE_JOINT_FRICTION),
        "input": {
            "crank_angles_deg": [0.0, 90.0, 210.0],
            "crank_speed_rad_s": 12.0,
            "crank_acceleration_rad_s2": 0.8,
        },
        "load": dict(BASE_LOAD),
    }


def _invalid_cases() -> dict[str, dict]:
    """Build representative failures for every input contract family."""
    cases = {}

    missing_root = _base_case()
    missing_root.pop("load")
    cases["missing_root"] = missing_root

    branch = _base_case()
    branch["mechanism"]["branch"] = 0
    cases["branch"] = branch

    negative_length = _base_case()
    negative_length["mechanism"]["rocker_m"] = -0.48
    cases["negative_length"] = negative_length

    nonintersecting = _base_case()
    nonintersecting["mechanism"]["coupler_m"] = 0.01
    cases["nonintersecting"] = nonintersecting

    empty_angles = _base_case()
    empty_angles["input"]["crank_angles_deg"] = []
    cases["empty_angles"] = empty_angles

    angle_out_of_range = _base_case()
    angle_out_of_range["input"]["crank_angles_deg"] = [0.0, 360.0]
    cases["angle_out_of_range"] = angle_out_of_range

    nonfinite_angle = _base_case()
    nonfinite_angle["input"]["crank_angles_deg"] = [0.0, float("nan")]
    cases["nonfinite_angle"] = nonfinite_angle

    zero_speed = _base_case()
    zero_speed["input"]["crank_speed_rad_s"] = 0.0
    cases["zero_speed"] = zero_speed

    nonfinite_acceleration = _base_case()
    nonfinite_acceleration["input"]["crank_acceleration_rad_s2"] = float("inf")
    cases["nonfinite_acceleration"] = nonfinite_acceleration

    negative_force = _base_case()
    negative_force["load"]["force_n"] = -1.0
    cases["negative_force"] = negative_force

    nonfinite_direction = _base_case()
    nonfinite_direction["load"]["direction_deg"] = float("nan")
    cases["nonfinite_direction"] = nonfinite_direction

    lower_ratio = _base_case()
    lower_ratio["load"]["application_ratio"] = 0.0
    cases["lower_ratio"] = lower_ratio

    upper_ratio = _base_case()
    upper_ratio["load"]["application_ratio"] = 1.01
    cases["upper_ratio"] = upper_ratio

    zero_mass = _base_case()
    zero_mass["mass_properties"]["coupler"]["mass_kg"] = 0.0
    cases["zero_mass"] = zero_mass

    zero_inertia = _base_case()
    zero_inertia["mass_properties"]["crank"]["inertia_kg_m2"] = 0.0
    cases["zero_inertia"] = zero_inertia

    invalid_com = _base_case()
    invalid_com["mass_properties"]["rocker"]["com_ratio"] = 1.1
    cases["invalid_com"] = invalid_com

    nonpositive_gravity = _base_case()
    nonpositive_gravity["mass_properties"]["gravity_m_s2"] = 0.0
    cases["nonpositive_gravity"] = nonpositive_gravity

    negative_stiffness = _base_case()
    negative_stiffness["return_spring"]["stiffness_n_m_per_rad"] = -1.0
    cases["negative_stiffness"] = negative_stiffness

    nonfinite_zero_angle = _base_case()
    nonfinite_zero_angle["return_spring"]["zero_angle_deg"] = float("nan")
    cases["nonfinite_zero_angle"] = nonfinite_zero_angle

    negative_friction = _base_case()
    negative_friction["joint_friction"]["b_n_m"] = -0.2
    cases["negative_friction"] = negative_friction

    return cases


@case_contract(
    "CASE-C1-01",
    "R1",
    "T1",
    "`branch=1` 选择该线左侧的圆交点",
)
def test_c1_branch_positive_positions():
    """CASE-C1-01 / R1 / T1: “`branch=1` 选择该线左侧的圆交点”."""
    from toggle_linkage.geometry import solve_closure

    mechanism = _mechanism(1)
    matched = True
    for angle in (15.0, 100.0, 220.0, 310.0):
        actual = solve_closure(mechanism, math.radians(angle))
        expected = _closure_reference(mechanism, angle)
        matched = matched and math.isclose(
            actual["coupler_point"][0], expected["B"][0], rel_tol=1e-12, abs_tol=1e-12
        )
        matched = matched and math.isclose(
            actual["coupler_point"][1], expected["B"][1], rel_tol=1e-12, abs_tol=1e-12
        )
        matched = matched and math.isclose(
            actual["coupler_angle_rad"],
            expected["theta3"],
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
        matched = matched and math.isclose(
            actual["rocker_angle_rad"],
            expected["theta4"],
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
    direct = solve_closure(mechanism, math.radians(15.0))
    direct_expected = _closure_reference(mechanism, 15.0)
    assert math.isclose(
        direct["coupler_point"][0],
        direct_expected["B"][0],
        rel_tol=1e-12,
        abs_tol=1e-12,
    )
    assert math.isclose(
        direct["coupler_point"][1],
        direct_expected["B"][1],
        rel_tol=1e-12,
        abs_tol=1e-12,
    )
    assert matched


@case_contract(
    "CASE-C1-02",
    "R1",
    "T2",
    "`crank*exp(i*crank_angle) + coupler*exp(i*coupler_angle) - rocker*exp(i*rocker_angle) - ground = 0`",
)
def test_c1_closure_at_cardinal_angles():
    """CASE-C1-02 / R1 / T2: “`crank*exp(i*crank_angle) + coupler*exp(i*coupler_angle) - rocker*exp(i*rocker_angle) - ground = 0`”."""
    from toggle_linkage.geometry import solve_closure

    mechanism = _mechanism(1)
    residuals = []
    for angle in (0.0, 90.0, 180.0, 270.0):
        state = solve_closure(mechanism, math.radians(angle))
        bx, by = state["coupler_point"]
        rx = (
            mechanism["crank_m"] * math.cos(state["crank_angle_rad"])
            + mechanism["coupler_m"] * math.cos(state["coupler_angle_rad"])
            - mechanism["rocker_m"] * math.cos(state["rocker_angle_rad"])
            - mechanism["ground_m"]
        )
        ry = (
            mechanism["crank_m"] * math.sin(state["crank_angle_rad"])
            + mechanism["coupler_m"] * math.sin(state["coupler_angle_rad"])
            - mechanism["rocker_m"] * math.sin(state["rocker_angle_rad"])
        )
        residuals.append(math.hypot(rx, ry))
        residuals.append(
            math.hypot(
                bx
                - (
                    mechanism["ground_m"]
                    + mechanism["rocker_m"]
                    * math.cos(state["rocker_angle_rad"])
                ),
                by
                - mechanism["rocker_m"] * math.sin(state["rocker_angle_rad"]),
            )
        )
    assert max(residuals) < 1e-12


@case_contract(
    "CASE-C1-03",
    "R1",
    "T3",
    "`branch=-1` 选择右侧交点",
)
def test_c1_branch_negative_is_mirror_solution():
    """CASE-C1-03 / R1 / T3: “`branch=-1` 选择右侧交点”."""
    from toggle_linkage.geometry import solve_closure

    positive = _mechanism(1)
    negative = _mechanism(-1)
    matched = True
    separated = []
    for angle in (40.0, 135.0, 250.0):
        got = solve_closure(negative, math.radians(angle))
        expected = _closure_reference(negative, angle)
        matched = matched and math.isclose(
            got["coupler_point"][0], expected["B"][0], rel_tol=1e-12, abs_tol=1e-12
        )
        matched = matched and math.isclose(
            got["coupler_point"][1], expected["B"][1], rel_tol=1e-12, abs_tol=1e-12
        )
        upper = solve_closure(positive, math.radians(angle))
        separated.append(math.hypot(
            got["coupler_point"][0] - upper["coupler_point"][0],
            got["coupler_point"][1] - upper["coupler_point"][1],
        ))
    assert matched
    assert min(separated) > 0.1


@case_contract(
    "CASE-C1-04",
    "R1",
    "T4",
    "当曲柄是最短杆、与机架相邻且满足 Grashof 条件 `最短杆+最长杆 <= 另外两杆之和` 时，`grashof_class` 写 `crank-rocker`",
)
def test_c1_grashof_classification():
    """CASE-C1-04 / R1 / T4: “当曲柄是最短杆、与机架相邻且满足 Grashof 条件 `最短杆+最长杆 <= 另外两杆之和` 时，`grashof_class` 写 `crank-rocker`”."""
    from toggle_linkage.geometry import classify_grashof, shortest_link

    mechanism = _mechanism()
    assert classify_grashof(mechanism) == "crank-rocker"
    assert shortest_link(mechanism) == "crank"


@case_contract(
    "CASE-C2-01",
    "R2",
    "T5",
    "速度与加速度应分别由该闭环方程的一阶、二阶时间导数求出",
)
def test_c2_velocity_matches_reference():
    """CASE-C2-01 / R2 / T5: “速度与加速度应分别由该闭环方程的一阶、二阶时间导数求出”."""
    from toggle_linkage.kinematics import solve_velocity

    mechanism = _mechanism()
    speed = 9.5
    matched = True
    for angle in (25.0, 145.0, 285.0):
        state = _closure_reference(mechanism, angle)
        expected = _velocity_reference(mechanism, state, speed)
        actual = solve_velocity(mechanism, state, speed)
        matched = matched and math.isclose(
            actual["coupler_omega_rad_s"],
            expected["omega3"],
            rel_tol=1e-11,
            abs_tol=1e-11,
        )
        matched = matched and math.isclose(
            actual["rocker_omega_rad_s"],
            expected["omega4"],
            rel_tol=1e-11,
            abs_tol=1e-11,
        )
    direct_state = _closure_reference(mechanism, 25.0)
    direct_actual = solve_velocity(mechanism, direct_state, speed)
    direct_expected = _velocity_reference(mechanism, direct_state, speed)
    assert math.isclose(
        direct_actual["coupler_omega_rad_s"],
        direct_expected["omega3"],
        rel_tol=1e-11,
        abs_tol=1e-11,
    )
    assert math.isclose(
        direct_actual["rocker_omega_rad_s"],
        direct_expected["omega4"],
        rel_tol=1e-11,
        abs_tol=1e-11,
    )
    assert matched


@case_contract(
    "CASE-C2-02",
    "R2",
    "T6",
    "速度与加速度应分别由该闭环方程的一阶、二阶时间导数求出",
)
def test_c2_velocity_closure_residual():
    """CASE-C2-02 / R2 / T6: “速度与加速度应分别由该闭环方程的一阶、二阶时间导数求出”."""
    from toggle_linkage.kinematics import solve_velocity

    mechanism = _mechanism()
    angle = 72.0
    state = _closure_reference(mechanism, angle)
    actual = solve_velocity(mechanism, state, 11.0)
    rx = (
        -mechanism["crank_m"] * math.sin(state["theta2"]) * 11.0
        - mechanism["coupler_m"]
        * math.sin(state["theta3"])
        * actual["coupler_omega_rad_s"]
        + mechanism["rocker_m"]
        * math.sin(state["theta4"])
        * actual["rocker_omega_rad_s"]
    )
    ry = (
        mechanism["crank_m"] * math.cos(state["theta2"]) * 11.0
        + mechanism["coupler_m"]
        * math.cos(state["theta3"])
        * actual["coupler_omega_rad_s"]
        - mechanism["rocker_m"]
        * math.cos(state["theta4"])
        * actual["rocker_omega_rad_s"]
    )
    assert math.hypot(rx, ry) < 1e-10


@case_contract(
    "CASE-C2-03",
    "R2",
    "T7",
    "`input.crank_speed_rad_s` 是有限非零曲柄角速度",
)
def test_c2_velocity_scales_with_crank_speed():
    """CASE-C2-03 / R2 / T7: “`input.crank_speed_rad_s` 是有限非零曲柄角速度”."""
    from toggle_linkage.kinematics import solve_velocity

    mechanism = _mechanism()
    state = _closure_reference(mechanism, 190.0)
    slow = solve_velocity(mechanism, state, 4.0)
    fast = solve_velocity(mechanism, state, 10.0)
    assert math.isclose(
        fast["coupler_omega_rad_s"],
        slow["coupler_omega_rad_s"] * 2.5,
        rel_tol=1e-12,
        abs_tol=1e-12,
    )
    assert math.isclose(
        fast["rocker_omega_rad_s"],
        slow["rocker_omega_rad_s"] * 2.5,
        rel_tol=1e-12,
        abs_tol=1e-12,
    )


@case_contract(
    "CASE-C3-01",
    "R3",
    "T8",
    "速度与加速度应分别由该闭环方程的一阶、二阶时间导数求出",
)
def test_c3_acceleration_matches_reference():
    """CASE-C3-01 / R3 / T8: “速度与加速度应分别由该闭环方程的一阶、二阶时间导数求出”."""
    from toggle_linkage.kinematics import solve_acceleration

    mechanism = _mechanism()
    speed = 12.0
    crank_acceleration = 1.4
    matched = True
    for angle in (18.0, 112.0, 268.0):
        state = _closure_reference(mechanism, angle)
        velocity = _velocity_reference(mechanism, state, speed)
        expected = _acceleration_reference(
            mechanism, state, velocity, speed, crank_acceleration
        )
        actual = solve_acceleration(
            mechanism, state, velocity, speed, crank_acceleration
        )
        matched = matched and math.isclose(
            actual["coupler_alpha_rad_s2"],
            expected["alpha3"],
            rel_tol=1e-10,
            abs_tol=1e-10,
        )
        matched = matched and math.isclose(
            actual["rocker_alpha_rad_s2"],
            expected["alpha4"],
            rel_tol=1e-10,
            abs_tol=1e-10,
        )
    direct_state = _closure_reference(mechanism, 18.0)
    direct_velocity = _velocity_reference(mechanism, direct_state, speed)
    direct_actual = solve_acceleration(
        mechanism,
        direct_state,
        direct_velocity,
        speed,
        crank_acceleration,
    )
    direct_expected = _acceleration_reference(
        mechanism,
        direct_state,
        direct_velocity,
        speed,
        crank_acceleration,
    )
    assert math.isclose(
        direct_actual["coupler_alpha_rad_s2"],
        direct_expected["alpha3"],
        rel_tol=1e-10,
        abs_tol=1e-10,
    )
    assert math.isclose(
        direct_actual["rocker_alpha_rad_s2"],
        direct_expected["alpha4"],
        rel_tol=1e-10,
        abs_tol=1e-10,
    )
    assert matched


@case_contract(
    "CASE-C3-02",
    "R3",
    "T9",
    "速度与加速度应分别由该闭环方程的一阶、二阶时间导数求出",
)
def test_c3_acceleration_closure_residual():
    """CASE-C3-02 / R3 / T9: “速度与加速度应分别由该闭环方程的一阶、二阶时间导数求出”."""
    from toggle_linkage.kinematics import solve_acceleration

    mechanism = _mechanism()
    speed = 10.5
    alpha2 = -0.7
    state = _closure_reference(mechanism, 205.0)
    velocity = _velocity_reference(mechanism, state, speed)
    actual = solve_acceleration(mechanism, state, velocity, speed, alpha2)
    a = mechanism["crank_m"]
    b = mechanism["coupler_m"]
    c = mechanism["rocker_m"]
    s2, c2 = math.sin(state["theta2"]), math.cos(state["theta2"])
    s3, c3 = math.sin(state["theta3"]), math.cos(state["theta3"])
    s4, c4 = math.sin(state["theta4"]), math.cos(state["theta4"])
    w2, w3, w4 = speed, velocity["omega3"], velocity["omega4"]
    alpha3 = actual["coupler_alpha_rad_s2"]
    alpha4 = actual["rocker_alpha_rad_s2"]
    rx = (
        -a * s2 * alpha2
        - b * s3 * alpha3
        + c * s4 * alpha4
        - a * c2 * w2 * w2
        - b * c3 * w3 * w3
        + c * c4 * w4 * w4
    )
    ry = (
        a * c2 * alpha2
        + b * c3 * alpha3
        - c * c4 * alpha4
        - a * s2 * w2 * w2
        - b * s3 * w3 * w3
        + c * s4 * w4 * w4
    )
    assert math.hypot(rx, ry) < 1e-9


@case_contract(
    "CASE-C3-03",
    "R3",
    "T10",
    "`a_Ci` 必须在世界坐标系中计算，不能只取切向项",
)
def test_c3_centripetal_terms_are_present():
    """CASE-C3-03 / R3 / T10: “`a_Ci` 必须在世界坐标系中计算，不能只取切向项”."""
    from toggle_linkage.kinematics import solve_acceleration

    mechanism = _mechanism()
    speed = 13.0
    state = _closure_reference(mechanism, 150.0)
    velocity = _velocity_reference(mechanism, state, speed)
    expected = _acceleration_reference(mechanism, state, velocity, speed, 0.0)
    actual = solve_acceleration(mechanism, state, velocity, speed, 0.0)
    assert abs(expected["alpha4"]) > 1.0
    assert math.isclose(
        actual["rocker_alpha_rad_s2"],
        expected["alpha4"],
        rel_tol=1e-11,
        abs_tol=1e-11,
    )


@case_contract(
    "CASE-C4-01",
    "R4",
    "T11",
    "`P = O4 + application_ratio * (B - O4)`",
)
def test_c4_load_point_and_torque():
    """CASE-C4-01 / R4 / T11: “`P = O4 + application_ratio * (B - O4)`”."""
    from toggle_linkage.loads import solve_load

    mechanism = _mechanism()
    state = _closure_reference(mechanism, 125.0)
    velocity = _velocity_reference(mechanism, state, 8.0)
    angular = _acceleration_reference(mechanism, state, velocity, 8.0, 0.8)
    matched = True
    for ratio in (0.35, 0.72, 1.0):
        load = dict(BASE_LOAD)
        load["application_ratio"] = ratio
        expected = _load_reference(
            mechanism,
            state,
            velocity,
            angular,
            load,
            8.0,
            0.8,
            BASE_MASS_PROPERTIES,
            BASE_RETURN_SPRING,
            BASE_JOINT_FRICTION,
        )
        actual = solve_load(
            mechanism,
            state,
            velocity,
            angular,
            load,
            8.0,
            0.8,
            BASE_MASS_PROPERTIES,
            BASE_RETURN_SPRING,
            BASE_JOINT_FRICTION,
        )
        matched = matched and math.isclose(
            actual["load_point"][0],
            expected["point"][0],
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
        matched = matched and math.isclose(
            actual["load_point"][1],
            expected["point"][1],
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
        matched = matched and math.isclose(
            actual["load_torque_n_m"],
            expected["load_torque"],
            rel_tol=1e-11,
            abs_tol=1e-11,
        )
        matched = matched and math.isclose(
            actual["gravity_generalized_torque_n_m"],
            expected["gravity_generalized"],
            rel_tol=1e-11,
            abs_tol=1e-11,
        )
        matched = matched and math.isclose(
            actual["spring_generalized_torque_n_m"],
            expected["spring_generalized"],
            rel_tol=1e-11,
            abs_tol=1e-11,
        )
        matched = matched and math.isclose(
            actual["inertial_generalized_torque_n_m"],
            expected["inertial_generalized"],
            rel_tol=1e-11,
            abs_tol=1e-11,
        )
    direct_load = dict(BASE_LOAD)
    direct_load["application_ratio"] = 0.35
    direct_actual = solve_load(
        mechanism,
        state,
        velocity,
        angular,
        direct_load,
        8.0,
        0.8,
        BASE_MASS_PROPERTIES,
        BASE_RETURN_SPRING,
        BASE_JOINT_FRICTION,
    )
    direct_expected = _load_reference(
        mechanism,
        state,
        velocity,
        angular,
        direct_load,
        8.0,
        0.8,
        BASE_MASS_PROPERTIES,
        BASE_RETURN_SPRING,
        BASE_JOINT_FRICTION,
    )
    assert math.isclose(
        direct_actual["load_point"][0],
        direct_expected["point"][0],
        rel_tol=1e-12,
        abs_tol=1e-12,
    )
    assert math.isclose(
        direct_actual["load_point"][1],
        direct_expected["point"][1],
        rel_tol=1e-12,
        abs_tol=1e-12,
    )
    assert matched


@case_contract(
    "CASE-C4-02",
    "R4",
    "T12",
    "传动角定义为连杆正向与摇杆正向所在直线的锐角，取值范围为 `0` 到 `90` 度",
)
def test_c4_transmission_angle_range():
    """CASE-C4-02 / R4 / T12: “传动角定义为连杆正向与摇杆正向所在直线的锐角，取值范围为 `0` 到 `90` 度”."""
    from toggle_linkage.kinematics import transmission_angle_deg

    mechanism = _mechanism()
    angles = []
    matched = True
    for angle in (0.0, 90.0, 180.0, 270.0):
        state = _closure_reference(mechanism, angle)
        actual = transmission_angle_deg(state)
        angles.append(actual)
        matched = matched and math.isclose(
            actual,
            _transmission_reference(state),
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
    assert matched
    assert min(angles) >= 0.0
    assert max(angles) <= 90.0


@case_contract(
    "CASE-C4-03",
    "R4",
    "T13",
    "drive_torque*crank_speed + Q_load*crank_speed + Q_gravity*crank_speed + Q_spring*crank_speed - friction_loss - Q_inertia*crank_speed = 0",
)
def test_c4_power_balance():
    """CASE-C4-03 / R4 / T13: “`drive_torque*crank_speed + Q_load*crank_speed + Q_gravity*crank_speed + Q_spring*crank_speed - friction_loss - Q_inertia*crank_speed = 0`”."""
    from toggle_linkage.loads import solve_load

    mechanism = _mechanism()
    speed = 12.0
    matched = True
    for angle in (30.0, 140.0, 250.0):
        state = _closure_reference(mechanism, angle)
        velocity = _velocity_reference(mechanism, state, speed)
        angular = _acceleration_reference(mechanism, state, velocity, speed, 1.2)
        actual = solve_load(
            mechanism,
            state,
            velocity,
            angular,
            BASE_LOAD,
            speed,
            1.2,
            BASE_MASS_PROPERTIES,
            BASE_RETURN_SPRING,
            BASE_JOINT_FRICTION,
        )
        expected = _load_reference(
            mechanism,
            state,
            velocity,
            angular,
            BASE_LOAD,
            speed,
            1.2,
            BASE_MASS_PROPERTIES,
            BASE_RETURN_SPRING,
            BASE_JOINT_FRICTION,
        )
        matched = matched and math.isclose(
            actual["load_torque_n_m"],
            expected["load_torque"],
            rel_tol=1e-11,
            abs_tol=1e-11,
        )
        matched = matched and math.isclose(
            actual["drive_torque_n_m"],
            expected["drive_torque"],
            rel_tol=1e-11,
            abs_tol=1e-11,
        )
        matched = matched and math.isclose(
            actual["friction_loss_w"],
            expected["friction_loss"],
            rel_tol=1e-11,
            abs_tol=1e-11,
        )
        matched = matched and math.isclose(
            actual["inertial_generalized_torque_n_m"],
            expected["inertial_generalized"],
            rel_tol=1e-11,
            abs_tol=1e-11,
        )
        matched = matched and math.isclose(
            actual["drive_power_w"]
            + actual["load_power_w"]
            + actual["gravity_power_w"]
            + actual["spring_power_w"]
            - actual["friction_loss_w"],
            actual["kinetic_power_w"],
            rel_tol=1e-12,
            abs_tol=1e-9,
        )
        matched = matched and math.isclose(
            actual["kinetic_power_w"],
            expected["kinetic_power"],
            rel_tol=1e-11,
            abs_tol=1e-11,
        )
        matched = matched and math.isclose(
            actual["drive_power_w"]
            + actual["load_power_w"]
            + actual["gravity_power_w"]
            + actual["spring_power_w"]
            - actual["friction_loss_w"]
            - actual["kinetic_power_w"],
            0.0,
            rel_tol=1e-12,
            abs_tol=1e-9,
        )
    direct_state = _closure_reference(mechanism, 30.0)
    direct_velocity = _velocity_reference(mechanism, direct_state, speed)
    direct_angular = _acceleration_reference(
        mechanism, direct_state, direct_velocity, speed, 1.2
    )
    direct = solve_load(
        mechanism,
        direct_state,
        direct_velocity,
        direct_angular,
        BASE_LOAD,
        speed,
        1.2,
        BASE_MASS_PROPERTIES,
        BASE_RETURN_SPRING,
        BASE_JOINT_FRICTION,
    )
    assert math.isclose(
        direct["drive_power_w"]
        + direct["load_power_w"]
        + direct["gravity_power_w"]
        + direct["spring_power_w"]
        - direct["friction_loss_w"]
        - direct["kinetic_power_w"],
        0.0,
        rel_tol=1e-12,
        abs_tol=1e-9,
    )
    assert matched


@case_contract(
    "CASE-C5-01",
    "R5",
    "T14",
    "输出根对象固定包含 `mechanism`、`samples` 和 `summary`",
)
def test_c5_cli_report_schema():
    """CASE-C5-01 / R5 / T14: “输出根对象固定包含 `mechanism`、`samples` 和 `summary`”."""
    status, payload, _stderr = _run_cli(_base_case())
    assert status == 0
    assert payload is not None
    report = json.loads(payload)
    assert set(report) == {"mechanism", "samples", "summary"}
    assert len(report["samples"]) == 3
    assert set(report["summary"]) == {
        "minimum_transmission_angle_deg",
        "maximum_abs_coupler_speed_rad_s",
        "maximum_abs_rocker_speed_rad_s",
        "maximum_abs_drive_torque_n_m",
        "maximum_friction_loss_w",
        "maximum_abs_kinetic_power_w",
    }
    required_sample_fields = {
        "crank_angle_deg",
        "crank_angle_rad",
        "coupler_angle_deg",
        "rocker_angle_deg",
        "coupler_point_m",
        "load_point_m",
        "coupler_omega_rad_s",
        "rocker_omega_rad_s",
        "coupler_alpha_rad_s2",
        "rocker_alpha_rad_s2",
        "transmission_angle_deg",
        "load_torque_n_m",
        "gravity_generalized_torque_n_m",
        "spring_generalized_torque_n_m",
        "inertial_generalized_torque_n_m",
        "friction_loss_w",
        "drive_torque_n_m",
        "load_power_w",
        "gravity_power_w",
        "spring_power_w",
        "kinetic_power_w",
        "drive_power_w",
    }
    first, second, third = report["samples"]
    assert set(first) == required_sample_fields
    assert set(second) == required_sample_fields
    assert set(third) == required_sample_fields


@case_contract(
    "CASE-C5-02",
    "R5",
    "T17",
    "如果报告路径的父目录尚不存在，成功路径也应创建它",
)
def test_c5_creates_missing_parent_directory():
    """CASE-C5-02 / R5 / T17: “如果报告路径的父目录尚不存在，成功路径也应创建它”."""
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        source = root / "case.json"
        target = root / "pending" / "reports" / "report.json"
        source.write_text(json.dumps(_base_case()), encoding="utf-8")
        completed = subprocess.run(
            [sys.executable, "-m", "toggle_linkage.cli", str(source), str(target)],
            cwd="/workspace",
            capture_output=True,
            text=True,
            timeout=20,
        )
        assert completed.returncode == 0
        assert target.is_file()
        assert len(json.loads(target.read_text(encoding="utf-8"))["samples"]) == 3


@case_contract(
    "CASE-C5-03",
    "R5",
    "T15",
    "命令应以非零状态退出，并且不能生成新的报告，也不能覆盖已有报告",
)
def test_c5_invalid_inputs_do_not_replace():
    """CASE-C5-03 / R5 / T15: “命令应以非零状态退出，并且不能生成新的报告，也不能覆盖已有报告”."""
    failures = {}
    for name, case in _invalid_cases().items():
        status, payload, _stderr = _run_cli(case, existing="previous report")
        failures[name] = status == 0 or payload != "previous report"
    assert not any(failures.values()), failures
    status, blocked_parent, _stderr = _run_cli_with_blocked_parent(_base_case())
    assert status != 0
    assert blocked_parent == "parent sentinel"


@case_contract(
    "CASE-E2E-01",
    "R1,R2,R3,R4,R5",
    "T16",
    "`samples` 保持输入角顺序",
)
def test_e2e_full_cycle_matches_reference():
    """CASE-E2E-01 / T16: “`samples` 保持输入角顺序”."""
    case = json.loads(Path("/workspace/data/example.json").read_text(encoding="utf-8"))
    status, payload, _stderr = _run_cli(case)
    assert status == 0
    assert payload is not None
    report = json.loads(payload)
    mechanism = case["mechanism"]
    settings = case["input"]
    load = case["load"]
    mass_properties = case["mass_properties"]
    return_spring = case["return_spring"]
    joint_friction = case["joint_friction"]
    matched = report["mechanism"]["grashof_class"] == "crank-rocker"
    matched = matched and report["mechanism"]["shortest_link"] == "crank"
    for sample, angle in zip(
        report["samples"], settings["crank_angles_deg"], strict=True
    ):
        expected = _reference_sample(
            mechanism,
            angle,
            settings["crank_speed_rad_s"],
            settings["crank_acceleration_rad_s2"],
            load,
            mass_properties,
            return_spring,
            joint_friction,
        )
        matched = matched and math.isclose(
            sample["crank_angle_deg"], angle, rel_tol=1e-12, abs_tol=1e-12
        )
        matched = matched and math.isclose(
            sample["coupler_point_m"][0],
            expected["position"]["B"][0],
            rel_tol=1e-11,
            abs_tol=1e-11,
        )
        matched = matched and math.isclose(
            sample["coupler_point_m"][1],
            expected["position"]["B"][1],
            rel_tol=1e-11,
            abs_tol=1e-11,
        )
        matched = matched and math.isclose(
            sample["rocker_omega_rad_s"],
            expected["velocity"]["omega4"],
            rel_tol=1e-10,
            abs_tol=1e-10,
        )
        matched = matched and math.isclose(
            sample["rocker_alpha_rad_s2"],
            expected["acceleration"]["alpha4"],
            rel_tol=1e-9,
            abs_tol=1e-9,
        )
        matched = matched and math.isclose(
            sample["drive_torque_n_m"],
            expected["load"]["drive_torque"],
            rel_tol=1e-10,
            abs_tol=1e-10,
        )
        matched = matched and math.isclose(
            sample["gravity_generalized_torque_n_m"],
            expected["load"]["gravity_generalized"],
            rel_tol=1e-10,
            abs_tol=1e-10,
        )
        matched = matched and math.isclose(
            sample["spring_generalized_torque_n_m"],
            expected["load"]["spring_generalized"],
            rel_tol=1e-10,
            abs_tol=1e-10,
        )
        matched = matched and math.isclose(
            sample["friction_loss_w"],
            expected["load"]["friction_loss"],
            rel_tol=1e-10,
            abs_tol=1e-10,
        )
        matched = matched and math.isclose(
            sample["inertial_generalized_torque_n_m"],
            expected["load"]["inertial_generalized"],
            rel_tol=1e-10,
            abs_tol=1e-10,
        )
        matched = matched and math.isclose(
            sample["kinetic_power_w"],
            expected["load"]["kinetic_power"],
            rel_tol=1e-10,
            abs_tol=1e-10,
        )
        matched = matched and math.isclose(
            sample["transmission_angle_deg"],
            expected["transmission"],
            rel_tol=1e-11,
            abs_tol=1e-11,
        )
    first_sample = report["samples"][0]
    first_expected = _reference_sample(
        mechanism,
        settings["crank_angles_deg"][0],
        settings["crank_speed_rad_s"],
        settings["crank_acceleration_rad_s2"],
        load,
        mass_properties,
        return_spring,
        joint_friction,
    )
    assert math.isclose(
        first_sample["coupler_point_m"][0],
        first_expected["position"]["B"][0],
        rel_tol=1e-11,
        abs_tol=1e-11,
    )
    assert math.isclose(
        first_sample["rocker_omega_rad_s"],
        first_expected["velocity"]["omega4"],
        rel_tol=1e-10,
        abs_tol=1e-10,
    )
    assert math.isclose(
        first_sample["drive_torque_n_m"],
        first_expected["load"]["drive_torque"],
        rel_tol=1e-10,
        abs_tol=1e-10,
    )
    assert matched
