"""Run one-fix and bypass specimens through the real test script."""

import json
import shutil
import subprocess
import tempfile
from pathlib import Path


TASK = Path(__file__).resolve().parents[1]
IMAGE = "harbor-quality:toggle-linkage-kinematics"
GROUPS = ("c1", "c2", "c3", "c4", "c5", "e2e")


def run_command(args, **kwargs):
    """Run a command and capture text output."""
    return subprocess.run(args, text=True, capture_output=True, check=False, **kwargs)


def specimen(root, label, files, override=None):
    """Create one workspace and run its complete grading script."""
    work = root / label / "workspace"
    shutil.copytree(TASK / "environment" / "workspace", work)
    for source_name, destination_name in files:
        shutil.copy2(
            TASK / "solution" / source_name,
            work / "toggle_linkage" / destination_name,
        )
    if override is not None:
        override(work)
    logs = root / label / "logs"
    verifier = logs / "verifier"
    verifier.mkdir(parents=True)
    for directory in (root / label, logs, verifier):
        directory.chmod(0o777)
    command = [
        "docker",
        "run",
        "--rm",
        "--user",
        "10001:10001",
        "--network",
        "host",
        "-v",
        f"{work}:/workspace:ro",
        "-v",
        f"{TASK / 'tests'}:/tests:ro",
        "-v",
        f"{logs}:/logs",
        IMAGE,
        "bash",
        "/tests/test.sh",
    ]
    outcome = run_command(command)
    reward_path = verifier / "reward.txt"
    reward = (
        float(reward_path.read_text(encoding="utf-8").strip())
        if reward_path.exists()
        else None
    )
    evidence = TASK / "solution" / "evidence" / f"{label}.log"
    group_logs = {}
    for group in GROUPS:
        group_log = verifier / f"{group}.log"
        group_logs[group] = (
            group_log.read_text(encoding="utf-8") if group_log.exists() else "missing"
        )
    evidence.write_text(
        "Command: "
        + " ".join(command)
        + f"\nExit: {outcome.returncode}\nReward: {reward}\n"
        + "stdout:\n"
        + outcome.stdout
        + "\nstderr:\n"
        + outcome.stderr
        + "".join(f"\n[{group}]\n{log}" for group, log in group_logs.items()),
        encoding="utf-8",
    )
    if reward is None:
        raise RuntimeError(f"{label}: no reward; inspect {evidence}")
    print(f"{label}: {reward}")
    return {
        "reward": reward,
        "evidence": str(evidence.relative_to(TASK)),
    }


def empty_output(work):
    """Replace the command with an empty sample collection."""
    (work / "toggle_linkage" / "cli.py").write_text(
        "import json, sys\n"
        "from pathlib import Path\n"
        "report = {'mechanism': {}, 'samples': [], 'summary': {}}\n"
        "Path(sys.argv[2]).write_text(json.dumps(report), encoding='utf-8')\n",
        encoding="utf-8",
    )


def fixed_output(work):
    """Replace the command with three copies of a structurally valid sample."""
    sample = {
        "crank_angle_deg": 0.0,
        "crank_angle_rad": 0.0,
        "coupler_angle_deg": 0.0,
        "rocker_angle_deg": 0.0,
        "coupler_point_m": [0.0, 0.0],
        "load_point_m": [0.0, 0.0],
        "coupler_omega_rad_s": 0.0,
        "rocker_omega_rad_s": 0.0,
        "coupler_alpha_rad_s2": 0.0,
        "rocker_alpha_rad_s2": 0.0,
        "transmission_angle_deg": 0.0,
        "load_torque_n_m": 0.0,
        "gravity_generalized_torque_n_m": 0.0,
        "spring_generalized_torque_n_m": 0.0,
        "inertial_generalized_torque_n_m": 0.0,
        "friction_loss_w": 0.0,
        "drive_torque_n_m": 0.0,
        "load_power_w": 0.0,
        "gravity_power_w": 0.0,
        "spring_power_w": 0.0,
        "kinetic_power_w": 0.0,
        "drive_power_w": 0.0,
    }
    report = {
        "mechanism": {"grashof_class": "crank-rocker", "shortest_link": "crank", "branch": 1},
        "samples": [sample, sample, sample],
        "summary": {
            "minimum_transmission_angle_deg": 0.0,
            "maximum_abs_coupler_speed_rad_s": 0.0,
            "maximum_abs_rocker_speed_rad_s": 0.0,
            "maximum_abs_drive_torque_n_m": 0.0,
            "maximum_friction_loss_w": 0.0,
            "maximum_abs_kinetic_power_w": 0.0,
        },
    }
    (work / "toggle_linkage" / "cli.py").write_text(
        "import json, sys\n"
        "from pathlib import Path\n"
        f"report = {report!r}\n"
        "Path(sys.argv[2]).write_text(json.dumps(report), encoding='utf-8')\n",
        encoding="utf-8",
    )


def drop_load_module(work):
    """Delete the force-resolution module."""
    (work / "toggle_linkage" / "loads.py").unlink()


def main():
    """Build the image and record every experiment."""
    build = run_command(
        ["docker", "build", "-q", "-t", IMAGE, str(TASK / "environment")]
    )
    if build.returncode:
        raise RuntimeError(build.stderr)
    (TASK / "solution" / "evidence").mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="toggle-linkage-quality-") as temp:
        root = Path(temp)
        root.chmod(0o777)
        baseline = specimen(root, "baseline", [])
        oracle = specimen(
            root,
            "oracle",
            [
                ("geometry.py", "geometry.py"),
                ("kinematics.py", "kinematics.py"),
                ("loads.py", "loads.py"),
                ("cli.py", "cli.py"),
            ],
        )
        partials = [
            {
                "challenge_id": "C1",
                **specimen(root, "partial-c1", [("geometry.py", "geometry.py")]),
            },
            {
                "challenge_id": "C2",
                **specimen(
                    root,
                    "partial-c2",
                    [("kinematics_velocity.py", "kinematics.py")],
                ),
            },
            {
                "challenge_id": "C3",
                **specimen(root, "partial-c3", [("kinematics.py", "kinematics.py")]),
            },
            {
                "challenge_id": "C4",
                **specimen(root, "partial-c4", [("loads.py", "loads.py")]),
            },
            {
                "challenge_id": "C5",
                **specimen(root, "partial-c5", [("cli.py", "cli.py")]),
            },
        ]
        bypasses = [
            {
                "id": "B1",
                "description": "命令只返回空的 samples 数组",
                **specimen(root, "bypass-empty", [], empty_output),
            },
            {
                "id": "B2",
                "description": "命令固定回放三条结构正确的零值记录",
                **specimen(root, "bypass-fixed", [], fixed_output),
            },
            {
                "id": "B3",
                "description": "删除载荷求解模块",
                **specimen(root, "bypass-delete", [], drop_load_module),
            },
        ]
    report = {
        "baseline": baseline,
        "oracle": oracle,
        "partials": partials,
        "bypasses": bypasses,
    }
    (TASK / "solution" / "quality-gate-results.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    if baseline["reward"] != 0 or oracle["reward"] != 1:
        raise RuntimeError("baseline/oracle gate failed")
    if any(not 0 < item["reward"] < 1 for item in partials):
        raise RuntimeError("partial gate failed")
    if any(item["reward"] == 1 for item in bypasses):
        raise RuntimeError("bypass gate failed")


if __name__ == "__main__":
    main()
