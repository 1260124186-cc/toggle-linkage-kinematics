#!/usr/bin/env python3
"""Run one executable mutant against each traceable pytest case."""

from __future__ import annotations

import json
import shutil
import subprocess
import tempfile
from pathlib import Path


TASK = Path(__file__).resolve().parents[1]
IMAGE = "harbor-mutation:toggle-linkage-kinematics"
SOLUTION_FILES = ("geometry.py", "kinematics.py", "loads.py", "cli.py")

MUTANTS = [
    {
        "case_id": "CASE-C1-01",
        "file": "geometry.py",
        "old": "    bx = ax + along * ux + branch * height * perp_x\n"
        "    by = ay + along * uy + branch * height * perp_y",
        "new": "    bx = ax + along * ux - branch * height * perp_x\n"
        "    by = ay + along * uy - branch * height * perp_y",
        "description": "反转 branch=1 的左侧法向符号",
    },
    {
        "case_id": "CASE-C1-02",
        "file": "geometry.py",
        "old": "along = (coupler * coupler - rocker * rocker + distance * distance) / (",
        "new": "along = (coupler * coupler + distance * distance) / (",
        "description": "从圆交点投影中删去摇杆长度平方",
    },
    {
        "case_id": "CASE-C1-03",
        "file": "geometry.py",
        "old": '    branch = int(mechanism["branch"])',
        "new": "    branch = 1",
        "description": "忽略 branch 输入并固定选择左侧交点",
    },
    {
        "case_id": "CASE-C1-04",
        "file": "geometry.py",
        "old": '        return "crank-rocker"',
        "new": '        return "other"',
        "description": "让 Grashof 分类始终返回 other",
    },
    {
        "case_id": "CASE-C2-01",
        "file": "kinematics.py",
        "old": "        crank * math.sin(theta2) * omega2,",
        "new": "        -crank * math.sin(theta2) * omega2,",
        "description": "反转速度方程第一行的曲柄速度项",
    },
    {
        "case_id": "CASE-C2-02",
        "file": "kinematics.py",
        "old": "        -rocker * math.cos(theta4),",
        "new": "        rocker * math.cos(theta4),",
        "description": "反转速度矩阵中摇杆余弦项的符号",
    },
    {
        "case_id": "CASE-C2-03",
        "file": "kinematics.py",
        "old": '        "rocker_omega_rad_s": omega4,',
        "new": '        "rocker_omega_rad_s": omega4 + 0.01,',
        "description": "给摇杆角速度叠加非物理的固定偏置",
    },
    {
        "case_id": "CASE-C3-01",
        "file": "kinematics.py",
        "old": "        + crank * c2 * omega2 * omega2",
        "new": "        - crank * c2 * omega2 * omega2",
        "description": "反转加速度方程中曲柄向心项",
    },
    {
        "case_id": "CASE-C3-02",
        "file": "kinematics.py",
        "old": "        - rocker * s4 * omega4 * omega4,",
        "new": "        + rocker * s4 * omega4 * omega4,",
        "description": "反转加速度方程第二行摇杆向心项",
    },
    {
        "case_id": "CASE-C3-03",
        "file": "kinematics.py",
        "old": "        crank * s2 * alpha2\n"
        "        + crank * c2 * omega2 * omega2\n"
        "        + coupler * c3 * omega3 * omega3\n"
        "        - rocker * c4 * omega4 * omega4,\n"
        "        -crank * c2 * alpha2\n"
        "        + crank * s2 * omega2 * omega2\n"
        "        + coupler * s3 * omega3 * omega3\n"
        "        - rocker * s4 * omega4 * omega4,",
        "new": "        crank * s2 * alpha2,\n"
        "        -crank * c2 * alpha2,",
        "description": "删掉加速度方程中的全部向心项",
    },
    {
        "case_id": "CASE-C4-01",
        "file": "loads.py",
        "old": "    rx = ratio * (bx - ground)\n    ry = ratio * by",
        "new": "    rx = bx - ground\n    ry = by",
        "description": "忽略 application_ratio 并把载荷加到摇杆末端",
    },
    {
        "case_id": "CASE-C4-02",
        "file": "kinematics.py",
        "old": "    return math.degrees(min(wrapped, math.pi - wrapped))",
        "new": "    return math.degrees(wrapped) + 90.0",
        "description": "将传动角错误平移 90 度",
    },
    {
        "case_id": "CASE-C4-03",
        "file": "loads.py",
        "old": "        + friction_loss / omega2",
        "new": "        - friction_loss / omega2",
        "description": "把摩擦耗散当作驱动侧输入",
    },
    {
        "case_id": "CASE-C5-01",
        "file": "cli.py",
        "old": '                "kinetic_power_w": load_result["kinetic_power_w"],\n',
        "new": "",
        "description": "从每个 sample 中删除 kinetic_power_w",
    },
    {
        "case_id": "CASE-C5-02",
        "file": "cli.py",
        "old": "    target.parent.mkdir(parents=True, exist_ok=True)",
        "new": "    pass",
        "description": "不再创建缺失的报告父目录",
    },
    {
        "case_id": "CASE-C5-03",
        "file": "cli.py",
        "old": "    if not 0 < ratio <= 1:",
        "new": "    if False:",
        "description": "跳过非法载荷比例校验",
    },
    {
        "case_id": "CASE-E2E-01",
        "file": "loads.py",
        "old": '    spring_torque = -float(return_spring["stiffness_n_m_per_rad"]) * (',
        "new": '    spring_torque = float(return_spring["stiffness_n_m_per_rad"]) * (',
        "description": "反转回位扭簧力矩符号",
    },
]


def run_command(arguments: list[str], timeout: int = 60) -> subprocess.CompletedProcess:
    """Run one subprocess with captured text output."""
    return subprocess.run(
        arguments,
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


def prepare_workspace(target: Path) -> None:
    """Copy the agent workspace and overlay the reference implementation."""
    shutil.copytree(TASK / "environment" / "workspace", target)
    for filename in SOLUTION_FILES:
        shutil.copy2(
            TASK / "solution" / filename,
            target / "toggle_linkage" / filename,
        )
    target.chmod(0o755)


def run_pytest(workspace: Path, test_name: str | None) -> subprocess.CompletedProcess:
    """Run one traceable test or the complete suite in the test image."""
    target = "/tests/test_outputs.py"
    if test_name:
        target = f"{target}::{test_name}"
    return run_command(
        [
            "docker",
            "run",
            "--rm",
            "--user",
            "10001:10001",
            "-v",
            f"{workspace}:/workspace:ro",
            "-v",
            f"{TASK / 'tests'}:/tests:ro",
            IMAGE,
            "python",
            "-m",
            "pytest",
            "-q",
            target,
        ]
    )


def main() -> int:
    """Build the mutation image, kill every mutant, and record evidence."""
    traceability = json.loads(
        (TASK / "solution" / "test-traceability.json").read_text(encoding="utf-8")
    )
    cases = {record["case_id"]: record for record in traceability["records"]}
    if len(cases) != 17 or len(MUTANTS) != 17:
        raise SystemExit("追溯表与变异表必须各覆盖 17 个 case")

    build = run_command(
        [
            "docker",
            "build",
            "-q",
            "-f",
            str(TASK / "solution" / "mutation_environment" / "Dockerfile"),
            "-t",
            IMAGE,
            str(TASK),
        ],
        timeout=600,
    )
    if build.returncode != 0:
        raise SystemExit(build.stderr)

    evidence_dir = TASK / "solution" / "evidence"
    evidence_dir.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="toggle-linkage-mutants-") as temp:
        root = Path(temp)
        root.chmod(0o755)
        baseline_workspace = root / "baseline"
        prepare_workspace(baseline_workspace)
        baseline = run_pytest(baseline_workspace, None)
        baseline_log = evidence_dir / "mutation-baseline.log"
        baseline_log.write_text(
            "Command: pytest -q /tests/test_outputs.py\n"
            f"Exit: {baseline.returncode}\nstdout:\n{baseline.stdout}\n"
            f"stderr:\n{baseline.stderr}\n",
            encoding="utf-8",
        )
        if baseline.returncode != 0:
            raise SystemExit(f"reference baseline failed: {baseline.stdout}")

        results = []
        for mutant in MUTANTS:
            case = cases[mutant["case_id"]]
            workspace = root / mutant["case_id"].lower()
            prepare_workspace(workspace)
            source_path = workspace / "toggle_linkage" / mutant["file"]
            source = source_path.read_text(encoding="utf-8")
            occurrences = source.count(mutant["old"])
            if occurrences != 1:
                raise SystemExit(
                    f"{mutant['case_id']} target occurs {occurrences} times in "
                    f"{mutant['file']}"
                )
            source_path.write_text(
                source.replace(mutant["old"], mutant["new"], 1),
                encoding="utf-8",
            )
            outcome = run_pytest(workspace, case["function"])
            evidence_path = evidence_dir / f"mutation-{mutant['case_id']}.log"
            evidence_path.write_text(
                "Command: "
                f"pytest -q /tests/test_outputs.py::{case['function']}\n"
                f"Mutation: {mutant['description']}\n"
                f"File: {mutant['file']}\n"
                f"Exit: {outcome.returncode}\n"
                f"stdout:\n{outcome.stdout}\nstderr:\n{outcome.stderr}\n",
                encoding="utf-8",
            )
            results.append(
                {
                    "case_id": mutant["case_id"],
                    "test_id": case["test_id"],
                    "test": case["function"],
                    "file": mutant["file"],
                    "description": mutant["description"],
                    "killed": outcome.returncode != 0,
                    "exit_code": outcome.returncode,
                    "evidence": str(evidence_path.relative_to(TASK)),
                }
            )
            print(
                f"{mutant['case_id']}: "
                f"{'KILLED' if outcome.returncode != 0 else 'SURVIVED'}"
            )

    survivors = [item for item in results if not item["killed"]]
    report = {
        "status": "pass" if not survivors else "fail",
        "baseline": {
            "tests_passed": 17,
            "exit_code": baseline.returncode,
            "evidence": "solution/evidence/mutation-baseline.log",
        },
        "summary": {
            "mutants": len(results),
            "killed": len(results) - len(survivors),
            "survived": len(survivors),
        },
        "mutants": results,
    }
    (TASK / "solution" / "mutation-results.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 1 if survivors else 0


if __name__ == "__main__":
    raise SystemExit(main())
