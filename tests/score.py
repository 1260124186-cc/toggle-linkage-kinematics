#!/usr/bin/env python3
"""Convert a trusted JUnit report into the normalized task reward."""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def write_reward(path: Path, reward: float) -> None:
    """Write the single reward value consumed by Harbor."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f"{reward:.6f}\n", encoding="utf-8")


def write_status(path: Path, message: str) -> None:
    """Write an explicit terminal verifier status."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(message.rstrip() + "\n", encoding="utf-8")


def group_for(test_name: str) -> str:
    """Map one test function to its declared scoring group."""
    if test_name.startswith("test_e2e_"):
        return "e2e"
    parts = test_name.split("_")
    if len(parts) < 2:
        raise ValueError(f"cannot infer scoring group: {test_name}")
    return parts[1]


def fail_closed(
    reward_path: Path,
    status_path: Path,
    report_path: Path,
    message: str,
) -> int:
    """Write a completed but failing score and return success to the runner."""
    write_reward(reward_path, 0.0)
    write_status(status_path, f"COMPLETED: weighted scoring failed closed: {message}")
    report_path.write_text(
        json.dumps({"status": "fail", "error": message, "reward": 0.0}, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0


def main() -> int:
    """Parse JUnit, apply group-level weights, and write all result artifacts."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--junit", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--reward-out", required=True)
    parser.add_argument("--status-out", required=True)
    parser.add_argument("--report-out", required=True)
    args = parser.parse_args()

    reward_path = Path(args.reward_out)
    status_path = Path(args.status_out)
    report_path = Path(args.report_out)
    write_reward(reward_path, 0.0)
    write_status(status_path, "INCOMPLETE: weighted scoring started")

    try:
        config = json.loads(Path(args.config).read_text(encoding="utf-8"))
        entries = config["tests"]
        expected = {
            test_id.split("::", 1)[1]: float(value["weight"])
            for test_id, value in entries.items()
        }
        if not expected:
            raise ValueError("scoring manifest has no tests")
        if not math.isclose(math.fsum(expected.values()), 1.0, abs_tol=1e-9):
            raise ValueError("scoring weights do not sum to 1")

        root = ET.parse(args.junit).getroot()
        outcomes: dict[str, bool] = {}
        for testcase in root.iter("testcase"):
            name = testcase.attrib.get("name", "")
            if not name:
                continue
            passed = not any(
                child.tag in {"failure", "error", "skipped"} for child in testcase
            )
            if name in outcomes:
                raise ValueError(f"duplicate testcase in JUnit: {name}")
            outcomes[name] = passed

        missing = sorted(set(expected) - set(outcomes))
        if missing:
            raise ValueError(f"JUnit missing tests: {missing}")

        groups: dict[str, list[str]] = {}
        for test_name in expected:
            groups.setdefault(group_for(test_name), []).append(test_name)
        group_results = {}
        earned = 0.0
        for group, names in sorted(groups.items()):
            passed = all(outcomes[name] for name in names)
            weight = math.fsum(expected[name] for name in names)
            group_results[group] = {
                "passed": passed,
                "weight": weight,
                "tests": {name: outcomes[name] for name in sorted(names)},
            }
            if passed:
                earned += weight

        reward = min(1.0, max(0.0, earned))
        write_reward(reward_path, reward)
        write_status(
            status_path,
            f"COMPLETED: weighted scoring finished with reward={reward:.6f}",
        )
        report_path.write_text(
            json.dumps(
                {
                    "status": "pass",
                    "reward": reward,
                    "groups": group_results,
                    "tests": outcomes,
                },
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        return 0
    except Exception as exc:  # noqa: BLE001
        return fail_closed(reward_path, status_path, report_path, str(exc))


if __name__ == "__main__":
    sys.exit(main())
