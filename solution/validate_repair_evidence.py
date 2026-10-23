#!/usr/bin/env python3
"""Validate traceability, mutation, and difficulty evidence as one repair gate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


TASK = Path(__file__).resolve().parents[1]


def load(path: Path) -> dict:
    """Load one JSON evidence file."""
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    """Reject incomplete repair evidence before delivery."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--difficulty-evidence",
        type=Path,
        default=TASK.parent / "review" / TASK.name / "gates" / "difficulty-evidence.json",
    )
    arguments = parser.parse_args()

    traceability = load(TASK / "solution" / "test-traceability.json")
    mutations = load(TASK / "solution" / "mutation-results.json")
    difficulty = load(arguments.difficulty_evidence.expanduser().resolve())
    errors = []

    if traceability.get("status") != "pass":
        errors.append("行为追溯状态不是 pass")
    trace_summary = traceability.get("summary", {})
    if trace_summary.get("test_count") != 17:
        errors.append("行为追溯未覆盖 17 个测试")
    if trace_summary.get("unique_case_ids") != 17:
        errors.append("行为追溯的 case_id 不是 17 个唯一值")
    if trace_summary.get("exact_instruction_quotes") != 17:
        errors.append("并非 17 个测试都引用题面原文")
    if trace_summary.get("direct_assertion_tests") != 17:
        errors.append("并非 17 个测试都有直接断言")

    trace_cases = {
        record["case_id"] for record in traceability.get("records", [])
    }
    mutant_cases = {
        mutant["case_id"] for mutant in mutations.get("mutants", [])
    }
    if trace_cases != mutant_cases:
        errors.append("变异 case_id 与行为追溯 case_id 不一致")
    if mutations.get("status") != "pass":
        errors.append("变异证据状态不是 pass")
    mutation_summary = mutations.get("summary", {})
    if mutation_summary.get("mutants") != 17:
        errors.append("变异数量不是 17")
    if mutation_summary.get("killed") != 17:
        errors.append("并非 17 个变异全部被杀死")
    if mutation_summary.get("survived") != 0:
        errors.append("仍存在存活变异")
    for mutant in mutations.get("mutants", []):
        evidence = TASK / mutant["evidence"]
        if not evidence.is_file() or evidence.stat().st_size == 0:
            errors.append(f"变异证据缺失: {mutant['evidence']}")

    if difficulty.get("status") != "pass":
        errors.append("难度证据状态不是 pass")
    axes = difficulty.get("non_direct_solving_axes", [])
    steps = difficulty.get("structured_solution_diagnosis", {}).get("steps", [])
    if len(axes) < 4:
        errors.append("非直译求解轴少于 4 条")
    if len(steps) < 5:
        errors.append("结构化解法链少于 5 步")
    calibration = difficulty.get("calibration", {})
    if calibration.get("n_completed_trials") != 1:
        errors.append("校准 trial 未完成")
    if calibration.get("n_errored_trials") != 0:
        errors.append("校准 trial 存在代理错误")
    mean = calibration.get("mean")
    if not isinstance(mean, (int, float)) or not 0 <= mean < 1:
        errors.append("校准均值必须是未满分的数值")
    for relative in [
        calibration.get("result_evidence"),
        *calibration.get("group_evidence", {}).values(),
    ]:
        if not isinstance(relative, str) or not (TASK / relative).is_file():
            errors.append(f"校准证据缺失: {relative}")

    if errors:
        print("FAIL: 返修证据门禁未通过")
        for error in errors:
            print(f"  - {error}")
        return 1
    print(
        "PASS: 17 个测试可追溯，17 个变异全部被杀，"
        "非直译轴、结构化解法链与未满分校准齐全"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
