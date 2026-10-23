#!/usr/bin/env python3
"""Validate and emit the test-to-instruction traceability ledger."""

from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path


TASK = Path(__file__).resolve().parents[1]
TEST_FILE = TASK / "tests" / "test_outputs.py"
INSTRUCTION_FILE = TASK / "instruction.md"
DESIGN_FILE = TASK / "solution" / "design-gates.json"
OUTPUT_FILE = TASK / "solution" / "test-traceability.json"


def decorator_metadata(node: ast.FunctionDef) -> dict[str, str] | None:
    """Return case metadata from the local case_contract decorator."""
    for decorator in node.decorator_list:
        if (
            isinstance(decorator, ast.Call)
            and isinstance(decorator.func, ast.Name)
            and decorator.func.id == "case_contract"
        ):
            values = [ast.literal_eval(argument) for argument in decorator.args]
            keys = ("case_id", "requirement_id", "test_id", "instruction_quote")
            return dict(zip(keys, values))
    return None


def main() -> int:
    """Run the audit and optionally write a machine-readable ledger."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--write", action="store_true")
    arguments = parser.parse_args()

    instruction = INSTRUCTION_FILE.read_text(encoding="utf-8")
    design = json.loads(DESIGN_FILE.read_text(encoding="utf-8"))
    tree = ast.parse(TEST_FILE.read_text(encoding="utf-8"))
    design_tests = {item["id"]: item for item in design["tests"]}
    records = []
    errors = []
    case_ids = set()
    test_ids = set()

    for node in tree.body:
        if not isinstance(node, ast.FunctionDef) or not node.name.startswith("test_"):
            continue
        metadata = decorator_metadata(node)
        if metadata is None:
            errors.append(f"{node.name} 缺少 case_contract 装饰器")
            continue
        case_id = metadata["case_id"]
        test_id = metadata["test_id"]
        requirement_ids = [
            value.strip()
            for value in metadata["requirement_id"].split(",")
            if value.strip()
        ]
        if case_id in case_ids:
            errors.append(f"重复 case_id: {case_id}")
        if test_id in test_ids:
            errors.append(f"重复 test_id: {test_id}")
        case_ids.add(case_id)
        test_ids.add(test_id)
        if case_id not in (ast.get_docstring(node) or ""):
            errors.append(f"{node.name} 的 docstring 未包含 {case_id}")
        if metadata["instruction_quote"] not in instruction:
            errors.append(f"{case_id} 的题面引用不是原文子串")
        design_test = design_tests.get(test_id)
        if design_test is None:
            errors.append(f"{case_id} 引用设计中不存在的 {test_id}")
        elif design_test.get("requirement_ids") != requirement_ids:
            errors.append(
                f"{case_id} 的 {test_id} 需求映射不一致: "
                f"{requirement_ids} != {design_test.get('requirement_ids')}"
            )
        assertion_count = sum(
            isinstance(child, ast.Assert) for child in ast.walk(node)
        )
        if assertion_count == 0:
            errors.append(f"{case_id} 没有直接断言")
        records.append(
            {
                "case_id": case_id,
                "function": node.name,
                "requirement_ids": requirement_ids,
                "test_id": test_id,
                "instruction_quote": metadata["instruction_quote"],
                "assertion_count": assertion_count,
                "source": "tests/test_outputs.py",
            }
        )

    expected_test_ids = [f"T{index}" for index in range(1, 18)]
    if sorted(test_ids, key=lambda value: int(value[1:])) != expected_test_ids:
        errors.append(f"测试 ID 未完整覆盖 T1-T17: {sorted(test_ids)}")

    report = {
        "status": "pass" if not errors else "fail",
        "summary": {
            "test_count": len(records),
            "unique_case_ids": len(case_ids),
            "exact_instruction_quotes": sum(
                record["instruction_quote"] in instruction for record in records
            ),
            "direct_assertion_tests": sum(
                record["assertion_count"] > 0 for record in records
            ),
        },
        "records": records,
        "errors": errors,
    }
    if arguments.write:
        OUTPUT_FILE.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    if errors:
        print("FAIL: 行为追溯未通过")
        for error in errors:
            print(f"  - {error}")
        return 1
    print(
        "PASS: 17 个测试均有唯一 case_id、原文引用、R/T 绑定和直接断言"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
