#!/usr/bin/env python3
"""Generate standards coverage report from matrix + passed test IDs."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--matrix", required=True, type=Path)
    ap.add_argument("--passed", required=True, type=Path)
    args = ap.parse_args()

    matrix = load_json(args.matrix)
    passed_blob = load_json(args.passed)
    passed_tests = set(passed_blob.get("passed_test_ids", []))

    statements = matrix.get("statements", [])
    implemented = [s for s in statements if s.get("status") == "implemented"]
    covered = []
    missing = []

    for stmt in implemented:
        tests = stmt.get("test_ids", [])
        if any(t in passed_tests for t in tests):
            covered.append(stmt)
        else:
            missing.append(stmt)

    spec = matrix.get("spec", {})
    print(
        f"Spec coverage for {matrix.get('target', '<unknown>')}\n"
        f"  Spec: {spec.get('name', '?')} {spec.get('version', '?')} "
        f"({spec.get('document', '?')})"
    )
    print(f"  Implemented statements: {len(implemented)}")
    print(f"  Covered statements:     {len(covered)}")
    pct = (100.0 * len(covered) / len(implemented)) if implemented else 0.0
    print(f"  Coverage:               {pct:.1f}%")

    if missing:
        print("\nMissing implemented statements:")
        for stmt in missing:
            print(f"  - {stmt.get('id')}: {stmt.get('statement')}")
        return 2

    print("\nAll implemented statements covered.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

