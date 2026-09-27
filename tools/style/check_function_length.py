#!/usr/bin/env python3
"""Check that no function is longer than 100 lines.

Architecture:
    AGENTS.md ("Function length") caps every function at 100 lines, measured
    from its signature to its closing brace, blank lines and comments
    included, because the rule is about fitting on one screen. lizard parses
    C, C++, and Python and reports that span as `length`.

    Functions that were already longer than the cap are listed in
    function_length_baseline.txt with their length. A listed function may
    shrink but not grow, and any function not listed must be within the cap.
    When a listed function is brought within the cap or removed, its entry is
    reported as stale so it can be deleted and the cap then applies to it.

Usage:
    tools/style/check_function_length.py

Verification:
    Correct means: exit status 1 when an unlisted function exceeds 100 lines
    or a listed one exceeds its recorded length, and 0 otherwise; stale
    entries are reported but do not fail the check. Checked by running it on
    the tree (passes) and after lengthening a listed function (fails).

References:
    AGENTS.md, "Coding Style" > "Function length".
    lizard: https://github.com/terryyin/lizard (CSV columns: NLOC, CCN, token,
    PARAM, length, location, file, function, long_name, start, end).
"""

import csv
import io
import pathlib
import subprocess
import sys

MAX_LINES = 100
_SOURCE_SUFFIXES = (".c", ".cc", ".cpp", ".h", ".hpp", ".py")
_BASELINE = pathlib.Path(__file__).with_name("function_length_baseline.txt")


def load_baseline() -> dict:
    """Reads the recorded lengths of functions that exceed the cap.

    Returns:
        A dict from `path::function` to its recorded length in lines. Blank
        lines and `#` comments in the baseline file are ignored.

    Verification:
        Correct means: the line `src/a.cpp::f 120` maps `src/a.cpp::f` to 120.
    """
    baseline = {}
    for line in _BASELINE.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            key, length = line.rsplit(maxsplit=1)
            baseline[key] = int(length)

    return baseline


def measure_functions() -> dict:
    """Measures every function in the tracked C, C++, and Python sources.

    Returns:
        A dict from `path::function` to its length in lines. When one file
        defines two functions with the same qualified name (overloads), the
        longer length is kept.

    Raises:
        subprocess.CalledProcessError: git or lizard failed to run.
    """
    files = subprocess.run(
        ["git", "ls-files", "--", *(f"*{suffix}" for suffix in _SOURCE_SUFFIXES)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.split()
    report = subprocess.run(
        [sys.executable, "-m", "lizard", "--csv", "-f", "/dev/stdin"],
        input="\n".join(files),
        check=True,
        capture_output=True,
        text=True,
    ).stdout

    lengths = {}
    for row in csv.reader(io.StringIO(report)):
        length, path, function = int(row[4]), row[6], row[7]
        key = f"{path}::{function}"
        lengths[key] = max(length, lengths.get(key, 0))

    return lengths


def main() -> int:
    """Reports functions over the cap and baseline entries that have gone stale.

    Returns:
        0 when every function is within the cap or its baseline, 1 otherwise.
    """
    baseline = load_baseline()
    lengths = measure_functions()

    failures = 0
    for key, length in sorted(lengths.items()):
        limit = baseline.get(key, MAX_LINES)
        if length > limit:
            print(f"{key}: {length} lines, limit {limit}")
            failures += 1

    for key in sorted(baseline):
        if lengths.get(key, 0) <= MAX_LINES:
            print(f"note: {key} is within {MAX_LINES} lines; remove it from {_BASELINE.name}")

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
