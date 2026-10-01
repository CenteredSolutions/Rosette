#!/usr/bin/env python3
"""Check that a closing brace on its own line is followed by a blank line.

Architecture:
    AGENTS.md ("Blocks and vertical whitespace") requires a blank line after a
    `}` that sits on a line by itself, so the end of a loop or branch is
    visually separated from the statement after it. clang-format has no option
    for this, so this script checks it. Only lines the branch adds or modifies
    are held to the rule (see changed_lines.py); untouched code is reported
    once it is edited.

    A `}` line may be followed directly by another `}`, by `else`, by the
    `while` of a `do` loop, by a `case` or `default` label, by `catch`, or by
    a preprocessor directive. A `}` with a trailing `//` comment counts as on
    its own line; `};`, `});`, and `} else {` do not.

Usage:
    tools/style/check_blank_after_brace.py --base origin/main

Verification:
    Correct means: exit status 1 and one `path:line` report per violation when
    a changed `}` line (or the changed line after it) breaks the rule, and
    exit status 0 otherwise. Checked against the AGENTS.md examples: the
    `for` loop example passes, and removing its blank line before `return`
    is reported.

References:
    AGENTS.md, "Coding Style" > "Blocks and vertical whitespace".
"""

import argparse
import pathlib
import re
import sys

from changed_lines import changed_lines, merge_base

_LONE_CLOSE = re.compile(r"^\s*\}\s*(//.*)?$")
_ALLOWED_NEXT = re.compile(r"^\s*(\}|else\b|while\b|case\b|default\b|catch\b|#)")


def find_violations(lines: list, changed: set) -> list:
    """Lists the 1-based line numbers of `}` lines missing the blank line after them.

    Args:
        lines: The file's lines, without line terminators.
        changed: 1-based line numbers the branch adds or modifies. A violation
            is reported when either the `}` line or the line after it is in
            this set.

    Returns:
        The 1-based numbers of offending `}` lines, in file order.

    Verification:
        Correct means: `}` then `return x;` is reported; `}` then a blank
        line, `} else {`, another `}`, `case 2:`, or `#endif` is not.

    References:
        AGENTS.md, "Coding Style" > "Blocks and vertical whitespace".
    """
    violations = []
    for index in range(len(lines) - 1):
        number = index + 1
        if number not in changed and number + 1 not in changed:
            continue

        following = lines[index + 1]
        if (
            _LONE_CLOSE.match(lines[index])
            and following.strip()
            and not _ALLOWED_NEXT.match(following)
        ):
            violations.append(number)

    return violations


def main() -> int:
    """Reports blank-line violations in the lines changed since the merge base.

    Returns:
        0 when no changed line breaks the rule, 1 otherwise.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--base", default="origin/main", help="revision to compare against")
    args = parser.parse_args()

    failures = 0
    for path, changed in sorted(changed_lines(merge_base(args.base)).items()):
        lines = pathlib.Path(path).read_text(encoding="utf-8").splitlines()
        for number in find_violations(lines, changed):
            print(f"{path}:{number}: add a blank line after this closing brace")
            failures += 1

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
