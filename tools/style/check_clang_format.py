#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
"""Check changed lines against clang-format, leaving column alignment to the author.

Architecture:
    `git clang-format --diff` proposes the formatting for lines changed since
    the base commit. clang-format can only force alignment on or off: it pads
    consecutive declarations and trailing comments into a column, or collapses
    them to one space. Neither suits every block, so a hunk whose removed and
    added lines differ only in the run of spaces between two tokens is
    accepted. Indentation, line breaks, braces, and the presence of a space
    are still enforced.

Usage:
    tools/style/check_clang_format.py --base <commit>
"""

import argparse
import re
import subprocess
import sys

_EXTENSIONS = "c,cc,cpp,h,hpp"
_INTERIOR_SPACES = re.compile(r"(?<=\S)[ \t]+")


def _normalize(lines):
    """Collapse each space run after a line's indentation to one space."""
    return [_INTERIOR_SPACES.sub(" ", line.rstrip()) for line in lines]


def _is_alignment_only(hunk):
    """Return whether a hunk's removed and added lines match once spaces collapse."""
    removed = [line[1:] for line in hunk if line.startswith("-")]
    added = [line[1:] for line in hunk if line.startswith("+")]
    return _normalize(removed) == _normalize(added)


def _hunks(diff):
    """Yield (file header, hunk lines) for each hunk of a unified diff."""
    header, hunk = [], None
    for line in diff.splitlines():
        if line.startswith("diff --git"):
            if hunk:
                yield header, hunk

            header, hunk = [line], None
        elif line.startswith("@@"):
            if hunk:
                yield header, hunk

            hunk = [line]
        elif hunk is not None:
            hunk.append(line)
        else:
            header.append(line)

    if hunk:
        yield header, hunk


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--base", required=True, help="commit the changed lines are measured from")
    args = parser.parse_args()

    diff = subprocess.run(
        ["git", "clang-format", "--diff", "--extensions", _EXTENSIONS, args.base],
        capture_output=True,
        text=True,
        check=False,
    ).stdout
    failing = [(header, hunk) for header, hunk in _hunks(diff) if not _is_alignment_only(hunk)]
    for header, hunk in failing:
        print("\n".join(header + hunk))

    if failing:
        print(f"Run: git clang-format --extensions {_EXTENSIONS} {args.base}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
