# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

"""Find the source lines a branch adds or modifies relative to a base revision.

Architecture:
    The style checks in tools/style/ hold new and edited code to the AGENTS.md
    rules without requiring a one-time reformat of the whole tree. This module
    turns `git diff -U0` into a map from file path to the set of line numbers
    that differ from the merge base. The diff is taken against the working
    tree, so the same answer serves a clean CI checkout and a developer's
    uncommitted edits.

Verification:
    Correct means: every line reported is present in the working-tree file and
    absent from (or different in) the merge-base version, and deleted lines
    are never reported. Checked by running a check against a branch with a
    known edit and confirming only that edit is flagged.

References:
    git-diff(1), "Combined diff format" and the unified hunk header
    `@@ -a,b +c,d @@`.
"""

import re
import subprocess

_HUNK_HEADER = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,(\d+))? @@")

C_FAMILY_SUFFIXES = (".c", ".cc", ".cpp", ".h", ".hpp")


def merge_base(base_ref: str) -> str:
    """Resolves the commit the current branch forked from.

    Args:
        base_ref: Any revision git accepts, such as `origin/main` or a SHA.

    Returns:
        The merge-base commit SHA of `base_ref` and `HEAD`.

    Raises:
        subprocess.CalledProcessError: `base_ref` is unknown or shares no
            history with `HEAD`.

    Verification:
        Correct means: the SHA matches `git merge-base base_ref HEAD`.

    References:
        git-merge-base(1).
    """
    result = subprocess.run(
        ["git", "merge-base", base_ref, "HEAD"], check=True, capture_output=True, text=True
    )
    return result.stdout.strip()


def changed_lines(base_sha: str, suffixes: tuple = C_FAMILY_SUFFIXES) -> dict:
    """Maps each changed file to the working-tree line numbers it adds.

    Walks the hunk headers of a zero-context diff; each `+c,d` header names
    `d` consecutive new-side lines starting at line `c`. A header with `d`
    equal to zero is a pure deletion and contributes no lines. Deleted files
    are skipped because they have no working-tree lines.

    Args:
        base_sha: The commit to diff the working tree against.
        suffixes: File suffixes to include; other files are ignored.

    Returns:
        A dict from repository-relative path to a set of 1-based line numbers.
        Files with only deletions map to an empty set and are omitted.

    Verification:
        Correct means: for a hunk `@@ -5,0 +6,3 @@` the file maps to
        {6, 7, 8}, and for `@@ -5,2 +4,0 @@` nothing is added.

    References:
        git-diff(1), "Generating patch text with -p".
    """
    diff = subprocess.run(
        ["git", "diff", "-U0", "--no-color", "--diff-filter=AMR", base_sha, "--"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout

    result = {}
    path = None
    for line in diff.splitlines():
        if line.startswith("+++ "):
            target = line[4:]
            path = target[2:] if target.startswith("b/") else None
            if path is not None and not path.endswith(suffixes):
                path = None

            continue

        match = _HUNK_HEADER.match(line)
        if path is None or match is None:
            continue

        start = int(match.group(1))
        count = int(match.group(2)) if match.group(2) is not None else 1
        if count:
            result.setdefault(path, set()).update(range(start, start + count))

    return result
