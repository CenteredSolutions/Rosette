# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

"""Decide whether a CI run must build and test, or only touches documentation.

The build workflows (host-tests, port-stub, rmw-rosette) start with a `changes`
job that runs this script. A pull request whose every changed path is
documentation skips the build jobs; GitHub reports a job skipped by its `if:`
as successful, so each workflow's `result` job, the check branch protection
requires, still passes. A path filter on the workflow itself would instead
leave the required check pending and block the merge.

The script errs toward building: any event other than a pull request, any
path outside the documentation set, and any failure to list the changes all
print `code=true`.
"""

import subprocess
import sys

# Paths that no build, test, or tool reads. Markdown is checked by the style
# workflow (reuse lint), which always runs.
DOC_SUFFIXES = (".md",)
DOC_PREFIXES = ("docs/", "LICENSES/", ".github/ISSUE_TEMPLATE/")
DOC_FILES = ("NOTICE", ".git-blame-ignore-revs")


def is_documentation(path):
    """Return whether a repository path is documentation only.

    Args:
        path: A path relative to the repository root, with `/` separators, as
            `git diff --name-only` prints it.

    Returns:
        True when the path is Markdown, under docs/, LICENSES/ or the issue
        templates, or one of DOC_FILES; False for every other path.

    Verification:
        Correct means no path that a CMake build, a test, or a CI tool reads is
        classified as documentation. `git grep -E '\\.md\\b' tests tools cmake`
        finds no reader of a Markdown file, only cross-references in comments.
    """
    return (
        path.endswith(DOC_SUFFIXES)
        or path.startswith(DOC_PREFIXES)
        or path in DOC_FILES
    )


def changed_paths():
    """List the paths a pull request changes.

    `actions/checkout` checks a pull request out as GitHub's merge commit, whose
    first parent is the tip of the base branch, so the diff between the two is
    exactly the pull request's change. The checkout needs `fetch-depth: 2`.

    Returns:
        The changed paths, or None when git cannot produce the list.

    References:
        GitHub Docs, "Events that trigger workflows", pull_request: GITHUB_SHA
        is the last merge commit of the pull request merge branch.
    """
    result = subprocess.run(
        ["git", "diff", "--name-only", "HEAD^1", "HEAD"],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr)
        return None

    return [line for line in result.stdout.splitlines() if line]


def main(argv):
    """Print `code=true` or `code=false` for $GITHUB_OUTPUT.

    Args:
        argv: The command line; argv[1] is the triggering event name
            (`github.event_name`).

    Returns:
        0 always, so a classification problem never fails the run; it selects
        the full build instead.

    Verification:
        Correct means `code=false` is printed only for a pull_request event
        whose changed paths are all documentation. Checked by running the
        script on a docs-only change, a source change, and a push event.

    References:
        GitHub Docs, "Using conditions to control job execution": a job
        skipped by its condition reports its status as Success.
    """
    event = argv[1] if len(argv) > 1 else ""
    paths = changed_paths() if event == "pull_request" else None
    docs_only = bool(paths) and all(is_documentation(p) for p in paths)

    for path in paths or []:
        print(("doc  " if is_documentation(path) else "code ") + path, file=sys.stderr)

    print(f"code={'false' if docs_only else 'true'}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
