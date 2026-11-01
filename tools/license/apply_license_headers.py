#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
"""Give every tracked source file Rosette's SPDX copyright and license header.

Rosette follows the REUSE specification: each file states its copyright holder
and license in two SPDX tags, either in a comment at the top of the file or,
for formats that cannot carry a comment (JSON, Markdown, .srv, package.xml,
plain text), in REUSE.toml at the repository root. `reuse lint`, run by
tools/style/run_style_checks.sh, rejects any file that has neither.

This tool writes the in-file form. It is idempotent: a file that already
carries an SPDX license tag is left alone, so it can be re-run after a
merge or on a branch that adds files. A file whose header names another
copyright holder (the Pico SDK board headers, BSD-3-Clause) keeps that holder
and its license; Centered Solutions, LLC is added beside it as the holder of
the modifications.

Usage:
    tools/license/apply_license_headers.py            # rewrite files in place
    tools/license/apply_license_headers.py --check    # list files it would change

Verification:
    Correct means `reuse lint` passes on a clean checkout, every rewritten
    file still builds and keeps its shebang on line 1, and a second run
    changes nothing.

References:
    REUSE Specification 3.3, https://reuse.software/spec-3.3/
    SPDX Specification 2.3, Annex D (license identifiers in source files)
    Apache License 2.0, Appendix "How to apply the Apache License to your work"
"""

import argparse
import os
import re
import subprocess
import sys

HOLDER = "Centered Solutions, LLC"
YEAR = "2026"
LICENSE = "Apache-2.0"
COPYRIGHT_TAG = "SPDX-FileCopyright" + "Text"  # split so this file's own header is the only match
LICENSE_TAG = "SPDX-License" + "-Identifier"

# Files carrying `//` comments, and files carrying `#` comments, by suffix or name.
SLASH_SUFFIXES = (".c", ".h", ".cpp", ".hpp", ".h.in")
HASH_SUFFIXES = (".py", ".sh", ".cmake", ".yml", ".yaml", ".mk", ".Dockerfile", ".sh.in", ".cmake.in")
HASH_NAMES = ("CMakeLists.txt", ".clang-format", ".clangd", ".gitignore")


def comment_prefix(path):
    """Return the line-comment prefix for a file, or None when REUSE.toml covers it.

    Args:
        path: Repository-relative path of a tracked file.

    Returns:
        "//" for C and C++ sources, "#" for Python, shell, CMake, YAML, Make and
        tool configuration, and None for every other file.
    """
    name = os.path.basename(path)
    if name.endswith(SLASH_SUFFIXES):
        return "//"

    if name in HASH_NAMES or name.endswith(HASH_SUFFIXES):
        return "#"

    return None


def add_beside_third_party(text):
    """Add Rosette's copyright line under a third-party copyright line.

    Args:
        text: Contents of a file whose header names another copyright holder.

    Returns:
        The contents with one SPDX copyright line for Rosette inserted after the
        first `Copyright (c)` line, in that line's comment style, or None when
        no such line exists.
    """
    match = re.search(r"^(\s*\*?\s*)Copyright \(c\).*$", text, re.MULTILINE)
    if match is None:
        return None

    line = f"{match.group(1)}{COPYRIGHT_TAG}: {YEAR} {HOLDER}"
    return text[: match.end()] + "\n" + line + text[match.end() :]


def with_header(path, text):
    """Return a file's contents with Rosette's header, or None when no change applies.

    The header is two comment lines and a blank line. It goes on line 1, or
    after a `#!` line and a Python coding line, which must stay first.

    Args:
        path: Repository-relative path, which selects the comment style.
        text: Current contents of the file.

    Returns:
        The rewritten contents, or None when the file already names its license,
        takes no comment, or is a third-party file without a recognizable
        copyright line.
    """
    if LICENSE_TAG in text:
        if re.search(r"^\s*\*?\s*Copyright \(c\)", text, re.MULTILINE) and HOLDER not in text:
            return add_beside_third_party(text)

        return None

    prefix = comment_prefix(path)
    if prefix is None:
        return None

    header = f"{prefix} {COPYRIGHT_TAG}: {YEAR} {HOLDER}\n{prefix} {LICENSE_TAG}: {LICENSE}\n"
    lines = text.splitlines(keepends=True)
    keep = 0
    if lines and lines[0].startswith("#!"):
        keep = 1
        if len(lines) > 1 and re.match(r"#.*coding[:=]", lines[1]):
            keep = 2

    rest = "".join(lines[keep:])
    separator = "" if keep else "\n"
    return "".join(lines[:keep]) + header + separator + rest


def main():
    """Rewrite or list tracked files that lack Rosette's license header.

    Returns:
        0 when every file already has a header or was rewritten; 1 under
        --check when at least one file would change.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="list files that need a header; change nothing")
    args = parser.parse_args()

    root = subprocess.check_output(["git", "rev-parse", "--show-toplevel"], text=True).strip()
    files = subprocess.check_output(["git", "ls-files", "-z"], cwd=root, text=True).split("\0")
    changed = []
    for path in filter(None, files):
        full = os.path.join(root, path)
        if os.path.islink(full) or not os.path.isfile(full):
            continue

        with open(full, encoding="utf-8") as handle:
            text = handle.read()

        updated = with_header(path, text)
        if updated is None:
            continue

        changed.append(path)
        if not args.check:
            with open(full, "w", encoding="utf-8") as handle:
                handle.write(updated)

    for path in changed:
        print(path)

    return 1 if (args.check and changed) else 0


if __name__ == "__main__":
    sys.exit(main())
