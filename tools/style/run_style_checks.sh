#!/usr/bin/env bash
# Run every AGENTS.md style check, the same way locally and in CI.
#
# Formatting and the blank-line rule apply to lines changed since the merge
# base with BASE_REF (default origin/main), including uncommitted edits, so
# untouched code is held to them once it is edited. The function-length check
# covers the whole tree against tools/style/function_length_baseline.txt.
#
# Setup:  python3 -m pip install -r tools/style/requirements.txt
# Usage:  tools/style/run_style_checks.sh [BASE_REF]
# Fix formatting in place:  git clang-format --extensions c,cc,cpp,h,hpp "$(git merge-base origin/main HEAD)"
set -uo pipefail

base_ref="${1:-origin/main}"
here="$(cd "$(dirname "$0")" && pwd)"
base_sha="$(git merge-base "$base_ref" HEAD)" || exit 2
status=0

echo "== clang-format (lines changed since ${base_sha:0:12})"
if ! git clang-format --diff --extensions c,cc,cpp,h,hpp "$base_sha"; then
    echo "Run: git clang-format --extensions c,cc,cpp,h,hpp $base_sha"
    status=1
fi

echo "== blank line after closing brace"
python3 "$here/check_blank_after_brace.py" --base "$base_sha" || status=1

echo "== function length"
python3 "$here/check_function_length.py" || status=1

exit "$status"
