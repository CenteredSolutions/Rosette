#!/usr/bin/env bash
# Runs every libFuzzer target of a build for a fixed time each.
#
#   tools/fuzz/run_fuzz_campaign.sh <build-dir> [seconds-per-target]
#
# <build-dir> is a tests/spec build configured with -DROSETTE_SPEC_ENABLE_FUZZ=ON,
# or a tests/host build configured with -DROSETTE_HOST_ENABLE_FUZZ=ON, and a Clang
# compiler. Each target starts from the seed corpus the build
# generated plus whatever an earlier campaign left in <build-dir>/fuzz/work/<name>,
# and writes the inputs it finds interesting back there. A crash, sanitizer
# report, or timeout leaves its reproducer in <build-dir>/fuzz/crashes and
# fails the run.
#
# Reproduce a finding with:  <build-dir>/<name>_fuzz <build-dir>/fuzz/crashes/<file>

set -euo pipefail

build_dir=${1:?usage: run_fuzz_campaign.sh <build-dir> [seconds-per-target]}
seconds=${2:-60}

targets=()
for binary in "$build_dir"/*_fuzz; do
    targets+=("$(basename "$binary" _fuzz)")
done

if [[ ${#targets[@]} -eq 0 ]]; then
    echo "error: no *_fuzz targets in $build_dir; configure with Clang and fuzzing enabled" >&2
    exit 2
fi

crash_dir="$build_dir/fuzz/crashes"
mkdir -p "$crash_dir"

failed=0
for name in "${targets[@]}"; do
    binary="$build_dir/${name}_fuzz"
    if [[ ! -x "$binary" ]]; then
        echo "error: $binary not built; configure with Clang and -DROSETTE_SPEC_ENABLE_FUZZ=ON" >&2
        exit 2
    fi

    work="$build_dir/fuzz/work/$name"
    mkdir -p "$work"

    echo "== $name (${seconds}s)"
    if ! ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=print_stacktrace=1 "$binary" \
            -max_total_time="$seconds" -timeout=10 -rss_limit_mb=2048 \
            -artifact_prefix="$crash_dir/$name-" \
            "$work" "$build_dir/fuzz/corpus/$name"; then
        echo "FAILED: $name; reproducer(s) in $crash_dir" >&2
        failed=1
    fi
done

exit "$failed"
