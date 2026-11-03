#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
"""Check the rmw_* functions librmw_rosette exports against the ROS 2 loader.

Architecture:
    An rclcpp or rclpy program does not link an rmw implementation directly.
    It links librmw_implementation, which loads the library that
    RMW_IMPLEMENTATION names and resolves every rmw_* function it forwards with
    dlsym. A function missing from librmw_rosette is a runtime failure, often
    only on the first call, so the build alone does not prove the rmw is
    complete. This check makes that a CI failure, per ROS 2 distribution.

    It makes two comparisons of the dynamic symbol table:

    - `loader`: every rmw_* function librmw_implementation exports must also
      be exported by librmw_rosette. The loader's own export list is the
      authoritative set of entry points for the distribution it ships with.
    - `baseline`: the full rmw_* export list must equal the committed baseline
      for the distribution (rmw_rosette/test/exported_symbols_<distro>.txt).
      An export appearing or disappearing is then a reviewed change to the
      baseline, never a side effect, which keeps one distribution's ABI
      unchanged while another distribution's support is extended.

Usage:
    tools/ci/check_rmw_rosette_exports.py --library <librmw_rosette.so>
        --loader <librmw_implementation.so> --baseline <exported_symbols.txt>
    Add --write-baseline to record the library's current exports instead.

Verification:
    Correct means: exit status 1 when the loader exports an rmw_* function the
    library does not, or when the library's rmw_* exports differ from the
    baseline in either direction; exit status 0 otherwise, with the counts
    printed. Checked by the rmw-rosette workflow on Humble and Jazzy; deleting
    a line from a baseline, or an rmw_* definition from rmw_rosette/src, makes
    the job fail.

References:
    - ros2/rmw_implementation, src/functions.cpp: the forwarding functions and
      their dlsym lookup by name.
    - ROS 2 design, "ROS 2 middleware interface" (design.ros2.org/articles/ros_middleware_interface.html).
"""

import argparse
import pathlib
import subprocess
import sys


def exported_rmw_functions(library: pathlib.Path) -> set[str]:
    """Lists the rmw_* symbols a shared library defines in its dynamic table.

    Args:
        library: Path to the shared object to read.

    Returns:
        The names of every defined dynamic symbol that starts with `rmw_`.

    Raises:
        subprocess.CalledProcessError: nm cannot read the library.

    Verification:
        Correct means: the set equals `nm -D --defined-only` filtered to names
        starting with rmw_; undefined (imported) symbols are excluded.
    """
    listing = subprocess.run(
        ["nm", "-D", "--defined-only", str(library)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    names = set()
    for line in listing.splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[2].startswith("rmw_"):
            names.add(fields[2])

    return names


def report(title: str, names: set[str]) -> None:
    """Prints a sorted, indented list of symbol names under a title."""
    print(title)
    for name in sorted(names):
        print(f"  {name}")


def main() -> int:
    """Runs the loader and baseline comparisons and returns the exit status.

    Returns:
        0 when both comparisons pass or a baseline was written, 1 otherwise.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--library", type=pathlib.Path, required=True)
    parser.add_argument("--loader", type=pathlib.Path, required=True)
    parser.add_argument("--baseline", type=pathlib.Path, required=True)
    parser.add_argument("--write-baseline", action="store_true")
    args = parser.parse_args()

    have = exported_rmw_functions(args.library)
    if args.write_baseline:
        args.baseline.write_text("".join(f"{name}\n" for name in sorted(have)))
        print(f"wrote {len(have)} symbols to {args.baseline}")
        return 0

    want = exported_rmw_functions(args.loader)
    baseline = set(args.baseline.read_text().split())
    print(f"loader forwards {len(want)} rmw_* functions; "
          f"librmw_rosette exports {len(have)}; baseline lists {len(baseline)}")

    failed = False
    if want - have:
        report("loader: functions the loader forwards that librmw_rosette does not export:",
               want - have)
        failed = True

    if have != baseline:
        report("baseline: exported but not in the baseline:", have - baseline)
        report("baseline: in the baseline but not exported:", baseline - have)
        print(f"If the change is intended, rerun with --write-baseline and commit {args.baseline}.")
        failed = True

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
