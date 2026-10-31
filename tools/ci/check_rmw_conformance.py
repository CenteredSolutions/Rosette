#!/usr/bin/env python3
"""Compare rmw_rosette's ROS 2 conformance results with the recorded known failures.

Architecture:
    Two upstream suites check rmw_rosette the way they check every stock rmw:
    test_rmw_implementation (ros2/rmw_implementation) calls the rmw API
    directly and checks each entry point's results and error codes, and
    test_communication (ros2/system_tests) runs publishers, subscriptions,
    services and actions between two processes, one per rmw, so rmw_rosette
    talks to itself, to rmw_fastrtps_cpp and to rmw_cyclonedds_cpp. Both
    register one CTest test per rmw (or rmw pair) through ament, whose
    run_test.py writes a JUnit-style result file for each, gtest's XML or
    launch_testing's xunit, and a failing placeholder when the test crashes
    or times out before writing one.

    rmw_rosette does not pass every case yet. The known failures live in a
    committed list per distribution (rmw_rosette/test/conformance_known_failures_<distro>.txt),
    and this check holds the results to it exactly:

    - a failing case not on the list fails the check (a regression);
    - a listed case that passes fails the check, so a fix removes its line
      and the list only shrinks;
    - a listed case absent from the results fails the check, so a renamed or
      removed upstream test does not leave a stale line behind.

    A line prefixed with `flaky ` is allowed to pass, fail or be absent, for a
    case whose outcome depends on timing rather than on rmw_rosette's
    behaviour; a test process that aborts part-way reports fewer cases, so a
    flaky case is not always in the results.

    A case is named `<result file stem>::<classname>.<name>`, for example
    `test_client__rmw_rosette::test_rmw_implementation.TestClient.check_qos`.
    Only result files whose name contains `rmw_rosette` are read.

Usage:
    tools/ci/check_rmw_conformance.py --known <known_failures.txt>
        <test_results dir>...
    Add --write-known to record the current failures as the list instead.

Verification:
    Correct means: exit status 1 for a new failure, an unexpected pass or a
    stale entry, each listed by name; exit status 0 otherwise, with the pass,
    failure and known counts printed. Checked by the rmw-rosette workflow's
    conformance job on Humble and Jazzy; deleting a line from a known list,
    or adding a passing case to it, makes the job fail.

References:
    - ros2/ament_cmake_ros, ament_add_ros_isolated_gtest / run_test.py: the
      result files and the missing-result placeholder.
    - ros2/rmw_implementation, test_rmw_implementation/CMakeLists.txt.
    - ros2/system_tests, test_communication/CMakeLists.txt.
"""

import argparse
import pathlib
import sys
import xml.etree.ElementTree as ET

FLAKY = "flaky "


def collect_cases(result_dirs: list[pathlib.Path]) -> dict[str, bool]:
    """Reads every rmw_rosette result file under the given directories.

    Args:
        result_dirs: Directories to search recursively for `*.xml` result
            files, usually `build/<package>/test_results`.

    Returns:
        A map from case name to True when the case passed, False when it
        failed or errored. A skipped case counts as passed.

    Raises:
        xml.etree.ElementTree.ParseError: a result file is not valid XML.

    Verification:
        Correct means: every `<testcase>` of every result file whose name
        contains `rmw_rosette` appears once, failed exactly when it holds a
        `<failure>` or `<error>` element.
    """
    cases = {}
    for result_dir in result_dirs:
        for path in sorted(result_dir.rglob("*.xml")):
            if "rmw_rosette" not in path.name:
                continue

            stem = path.name.split(".")[0]
            for case in ET.parse(path).getroot().iter("testcase"):
                name = f"{stem}::{case.get('classname')}.{case.get('name')}"
                failed = case.find("failure") is not None or case.find("error") is not None
                cases[name] = not failed

    return cases


def read_known(path: pathlib.Path) -> tuple[set[str], set[str]]:
    """Reads a known-failure list.

    Args:
        path: The list: one case name per line, `#` comments and blank lines
            ignored, `flaky ` marking a case allowed either outcome.

    Returns:
        The names that must fail, and the names allowed either outcome.
    """
    failing, flaky = set(), set()
    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue

        if line.startswith(FLAKY):
            flaky.add(line[len(FLAKY):].strip())
        else:
            failing.add(line)

    return failing, flaky


def report(title: str, names: set[str]) -> None:
    """Prints a sorted, indented list of case names under a title."""
    print(title)
    for name in sorted(names):
        print(f"  {name}")


def main() -> int:
    """Runs the comparison and returns the exit status.

    Returns:
        0 when the results match the known list or a list was written, 1
        otherwise.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--known", type=pathlib.Path, required=True)
    parser.add_argument("--write-known", action="store_true")
    parser.add_argument("result_dirs", type=pathlib.Path, nargs="+")
    args = parser.parse_args()

    cases = collect_cases(args.result_dirs)
    if not cases:
        print("no rmw_rosette results found; did the tests run?")
        return 1

    failed = {name for name, passed in cases.items() if not passed}
    if args.write_known:
        _, flaky = read_known(args.known) if args.known.exists() else (set(), set())
        lines = [f"{FLAKY}{name}" for name in sorted(flaky)]
        lines += sorted(failed - flaky)
        args.known.write_text("".join(f"{line}\n" for line in lines))
        print(f"wrote {len(failed - flaky)} failures and {len(flaky)} flaky cases to {args.known}")
        return 0

    known, flaky = read_known(args.known)
    print(f"{len(cases) - len(failed)}/{len(cases)} rmw_rosette cases pass; "
          f"{len(known)} known failures, {len(flaky)} flaky")

    new = failed - known - flaky
    fixed = {name for name in known if cases.get(name)}
    stale = known - cases.keys()
    if new:
        report("new failures (fix them, or list them with a reason):", new)

    if fixed:
        report(f"passing now (remove them from {args.known}):", fixed)

    if stale:
        report(f"not in the results (remove them from {args.known}):", stale)

    return 1 if new or fixed or stale else 0


if __name__ == "__main__":
    sys.exit(main())
