#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0
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
    flaky case is not always in the results. A line prefixed with `teardown `
    is narrower, for the post-shutdown cases that check every launched
    process's exit code ("Proc <name> exited with code <n>"): its case may
    pass or be absent, and may fail only for one of two reasons. Either
    launch had to stop a process at teardown, with one of the signals it
    sends (SIGINT -2, SIGTERM -15, SIGKILL -9), which is a matter of the
    runner's speed; or a case of the same test and parameter that is a
    listed failure failed too, so the exit code reports that known failure
    again, and whether the process gave up before launch stopped it is a
    matter of timing. A crash or an error exit in a test whose other cases
    pass fails the check. Flaky and teardown lines may
    use `*` to match any run of characters, for a family of cases that share
    one cause, such as every AfterShutdown case; a must-fail line names one
    case exactly, so it still goes stale when that case passes or
    disappears. Every exempted failure is printed with its message.

    The failure list alone cannot notice a test that stops reporting while
    it passes, for example the rmw_rosette<->rmw_cyclonedds_cpp pairs when
    rmw_cyclonedds_cpp is missing from the image. A second committed file per
    distribution (rmw_rosette/test/conformance_tests_<distro>.txt, --tests)
    lists every CTest test that must report results, one name per line, and
    the check fails when one of them reports nothing or a test outside it
    reports results, so the inventory changes only together with the suites.

    A line prefixed with `skip ` holds a regular expression over CTest test
    names, for a whole test that fails for a reason already understood and
    costs minutes of timeouts to confirm. The workflow asks this script for
    the skip lines joined into one alternation (--ctest-exclude) and passes it
    to `ctest -E`, so the test does not run; result files whose name matches
    are ignored, and a must-fail line naming a case of a skipped test is
    reported stale. A skip line uses only the syntax CMake's regular
    expressions and Python's `re` share: `^ $ . * + ? [] () |`, and all skip
    lines together hold at most nine `()` groups, CMake's limit for one
    expression.

    A case is named `<result file stem>::<classname>.<name>`, for example
    `test_client__rmw_rosette::test_rmw_implementation.TestClient.check_qos`.
    Only result files whose name contains `rmw_rosette` are read.

Usage:
    tools/ci/check_rmw_conformance.py --known <known_failures.txt>
        --tests <tests.txt> <test_results dir>...
    Add --write-known to record the current failures and tests as the lists
    instead.
    tools/ci/check_rmw_conformance.py --known <known_failures.txt> --ctest-exclude
    prints the `ctest -E` expression for the list's skip lines.

Verification:
    Correct means: exit status 1 for a new failure, an unexpected pass, a
    stale entry, a teardown case failing for another reason, or a test
    missing from or added to the results, each listed by name; exit status 0
    otherwise, with the pass,
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
import re
import sys
import typing
import xml.etree.ElementTree as ET

FLAKY = "flaky "
TEARDOWN = "teardown "
SKIP = "skip "

# launch_testing.asserts.assertExitCodes, and the signals launch stops a
# process with at teardown (launch.actions.ExecuteLocal: SIGINT, then SIGTERM
# after sigterm_timeout, then SIGKILL after sigkill_timeout).
EXIT_CODE = re.compile(r"Proc \S+ exited with code (-?\d+)")
TEARDOWN_EXIT_CODES = {-2, -15, -9}


class Case(typing.NamedTuple):
    """One test case's outcome: whether it passed, and its failure text."""

    passed: bool
    message: str


class Known(typing.NamedTuple):
    """A known-failure list's lines, by kind."""

    failing: set[str]
    flaky: set[str]
    teardown: set[str]
    skip: list[str]


def collect_cases(result_dirs: list[pathlib.Path], skip: list[str]) -> dict[str, Case]:
    """Reads every rmw_rosette result file under the given directories.

    Args:
        result_dirs: Directories to search recursively for `*.xml` result
            files, usually `build/<package>/test_results`.
        skip: The skip lines of the known list; a result file whose stem
            (its CTest test name) one of them matches is not read.

    Returns:
        A map from case name to its outcome: passed unless it failed or
        errored (a skipped case counts as passed), and for a failure, the
        message and text of its `<failure>` and `<error>` elements.

    Raises:
        xml.etree.ElementTree.ParseError: a result file is not valid XML.

    Verification:
        Correct means: every `<testcase>` of every result file whose name
        contains `rmw_rosette` and matches no skip line appears once, failed
        exactly when it holds a `<failure>` or `<error>` element.
    """
    cases = {}
    for result_dir in result_dirs:
        for path in sorted(result_dir.rglob("*.xml")):
            if "rmw_rosette" not in path.name:
                continue

            stem = path.name.split(".")[0]
            if any(re.search(pattern, stem) for pattern in skip):
                continue

            for case in ET.parse(path).getroot().iter("testcase"):
                name = f"{stem}::{case.get('classname')}.{case.get('name')}"
                faults = case.findall("failure") + case.findall("error")
                message = "\n".join(f"{fault.get('message') or ''}\n{fault.text or ''}"
                                    for fault in faults)
                cases[name] = Case(not faults, message)

    return cases


def read_known(path: pathlib.Path) -> Known:
    """Reads a known-failure list.

    Args:
        path: The list: one case name per line, `#` comments and blank lines
            ignored, `flaky ` marking a case allowed either outcome,
            `teardown ` marking a case allowed to fail only at teardown, and
            `skip ` marking a CTest test-name expression not to run. A
            missing file is an empty list.

    Returns:
        The lines by kind; skip expressions keep their file order.
    """
    known = Known(set(), set(), set(), [])
    if not path.exists():
        return known

    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue

        if line.startswith(FLAKY):
            known.flaky.add(line[len(FLAKY):].strip())
        elif line.startswith(TEARDOWN):
            known.teardown.add(line[len(TEARDOWN):].strip())
        elif line.startswith(SKIP):
            known.skip.append(line[len(SKIP):].strip())
        else:
            known.failing.add(line)

    return known


def read_tests(path: pathlib.Path) -> set[str]:
    """Reads a test inventory: one CTest test name per line, `#` comments ignored.

    A missing file is an empty inventory, so every test that reports is
    listed as unexpected, which is the inventory to commit.
    """
    if not path.exists():
        return set()

    return {line for raw in path.read_text().splitlines()
            if (line := raw.split("#", 1)[0].strip())}


def is_flaky(name: str, flaky: set[str]) -> bool:
    """Tells whether a case is named by one of a list's pattern lines.

    Args:
        name: The case name, `<result file stem>::<classname>.<name>`.
        flaky: The flaky or teardown lines of a known list; `*` in a line
            matches any run of characters, and every other character matches
            itself.

    Returns:
        True when a line names the case or a pattern matches all of it.

    Verification:
        Correct means: `[`, `]`, `.` and the other regex and glob
        metacharacters in a case name match only themselves, so a
        parametrized name such as `...[Arrays]` can be listed exactly.
    """
    if name in flaky:
        return True

    return any("*" in line and re.fullmatch(".*".join(map(re.escape, line.split("*"))), name)
               for line in flaky)


def is_teardown_failure(message: str) -> bool:
    """Tells whether a failure is only launch stopping processes at teardown.

    Args:
        message: The case's failure text, as collect_cases() gathers it.

    Returns:
        True when the text reports at least one process exit code and every
        one is a signal launch sends at teardown (TEARDOWN_EXIT_CODES).

    Verification:
        Correct means: "Proc publisher-1 exited with code -15" is a teardown
        failure; a code of -11 (SIGSEGV), -6 (SIGABRT) or 1 beside it, or a
        failure with no exit code at all, is not.

    References:
        - launch_testing/asserts/assert_exit_codes.py, assertExitCodes.
        - launch/actions/execute_local.py, sigterm_timeout and sigkill_timeout.
    """
    codes = [int(code) for code in EXIT_CODE.findall(message)]
    return bool(codes) and all(code in TEARDOWN_EXIT_CODES for code in codes)


def shares_known_failure(name: str, known_failed: set[str]) -> bool:
    """Tells whether another case of the same test and parameter is a known failure that failed.

    Args:
        name: The case name, `<result file stem>::<classname>.<name>[<parameter>]`.
        known_failed: The listed must-fail cases that failed in these results.

    Returns:
        True when a case in `known_failed` has the same result file stem and
        the same trailing `[<parameter>]` (or neither has one).

    Verification:
        Correct means: `t::c.AfterShutdown.x[Empty]` is covered by a failed
        `t::c.Main.y[Empty]`, and not by `t::c.Main.y[Arrays]` or by a case of
        another test.
    """
    stem, _, case = name.partition("::")
    parameter = case[case.rfind("["):] if case.endswith("]") else ""
    for other in known_failed:
        other_stem, _, other_case = other.partition("::")
        other_parameter = other_case[other_case.rfind("["):] if other_case.endswith("]") else ""
        if other_stem == stem and other_parameter == parameter:
            return True

    return False


def report(title: str, names: set[str]) -> None:
    """Prints a sorted, indented list of case names under a title."""
    print(title)
    for name in sorted(names):
        print(f"  {name}")


def write_lists(args: argparse.Namespace, known: Known, cases: dict[str, Case]) -> None:
    """Records the current results as the known-failure list and the test inventory.

    Skip, flaky and teardown lines are kept as they are; every failing case
    that no flaky or teardown line covers becomes a must-fail line, and every
    test that reported becomes an inventory line.
    """
    failed = {name for name, case in cases.items() if not case.passed}
    exempt = known.flaky | known.teardown
    failing = {name for name in failed if not is_flaky(name, exempt)}
    lines = [f"{SKIP}{pattern}" for pattern in known.skip]
    lines += [f"{FLAKY}{name}" for name in sorted(known.flaky)]
    lines += [f"{TEARDOWN}{name}" for name in sorted(known.teardown)]
    lines += sorted(failing)
    args.known.write_text("".join(f"{line}\n" for line in lines))
    tests = sorted({name.split("::", 1)[0] for name in cases})
    args.tests.write_text("".join(f"{test}\n" for test in tests))
    print(f"wrote {len(failing)} failures, {len(known.flaky)} flaky and {len(known.teardown)} "
          f"teardown lines and {len(known.skip)} skipped tests to {args.known}, "
          f"and {len(tests)} tests to {args.tests}")


def compare(args: argparse.Namespace, known: Known, cases: dict[str, Case]) -> int:
    """Holds the results to the known-failure list and the test inventory.

    Returns:
        0 when they match, 1 otherwise, with each mismatch listed by name.
    """
    failed = {name for name, case in cases.items() if not case.passed}
    print(f"{len(cases) - len(failed)}/{len(cases)} rmw_rosette cases pass; "
          f"{len(known.failing)} known failures, {len(known.flaky)} flaky and "
          f"{len(known.teardown)} teardown lines, {len(known.skip)} skip expressions")

    unlisted = {name for name in failed - known.failing if not is_flaky(name, known.flaky)}
    known_failed = known.failing & failed
    teardown = {name for name in unlisted
                if is_flaky(name, known.teardown)
                and (is_teardown_failure(cases[name].message)
                     or shares_known_failure(name, known_failed))}
    exempt = (failed - known.failing - unlisted) | teardown
    new = unlisted - teardown
    fixed = {name for name in known.failing if name in cases and cases[name].passed}
    stale = known.failing - cases.keys()
    reported = {name.split("::", 1)[0] for name in cases}
    expected = read_tests(args.tests)
    for name in sorted(exempt):
        print(f"allowed failure: {name}")
        for line in cases[name].message.strip().splitlines()[:3]:
            print(f"    {line}")

    if new:
        report("new failures (fix them, or list them with a reason):", new)
        for name in sorted(n for n in new if is_flaky(n, known.teardown)):
            print(f"  {name} failed with no teardown signal and no known failure beside it:")
            for line in cases[name].message.strip().splitlines()[:5]:
                print(f"    {line}")

    if fixed:
        report(f"passing now (remove them from {args.known}):", fixed)

    if stale:
        report(f"not in the results (remove them from {args.known}):", stale)

    if expected - reported:
        report(f"tests that reported no results (listed in {args.tests}):", expected - reported)

    if reported - expected:
        report(f"tests not in the inventory (add them to {args.tests}):", reported - expected)

    mismatch = new or fixed or stale or expected != reported
    return 1 if mismatch else 0


def main() -> int:
    """Runs the comparison and returns the exit status.

    Returns:
        0 when the results match the lists or the lists were written, 1
        otherwise.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--known", type=pathlib.Path, required=True)
    parser.add_argument("--tests", type=pathlib.Path,
                        help="the inventory of tests that must report results")
    parser.add_argument("--write-known", action="store_true")
    parser.add_argument("--ctest-exclude", action="store_true",
                        help="print the ctest -E expression for the skip lines and exit")
    parser.add_argument("result_dirs", type=pathlib.Path, nargs="*")
    args = parser.parse_args()

    known = read_known(args.known)
    if args.ctest_exclude:
        # Joined bare: `|` binds loosest, and CMake's regular expressions allow
        # only nine groups in one expression.
        print("|".join(known.skip))
        return 0

    if not args.result_dirs or args.tests is None:
        parser.error("--tests and result_dirs are required unless --ctest-exclude is given")

    cases = collect_cases(args.result_dirs, known.skip)
    if not cases:
        print("no rmw_rosette results found; did the tests run?")
        return 1

    if args.write_known:
        write_lists(args, known, cases)
        return 0

    return compare(args, known, cases)


if __name__ == "__main__":
    sys.exit(main())
