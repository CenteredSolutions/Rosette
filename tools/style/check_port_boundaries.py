#!/usr/bin/env python3
"""Check that chip, OS, and network-stack code stays inside its port.

Architecture:
    Rosette runs on any combination of one chip port (rp2, host, ...), one OS
    port (baremetal, freertos, posix, ...), and one network port (a WIZnet
    chip's hardware sockets, FreeRTOS+TCP, lwIP, POSIX sockets, ...). A board under
    boards/ names one of each. The code that touches a vendor SDK, an RTOS, or
    a host OS lives under ports/<axis>/<name>/; everything else in the tree is
    portable and reaches the port only through the contracts in
    include/rosette/platform and include/rosette/net
    (docs/design/ports_move_plan.md, "Rules").

    This check reads every C and C++ file in the tree and reports three kinds
    of violation:

    - `header`: the file includes a third-party header from a family its zone
      does not allow, for example `hardware/spi.h` (Pico SDK) outside
      ports/chip/rp2, or `FreeRTOS.h` inside ports/chip/rp2. The WIZnet
      ioLibrary is allowed nowhere.
    - `port-include`: the file includes a header that lives in a port it does
      not belong to. Portable code includes no port header; a port includes
      only its own headers and portable ones.
    - `port-macro`: the file selects code with `#if`/`#ifdef` on a port
      selector macro (ROSETTE_OS_*, ROSETTE_CHIP_*, PICO_*, __linux__, ...).
      The build picks the port by linking it; portable code does not branch
      on which one it got.

    Violations that already exist are listed in port_boundary_baseline.txt.
    A listed violation is tolerated, and any violation not listed fails the
    check, so the tree can only get cleaner. A listed violation that no longer
    occurs is reported as stale so it can be deleted.

Usage:
    tools/style/check_port_boundaries.py                  check against the baseline
    tools/style/check_port_boundaries.py --all            list every violation
    tools/style/check_port_boundaries.py --write-baseline record the current violations

Verification:
    Correct means: exit status 1 when a file has a violation that is not in
    the baseline, and 0 otherwise; stale baseline entries are reported but do
    not fail the check. Checked by running it on the tree (passes against the
    committed baseline), and after adding `#include "hardware/gpio.h"` to
    src/rtps/participant.cpp (fails, naming that file and family).

References:
    docs/design/ports_move_plan.md, "Rules" (the zone table mirrored below).
    AGENTS.md, "Architecture Direction".
    FreeRTOS kernel, portable/<compiler>/<arch>/portmacro.h: the fixed-name
    per-port header pattern this layout follows.
"""

import argparse
import pathlib
import re
import subprocess
import sys

_ROOT = pathlib.Path(__file__).resolve().parents[2]
_BASELINE = pathlib.Path(__file__).with_name("port_boundary_baseline.txt")
_SOURCE_SUFFIXES = (".c", ".cc", ".cpp", ".h", ".hpp")

# Third-party header families, matched against the text between the include
# delimiters. A header matching no family is portable (the C and C++ standard
# libraries, littlefs, ROS 2, MicroPython, and Rosette's own headers).
_FAMILIES = {
    "pico_sdk": re.compile(r"^(pico|hardware|boards)/|^pico\.h$"),
    "freertos_kernel": re.compile(
        r"^(FreeRTOS|task|semphr|queue|timers|event_groups|portmacro"
        r"|stream_buffer|message_buffer)\.h$"),
    "freertos_tcp": re.compile(
        r"^FreeRTOS_\w+\.h$"
        r"|^(NetworkInterface|NetworkBufferManagement|pack_struct_start|pack_struct_end)\.h$"),
    "lwip": re.compile(r"^(lwip|netif|arch)/|^lwipopts\.h$"),
    "wiznet_iolib": re.compile(r"^(wizchip_conf|w5100s?|w5300|w5500|w6100|w6300|socket|dhcp|dns)\.h$"),
    "posix": re.compile(
        r"^(sys|arpa|netinet|net)/"
        r"|^(unistd|pthread|fcntl|poll|netdb|ifaddrs|dlfcn|signal|sched|termios|syslog)\.h$"),
    "board": re.compile(r"^(board_config|board)\.h$"),
}
_ALL = frozenset(_FAMILIES)

# Families no zone may include, boards and tests among them. Rosette drives
# WIZnet chips through its own register-level driver (rosette/net/wiznet/),
# so the WIZnet ioLibrary is not a dependency.
_BANNED = frozenset({"wiznet_iolib"})

# Zones, first match wins: a path prefix, the header families files there may
# include, and the port the prefix names (None for non-port zones). Every
# path that matches no zone is portable and may include no family.
_ZONES = (
    ("ports/chip/rp2/", {"pico_sdk", "board"}, "ports/chip/rp2/"),
    ("ports/chip/host/", {"posix"}, "ports/chip/host/"),
    ("ports/chip/stub/", set(), "ports/chip/stub/"),
    ("ports/os/baremetal/", set(), "ports/os/baremetal/"),
    ("ports/os/freertos/", {"freertos_kernel"}, "ports/os/freertos/"),
    ("ports/os/posix/", {"posix"}, "ports/os/posix/"),
    ("ports/os/stub/", set(), "ports/os/stub/"),
    ("ports/net/freertos_tcp/", {"freertos_kernel", "freertos_tcp"}, "ports/net/freertos_tcp/"),
    ("ports/net/lwip/", {"lwip"}, "ports/net/lwip/"),
    ("ports/net/posix/", {"posix"}, "ports/net/posix/"),
    ("ports/net/wiznet/", set(), "ports/net/wiznet/"),
    ("boards/", _ALL, None),
    ("tests/", _ALL, None),
    ("rmw_rosette/test/", _ALL, None),
    ("examples/", _ALL, None),
    ("tools/", _ALL, None),
)

# Macros that name a chip, OS, or network stack. Portable code that branches
# on one of these carries a port inside it.
_PORT_MACRO = re.compile(
    r"\b(ROSETTE_OS_\w+|ROSETTE_CHIP_\w+|ROSETTE_FREERTOS_TCP|PICO_\w+|LIB_PICO_\w+"
    r"|__linux__|__APPLE__|_WIN32|ESP_PLATFORM|__ZEPHYR__)\b")
_INCLUDE = re.compile(r'^\s*#\s*include\s*([<"])([^>"]+)[>"]')
_CONDITIONAL = re.compile(r"^\s*#\s*(if|ifdef|ifndef|elif)\b(.*)")

# Directories the build puts on the include path, in search order after the
# including file's own directory (CMakeLists.txt, cmake/*.cmake). Each port's
# own directory follows them: its port.cmake puts it on the include path.
_INCLUDE_ROOTS = (
    "include",
    "include/rosette/net",
    "include/rosette/rtps",
    "rmw_rosette/include",
    "ports",
)

# Headers every port of an axis supplies under one name, found through
# whichever port the build selected: including one is the contract's seam,
# not a reach into a particular port (platform.hpp -> rosette_os_port.hpp).
_FIXED_NAME_HEADERS = frozenset({"rosette_os_port.hpp"})
_EXCLUDED_PREFIXES = ("third_party/", "external/", "build")


def list_sources() -> list:
    """Lists the C and C++ files the check covers.

    Returns:
        Repository-relative POSIX paths of tracked and untracked (but not
        ignored) sources that exist on disk, excluding vendored code and build
        trees.

    Verification:
        Correct means: src/rtps/participant.cpp is listed, a file under
        third_party/ or build*/ is not, and a new untracked source is.
    """
    listing = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard"],
        cwd=_ROOT, check=True, capture_output=True, text=True).stdout
    return sorted(
        path for path in listing.splitlines()
        if path.endswith(_SOURCE_SUFFIXES)
        and not path.startswith(_EXCLUDED_PREFIXES)
        and (_ROOT / path).is_file())


def zone_of(path: str) -> tuple:
    """Finds the zone a file belongs to.

    Args:
        path: Repository-relative path of the file.

    Returns:
        `(allowed_families, own_port)`. `allowed_families` is the set of
        header families the file may include; `own_port` is the port prefix
        (for example "ports/chip/rp2/") for a file inside a port, or None.
        A path in no listed zone is portable: `(set(), None)`, except that a
        port directory missing from the table still owns its own files.

    Verification:
        Correct means: "ports/chip/rp2/x.cpp" gives ({"pico_sdk", "board"},
        "ports/chip/rp2/"), "boards/b/board_init.c" gives (all families,
        None), and "src/rtps/spdp.cpp" gives (set(), None).
    """
    for prefix, allowed, port in _ZONES:
        if path.startswith(prefix):
            return set(allowed), port

    return set(), port_of(path)


def port_of(path: str):
    """Names the port a file lives in.

    Args:
        path: Repository-relative path.

    Returns:
        "ports/<axis>/<name>/" when the path is inside one, otherwise None.

    Verification:
        Correct means: "ports/os/freertos/rosette_os_port.hpp" gives
        "ports/os/freertos/", and "include/rosette/net/socket.hpp" gives None.
    """
    parts = path.split("/")
    if len(parts) > 3 and parts[0] == "ports":
        return "/".join(parts[:3]) + "/"

    return None


def resolve_include(including: str, target: str, quoted: bool, sources: set):
    """Finds the repository file an include names, searching as the build does.

    Args:
        including: Repository-relative path of the file with the include.
        target: The text between the include's delimiters.
        quoted: True for `"..."`, which searches the including file's own
            directory first; False for `<...>`.
        sources: Every repository source path, for membership tests.

    Returns:
        The repository-relative path of the included file, or None for a
        header outside the repository (a toolchain, SDK, or vendored header).

    Verification:
        Correct means: `"socket.hpp"` from include/rosette/net/inet.h resolves
        to include/rosette/net/socket.hpp, `"wiznet/device.hpp"` resolves
        through the include/rosette/net root, `"rp2_spi_bus.hpp"` resolves
        to ports/chip/rp2/rp2_spi_bus.hpp, and `<lfs.h>` resolves to None.
    """
    directories = [str(pathlib.PurePosixPath(including).parent)] if quoted else []
    directories.extend(_INCLUDE_ROOTS)
    directories.extend(sorted({port_of(source).rstrip("/") for source in sources
                               if port_of(source) is not None}))
    for directory in directories:
        candidate = pathlib.PurePosixPath(directory, target).as_posix()
        normalized = pathlib.Path(candidate).as_posix()
        parts = []
        for part in normalized.split("/"):
            if part == "..":
                if parts:
                    parts.pop()

            elif part not in ("", "."):
                parts.append(part)

        resolved = "/".join(parts)
        if resolved in sources:
            return resolved

    return None


def scan_file(path: str, sources: set) -> dict:
    """Finds every port-boundary violation in one file.

    Each include and each preprocessor conditional is tested against the
    file's zone: a third-party header must be in an allowed family, a
    repository header must not live in another port, and a conditional in
    portable code must not test a port selector macro. Violations are keyed
    without line numbers, so the baseline survives edits that move lines.

    Args:
        path: Repository-relative path of the file to scan.
        sources: Every repository source path, for include resolution.

    Returns:
        A dict from `(path, kind, detail)` to the first line number where it
        occurs. `kind` is "header", "port-include", or "port-macro";
        `detail` is `family:header`, the included path, or the macro name.

    Verification:
        Correct means: hardware/spi.h included from src/rtps/participant.cpp
        yields ("header", "pico_sdk:hardware/spi.h"); "rp2_spi_bus.hpp"
        included from include/rosette/net/wiznet/device.hpp yields
        ("port-include", "ports/chip/rp2/rp2_spi_bus.hpp"); platform.hpp's
        "rosette_os_port.hpp" yields nothing; a boards/ file yields nothing.
    """
    allowed, own_port = zone_of(path)
    exempt = allowed == set(_ALL) and own_port is None
    found = {}
    text = (_ROOT / path).read_text(encoding="utf-8", errors="replace")
    for number, line in enumerate(text.splitlines(), start=1):
        include = _INCLUDE.match(line)
        if include is not None:
            quoted, target = include.group(1) == '"', include.group(2).strip()
            for family, pattern in _FAMILIES.items():
                if pattern.search(target) and (family not in allowed or family in _BANNED):
                    found.setdefault((path, "header", f"{family}:{target}"), number)

            if pathlib.PurePosixPath(target).name in _FIXED_NAME_HEADERS:
                continue

            resolved = resolve_include(path, target, quoted, sources)
            target_port = port_of(resolved) if resolved else None
            if not exempt and target_port is not None and target_port != own_port:
                found.setdefault((path, "port-include", resolved), number)

            continue

        conditional = _CONDITIONAL.match(line)
        if conditional is not None and own_port is None and not exempt:
            for macro in _PORT_MACRO.findall(conditional.group(2)):
                found.setdefault((path, "port-macro", macro), number)

    return found


def load_baseline() -> set:
    """Reads the recorded violations the tree still carries.

    Returns:
        A set of `(path, kind, detail)` tuples. Blank lines and `#` comments
        in the baseline file are ignored; a missing file is an empty baseline.

    Verification:
        Correct means: the line `src/a.cpp header posix:unistd.h` gives
        ("src/a.cpp", "header", "posix:unistd.h").
    """
    if not _BASELINE.exists():
        return set()

    baseline = set()
    for line in _BASELINE.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            path, kind, detail = line.split(maxsplit=2)
            baseline.add((path, kind, detail))

    return baseline


def write_baseline(violations: dict) -> None:
    """Records the current violations as the baseline.

    Args:
        violations: The scan result, keyed by `(path, kind, detail)`.

    Verification:
        Correct means: a check run right after this passes with no stale
        entries.
    """
    lines = [
        "# Port-boundary violations the tree still carries (check_port_boundaries.py).",
        "# An entry may be deleted but not added by hand: fix new code instead.",
        "# Format: <path> <kind> <detail>; see docs/design/ports_move_plan.md.",
        "",
    ]
    lines.extend(" ".join(key) for key in sorted(violations))
    _BASELINE.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    """Runs the check and prints its findings.

    Returns:
        1 when a violation is not in the baseline (or, with --all, when any
        violation exists), otherwise 0.

    Verification:
        Correct means: see the module docstring.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--all", action="store_true", help="list every violation, ignoring the baseline")
    parser.add_argument("--write-baseline", action="store_true", help="record the current violations")
    args = parser.parse_args()

    sources = list_sources()
    source_set = set(sources)
    violations = {}
    for path in sources:
        violations.update(scan_file(path, source_set))

    if args.write_baseline:
        write_baseline(violations)
        print(f"Recorded {len(violations)} violations in {_BASELINE.relative_to(_ROOT)}")
        return 0

    baseline = set() if args.all else load_baseline()
    new = sorted(key for key in violations if key not in baseline)
    for path, kind, detail in new:
        print(f"{path}:{violations[(path, kind, detail)]}: {kind}: {detail}")

    for path, kind, detail in sorted(baseline - set(violations)):
        print(f"stale baseline entry (delete it): {path} {kind} {detail}")

    if new:
        print(f"{len(new)} port-boundary violation(s); see docs/design/ports_move_plan.md, \"Rules\".")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
