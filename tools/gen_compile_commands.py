#!/usr/bin/env python3
"""Merge this project's compile_commands.json fragments into one at the repo root.

clangd (and Zed's clangd integration) find `compile_commands.json` by walking
up from the open file toward the project root — so one merged file at the
root covers both the host test suite and any board firmware build you've
configured. clangd's background index then gives every header in
`include/rosette/` proper flags via whichever `.cpp`/`.c` includes it.

What this pulls in:
  - `build/tests-host/` — (re)configured here with
    `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`; fast, host toolchain only, covers
    the address layer and every host test.
  - any `compile_commands.json` already sitting under `examples/*/build*/` or
    `build*/` at the repo root — e.g. a board firmware build you configured
    with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`. Not created for you (a board
    build needs the ARM toolchain + a pico-sdk fetch); see README below.
  - vendor/dependency sources (`_deps`, fetched pico-sdk/FreeRTOS/etc.) are
    filtered out — clangd doesn't need them indexed, and they bloat the file.

Usage:
    tools/gen_compile_commands.py

Re-run after adding a new source file, or after (re)configuring a board build
with compile commands export on:

    cmake -S . -B examples/led_files/build-pico -DROSETTE_BOARD=w5100s-evb-pico \\
          -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    tools/gen_compile_commands.py
"""

import json
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
PROJECT_DIRS = ("include", "src", "examples", "boards", "tests", "bindings")


def is_project_file(file_path: str) -> bool:
    """True for a real, tracked source file — not a build artifact or vendored dep."""
    try:
        rel = pathlib.Path(file_path).relative_to(ROOT)
    except ValueError:
        return False
    parts = rel.parts
    if not parts or parts[0] not in PROJECT_DIRS:
        return False
    if any(p == "_deps" or p.startswith("build") for p in parts):
        return False
    return True


def load_filtered(db_path: pathlib.Path) -> list:
    try:
        entries = json.loads(db_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        print(f"  ! skipping {db_path} ({exc})", file=sys.stderr)
        return []
    return [e for e in entries if is_project_file(e["file"])]


def configure_host_tests() -> pathlib.Path:
    build_dir = ROOT / "build" / "tests-host"
    print(f"-- configuring {build_dir.relative_to(ROOT)} (host tests)")
    subprocess.run(
        ["cmake", "-S", str(ROOT / "tests" / "host"), "-B", str(build_dir),
         "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"],
        check=True, stdout=subprocess.DEVNULL,
    )
    return build_dir / "compile_commands.json"


def discover_board_databases() -> list:
    found = []
    for pattern in ("examples/*/build*/compile_commands.json", "build*/compile_commands.json"):
        found.extend(sorted(ROOT.glob(pattern)))
    return found


def main() -> int:
    databases = [configure_host_tests()]
    databases += discover_board_databases()

    merged: dict[str, dict] = {}
    for db in databases:
        if not db.is_file():
            continue
        entries = load_filtered(db)
        print(f"-- {db.relative_to(ROOT)}: {len(entries)} project source(s)")
        for e in entries:
            merged[e["file"]] = e    # last write wins on overlap

    out = ROOT / "compile_commands.json"
    out.write_text(json.dumps(list(merged.values()), indent=2) + "\n")
    print(f"-- wrote {out.relative_to(ROOT)}: {len(merged)} entries total")

    if len(databases) == 1:
        print(
            "\nNote: no board build found (examples/*/build*/ or build*/ with\n"
            "compile commands exported), so WIZnet/pico-only headers will fall\n"
            "back to weak default flags in clangd. Configure one, e.g.:\n\n"
            "    cmake -S . -B examples/led_files/build-pico \\\n"
            "          -DROSETTE_BOARD=w5100s-evb-pico -DCMAKE_EXPORT_COMPILE_COMMANDS=ON\n\n"
            "then re-run this script."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
