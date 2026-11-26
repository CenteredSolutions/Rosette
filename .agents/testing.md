# Testing

[GETTING_STARTED.md](../GETTING_STARTED.md) shows how to run each suite. This file says which suites prove a change,
and lists the traps that make a run lie.

## Which tests prove a change

| Change | Run |
| --- | --- |
| `src/rtps`, `include/rosette/rtps` | `tests/spec` and `tests/loopback` |
| Parsers of wire data (messages, fragments, SPDP/SEDP, matching) | the above, plus the fuzz replay (`tests/spec/README.md`) |
| Networking, address providers, WIZnet, buffer pools | `tests/host` |
| The platform contract or a port | `tests/platform` on the host, then on the board |
| `rmw_rosette/` | its colcon tests (below), on Humble and Jazzy when compat code moves |
| Anything a peer sees on the wire | the `tests/ros2` sessions against `rmw_fastrtps_cpp` and `rmw_cyclonedds_cpp` |

Host suites: `cmake -S tests/<suite> -B build/tests-<suite>`, then build and `ctest --output-on-failure`.
CI also builds them with `-DROSETTE_SANITIZE=ON`.

`rmw_rosette`, from a sourced ROS 2 environment, the way CI runs it:

```bash
colcon build --base-paths rmw_rosette
colcon test --base-paths rmw_rosette
colcon test-result --verbose
```

Without `--base-paths`, `colcon test` can find no package and report "0 tests", which looks like a pass.

## Traps

- **`set -u` before sourcing ROS.** `/opt/ros/<distro>/setup.bash` reads unset variables and aborts the script.
- **A symlinked package finds the wrong tree.** `ROSETTE_ROOT` defaults to the package's parent directory. In a
  workspace that symlinks `src/rmw_rosette`, that is the workspace's `src/`. Pass
  `--cmake-args -DROSETTE_ROOT=<repo or worktree>`.
- **A failed build still runs the sessions.** The `tests/ros2` scripts then fail with `rx=0`, which reads like a
  protocol bug. Confirm the build finished before reading a session's result.
- **Parallel runs find each other.** Containers on Docker's default bridge share multicast, and session names and
  log directories collide when two containers start in the same second. Give every parallel run its own
  `ROSETTE_DOMAIN` and its own `TMPDIR` and log directory. Check a capture's participant count before trusting it.
- **Losing SEDP without losing SPDP.** Inside a container without host networking (`--cap-add NET_ADMIN`), run
  `tc qdisc add dev lo root netem loss N%`. Unicast between local processes crosses `lo`; multicast loopback skips
  the qdisc.
- **UBSan keeps going under libFuzzer.** Set `UBSAN_OPTIONS=halt_on_error=1` (as
  `tools/fuzz/run_fuzz_campaign.sh` does), or a finding only appears in the log.
- **`pgrep -f` matches itself.** A pattern that also appears in the shell's own command line matches that shell:
  `while pgrep -f x` never ends, and `pkill -f x` kills the caller. Anchor the pattern (`"^python3 .*probe"`), or
  run the jobs in the foreground and `wait`.

## Containers

Test containers and images are disposable. Recreate one from `ros:<distro>-ros-base` (`rosdep install
--from-paths rmw_rosette`), or mount the worktree into a running one. Don't `docker commit` a container to test
another branch: its writable layer holds every build and can be tens of GB. Remove a stuck or bloated container
instead of working around it.
