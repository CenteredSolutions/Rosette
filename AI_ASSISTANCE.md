# AI-assisted engineering in Rosette

Rosette is built with AI coding assistants as a substantial part of the
implementation process, under engineering direction and review by Centered
Solutions, LLC. This file describes how that works, so that users, contributors
and reviewers can judge the result on its evidence.

## Who does what

The engineering team owns the system: its architecture, the requirements, the
choice of standards to implement, the review of every change, integration,
hardware validation on the bench, and maintenance. AI assistants draft
implementation, tests and documentation against that direction, investigate
failures, and review changes. Nothing merges without a human reviewing it, and
a human is accountable for every commit.

## Tools

| Assistant | Vendor | Used for |
| --- | --- | --- |
| Claude Code (Claude Opus 5.5, Claude Sonnet 5.5, Claude Sonnet 5) | Anthropic | Implementation, tests, documentation, debugging on the hardware bench, code review |
| ChatGPT | OpenAI | Design discussion and release planning |
| GitHub Copilot Autofix | GitHub | Fixes for code-scanning findings |

Commits drafted with an assistant credit it with a `Co-Authored-By:` trailer.
At the first public release, 231 of the 269 non-merge commits on `main` carry
one (Claude Opus 5.5: 170, Claude Sonnet 5.5: 30, Claude Sonnet 5: 27, Copilot
Autofix: 4). `git log --format=%B | grep -i '^co-authored-by'` reproduces the
count.

## How AI-written code is held to account

The project is set up so that correctness is established by evidence rather
than by trust in whoever, or whatever, wrote a change:

- **[AGENTS.md](AGENTS.md) is the working contract.** It states the
  architecture, the coding style, and a documentation rule: every non-trivial
  function says what proves it correct (`Verification:`) and which
  specification section it derives from (`References:`). A reviewer compares
  the implementation against the documented algorithm and the cited standard.
- **Spec-keyed conformance tests.** `tests/spec` checks DDSI-RTPS statements by
  specification section, with test IDs of the form `TC-<AREA>-<section>-<seq>`
  that the code's documentation cites.
- **Interoperability against stock middleware, not against Rosette.** The
  `tests/ros2` harness runs Rosette against unmodified `rmw_fastrtps_cpp`,
  `rmw_cyclonedds_cpp`, and the stock `micro_ros_agent`.
- **Hardware in the loop.** Firmware claims in the README are marked "runs"
  only after they run on the board against stock ROS 2.
- **Automated gates.** Formatting, function length, port boundaries, license
  headers, sanitizers, and fuzzing run in CI on every pull request.

## Standards and intellectual property

Rosette is an independent implementation of published standards and
interfaces: the OMG DDS Interoperability Wire Protocol (DDSI-RTPS) 2.5, OMG
CDR encoding as profiled by DDS-XTypes, the ROS 2 middleware interface
(`rmw`), and, through micro-ROS, OMG DDS-XRCE. Its socket layer is modeled on
the POSIX sockets interface (IEEE Std 1003.1) and runs over it on hosts. The
project does not claim ownership of these standards or of the intellectual
property of their owners. ROS and ROS 2 are trademarks of Open Robotics;
Rosette is not affiliated with or endorsed by Open Robotics.

Copyright in the copyrightable portions of Rosette is held by Centered
Solutions, LLC, subject to applicable third-party rights and to the
limitations of copyright protection for AI-generated material. The project is
released under the Apache License, Version 2.0, whose patent grant and
contribution terms apply to all of it.

## For contributors

AI-assisted contributions are welcome on the same terms as any other: you sign
off on the commit (see [CONTRIBUTING.md](CONTRIBUTING.md)), you have reviewed
and understood every line, you credit the assistant with a `Co-Authored-By:`
trailer, and you do not submit output that reproduces third-party code under
an incompatible license.
