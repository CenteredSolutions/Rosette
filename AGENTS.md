# Working on Rosette

## Architecture Direction
Rosette puts a microcontroller on a ROS 2 network. It supplies the networking layer (socket backends, the WIZnet
driver, address providers) and the storage layer, and it supports two ROS 2 backends on top of them. A firmware
designer picks one per project based on its constraints:

- **`rmw_rosette`**: a DDSI-RTPS stack (`src/rtps`, `include/rosette/rtps`) and an rmw implementation on top of it
  (`rmw_rosette/`). The board is a DDS participant and talks to every peer directly; no extra process is needed.
  On a host, any `rclcpp` or `rclpy` program runs on it with `RMW_IMPLEMENTATION=rmw_rosette`.
- **micro-ROS agent**: `rmw_microxrcedds` over XRCE-DDS to a `micro_ros_agent` on a host, which joins the DDS graph
  on the board's behalf. It costs less flash and RAM on the board and needs the agent running on the network.

Application code is stock `rclc` and builds against either backend unchanged. Only session setup differs: the agent
backend discovers, pings, and reconnects to its agent, and `rmw_rosette` has no such step. Everything Rosette adds
above the rmw layer stays consistent with both backends; a feature that works on only one of them is a defect in
that feature. Rosette does not provide its own ROS client API or emulate `rcl`, and the legacy `milkros` name and its
direct-RTPS `Context` API are retired.

Interoperability is proven against stock ROS 2 middleware (`rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp`) and the stock
`micro_ros_agent`, not against Rosette itself.

## Function Documentation Style
Documentation states the contract: what a function is for, what it promises, and which authority defines correct
behavior. A reader can choose and call the function, and a tester can write a test for it, without reading its body.
The headers are also the source for generated API pages, so the public contract lives there. Explain the subsystem,
protocol, or state machine a class belongs to once, in the file or class comment, not on every function.

Use Doxygen tags, which clangd, VS Code C/C++, and CLion render in hover documentation and signature help. Document
public API in the header, above the declaration.

- `@brief`: one sentence, on every function, so hover documentation is never empty.
- Purpose, for public API: one or two sentences on when to use this function and, when a near neighbor exists, when
  to use that one instead.
- `@param[in]`, `@param[out]`, `@param[in,out]`: every parameter on public API, so the generated pages are
  complete. Elsewhere, only parameters whose meaning, units, valid range, ownership, or lifetime the name and type do
  not already say.
- `@return`: how the caller interprets each outcome, not just the type.
- `Verification:`: the observable behavior a correct implementation must show, derived from the governing source and
  stated so a test can be built from it without reading the code. A test written from it checks that the function
  does what it is supposed to do, not merely that it runs. List the test-case IDs that check it, where they exist;
  the statement stands without them.
- `References:`: the RFC, OMG specification, datasheet register or timing table, or ROS 2 API documentation the
  behavior derives from, with section numbers.
- `Example:`, for public API entry points only (the class or function a caller reaches for first: a class comment, a
  factory or constructor, the main operation of a class, a port hook a board implements): a `@code{.cpp}` ...
  `@endcode` block of about ten lines showing the call in a larger program, with what is constructed first, how the
  result is checked, and what is torn down. Use real names and signatures, and name the compiled program or test that
  does the same thing (`Full program: examples/led_files/main.cpp`), which is the source of truth when the two
  disagree. Accessors, and methods a class-level example already shows, need no example of their own.

Private helpers with no external authority need only a `@brief` and any non-obvious side effect; they are tested
through the function that calls them.

```cpp
/**
 * @brief <one sentence>
 *
 * <public API: when to use this, and when to use its neighbor instead>
 *
 * @param[in]  name  <meaning, units, valid range, ownership>
 * @param[out] name  <what is written, and when>
 * @return <how the caller interprets each outcome>
 *
 * Verification:
 * - Correct means: <observable behavior the governing source requires>
 * - Covered by: <test-case IDs, where they exist>
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.4.7.2
 *
 * Example:
 * @code{.cpp}
 * <a few lines: set up, call, check the result, tear down>
 * @endcode
 * Full program: <examples/... or the test that does the same>
 */
```

Test cases in `tests/spec` carry IDs of the form `TC-<AREA>-<spec section>-<seq>` (for example
`TC-BLD-10.2-001`), so a function's `Verification:` and `References:` sections, the spec section, and the test that
checks it all point at each other.

Be brief. Do not repeat the signature or describe how the code does its job; state each section in the fewest
complete sentences that carry it. Document side effects, failure modes, timing assumptions, and concurrency
assumptions where a caller depends on them. Frame code and documentation as finished: never describe them as
interim, provisional, temporary, changed, or migrated.

### Module documentation
Add clear module-level architecture notes where helpful. Add comments to non-obvious code to explain rationale, especially where the design may not be immediately intuitive.

### Diagrams and tables
When a layout, state machine, or exchange is the point, draw it instead of describing it, the way RFCs draw
headers and state machines and datasheets tabulate registers. In source comments, use ASCII diagrams for wire
formats, bit fields, buffer layouts, and small state machines, and Markdown tables for state transitions, limits,
and register maps. Keep each one within the 100-column limit and wrap ASCII art in `@verbatim` so neither
clang-format nor Doxygen reflows it. Put Mermaid sequence and component diagrams in `docs/design` and PR
descriptions, where GitHub renders them. A diagram lives in the file or class comment next to the code it
describes, names the spec figure or table it follows, and is updated in the same change as that code; function
comments refer to it ("transitions T4–T6 above") instead of repeating it.

## Coding Style
Strive for smaller files so it's clear to simple humans where code is and how to find it. Aim to keep code files
shorter rather than longer with great intros and docstrings. Filenames should not be generic but should be specific
and indicate what the code in the files accomplishes. Again, to make it clear to humans where the code for features
is implemented.

Follow Google style conventions for Python and C++, with the project rules below taking precedence where they
differ. Python docstrings use Google style (`Args:`, `Returns:`, `Raises:`) and follow the same tiers as C++: a summary
line on every function; purpose and full `Args:`/`Returns:`/`Raises:` on public API; `Verification:` and
`References:` where an external source defines correct behavior; only the summary and non-obvious side effects on
private helpers.

### Wrapped expressions
When a boolean or arithmetic expression wraps, break before the operator so each continuation line starts with it.
The leading `&&`, `||`, `+`, or `?` tells the reader at a glance that the line continues an expression, and the
operands line up for comparison. This is the rule PEP 8 adopts for Python (after Knuth), and the same rule applies
to C++ here.

```cpp
if (reader.is_reliable()
    && proxy.highest_acked < change.sequence_number
    && !proxy.is_lost(change.sequence_number)) {
    resend(change);
}
```

### Namespace aliases
The RTPS stack is `rosette::rtps`. A `.cpp` file that uses it heavily (three or more references) may open with
`namespace rtps = rosette::rtps;` after its includes, so call sites read `rtps::Guid`. Headers always spell out
`rosette::rtps::`: an alias in a header leaks into every includer and reintroduces a top-level `rtps` name.

### Blocks and vertical whitespace
- The body of every `if`, `else`, `for`, `while`, and `do` goes on its own line, indented, inside braces, even when
  it is a single statement. `if (done) return;` on one line is not used.
- Separate the logical steps of a function with a blank line, so a reader can see which statements belong to which
  step, loop, or branch.
- A closing `}` on a line by itself is always followed by a blank line before the next statement. The exceptions
  are a `}` followed by another `}`, by `else`, by the `while` of a `do` loop, or by a `case` label.
- A run of one-line definitions that together read as a table, such as a chip's register offsets or a set of
  `constexpr` traits, may sit on consecutive lines with no blank line between them. Start a new group with a blank
  line and a heading comment. A definition whose body spans lines still ends in a lone `}` and takes a blank line
  after it.

```cpp
for (auto& proxy : proxies_) {
    if (proxy.is_matched()) {
        send_heartbeat(proxy);
    }

    proxy.age_out(now);
}

return proxies_.size();
```

```cpp
// ── common registers ──
static constexpr address_type common_mode() { return reg(0x0000u, common_block()); }     // MR
static constexpr address_type common_gateway() { return reg(0x0001u, common_block()); }  // GAR
static constexpr address_type common_subnet() { return reg(0x0005u, common_block()); }   // SUBR
```

### Function length
A function fits on one screen: no more than 100 lines, without exception. Below that ceiling, split a function only
when the piece being extracted is a real concept with a name a reader would recognize, such as a spec procedure or a
state-machine transition. Do not split a straight-line procedure into many small helpers just to shorten it; the
reader then has to jump between definitions to follow one sequence of steps. Private helpers are tested through the
function that calls them, not individually.

### Enforcement
`.clang-format` encodes the formatting rules, pinned to clang-format 18.1.8. `tools/style/run_style_checks.sh`
runs every check locally and in the GitHub `style` workflow: formatting and the blank-line rule on lines changed
since the merge base with `main`, and the function-length cap across the tree, where functions already over the cap
are listed in `tools/style/function_length_baseline.txt` and may only shrink. Install the tools with
`python3 -m pip install -r tools/style/requirements.txt`.

### Editing existing code
When editing code, improve surrounding code toward those standards where practical. Favor consistency, readability,
maintainability, and well-structured documentation.

When editing code and changing documentation, do not explain why things were changed. Instead, rewrite the
documentation so it is relevant and aligned with the rationale of the new code.
