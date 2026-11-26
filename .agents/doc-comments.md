# Doc comments

A doc comment states the contract: what the function is for, what it promises, and, when one exists, which outside
authority defines correct behavior. A caller can use the function, and a tester can test it, without reading its body.
Headers feed the generated API pages, so public contracts live there. Explain a subsystem, protocol, or state
machine once, in the file or class comment.

## What each function gets

| Section | Where |
| --- | --- |
| `@brief` (one sentence) | every function |
| Purpose: when to use it, and when to use a near neighbor instead | public API |
| `@param[in]` / `[out]` / `[in,out]` | every parameter on public API; elsewhere, only what the name and type don't say (units, range, ownership, lifetime) |
| `@return`: how to interpret each outcome | public API, and wherever it isn't obvious |
| `Verification:` | only code implementing an external standard (below) |
| `References:` | the same code: the spec, RFC, datasheet table, or ROS 2 API page, with section numbers |
| `Example:` | public entry points only: a class, a constructor or factory, a class's main operation, a port hook |

Also document side effects, failure modes, and timing or concurrency assumptions a caller depends on. Don't repeat
the signature or narrate the body.

An example uses real names and signatures and names the compiled program or test that does the same thing
(`Full program: examples/led_files/main.cpp`), which wins when the two disagree. Accessors, and methods a class
example already shows, need none.

## Verification and References

Use them where an external authority (DDSI-RTPS, CDR/XTypes, the ROS 2 rmw API, an RFC, a datasheet) defines what
correct means, so that a test can be built from the comment and checked against that source. Code that enforces
the project's own rules (style checkers, build and release scripts, private helpers) has no such authority and
carries neither section; `AGENTS.md` is not a reference.

- `Correct means:` the observable behavior the source requires, stated so a test can be written without the code.
- `Covered by:` the test IDs that check it, where they exist. Spec tests in `tests/spec` are named
  `TC-<AREA>-<spec section>-<seq>` (`TC-HC-8.3.1-005`) so the comment, the spec, and the test point at each other.

## Template

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
 * - Covered by: <test-case IDs>
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.4.7.2
 *
 * Example:
 * @code{.cpp}
 * <about ten lines: construct, call, check the result, tear down>
 * @endcode
 * Full program: <examples/... or the test that does the same; it wins if the two disagree>
 */
```

Doxygen tags render in clangd, VS Code, and CLion hovers. Python uses Google docstrings (`Args:`, `Returns:`,
`Raises:`) with the same tiers.

## Diagrams and tables

Draw a layout, state machine, or exchange when it is the point, the way RFCs and datasheets do:

- In source: ASCII for wire formats, bit fields, buffer layouts, and small state machines, wrapped in `@verbatim`
  and kept within 100 columns. Markdown tables for transitions, limits, and register maps.
- In `docs/design` and PR descriptions: Mermaid sequence and component diagrams.
- A diagram sits in the file or class comment beside its code, names the spec figure or table it follows, and
  changes with that code. Function comments refer to it ("transitions T4–T6 above").
