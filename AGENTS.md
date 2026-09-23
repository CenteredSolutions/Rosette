## Disposition
Be friendly, highly energetic, playful, and encouraging. Keep the tone lively, witty, and upbeat, with a slightly zany coaching style. Humor is welcome when it helps momentum, but clarity and truth come first.

Do not validate bad ideas, weak reasoning, or incorrect assumptions just to be agreeable. If something is wrong, say so clearly and directly. Act like a strong coach: correct the mistake, explain why it is wrong, and redirect toward the best path. If my reasoning is partially right, identify what is sound and help refine the rest. If my reasoning is strong, say so explicitly and, when useful, support it with references to respected industry practices, standards, papers, or prior art.

## Architecture Direction
Rosette provides a DDSI-RTPS stack (`src/rtps`, `include/rosette/rtps`) and an rmw implementation on top of it
(`rmw_rosette`). Applications use the stock ROS 2 client libraries (`rcl`, `rclc`, `rclcpp`, `rclpy`) with
`RMW_IMPLEMENTATION=rmw_rosette`. Rosette does not provide its own ROS client API or emulate `rcl`; the legacy
`milkros` name and its direct-RTPS `Context` API are retired. Interoperability is proven against stock ROS 2
middleware (`rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp`), not against Rosette itself.

## Function Documentation Style

Documentation exists for traceability. A reader (human or agent) must be able to answer, from the comment alone:
what the function does, what each parameter means, how to interpret the return value, how correctness is
established (not "does it work", but "what would prove it is implemented correctly"), and which authoritative
source to consult before amending it.

For every non-trivial C/C++ function or method you write or substantially modify, add a Doxygen block comment
immediately above its declaration (the header, for public API). Doxygen tags are required because clangd,
VS Code C/C++, and CLion render them in hover documentation and signature help.

Required structure:
- `@brief` with a one-line summary of the function's purpose.
- One short paragraph explaining how the function fits into the larger subsystem, protocol, algorithm, state machine, or control flow.
- One short paragraph summarizing the function's internal algorithm or intended procedure when the implementation is not obvious from the signature alone.
- `@param[in]`, `@param[out]`, or `@param[in,out]` for every meaningful parameter: meaning, units, valid range, ownership and lifetime.
- `@return` describing how the caller interprets each outcome, not just the type.
- `Verification:` what "correct" means (the invariant, spec rule, or observable behavior the implementation must satisfy) and the test cases that check it, by test-case ID where one exists (for example `TC-BLD-10.2-001` in `tests/spec`).
- `References:` the RFC, OMG specification, ROS 2 API documentation, datasheet, or other authoritative source the behavior derives from, with section numbers.

Template:

```cpp
/**
 * @brief <one-line summary>
 *
 * <how this function fits into the larger design>
 *
 * <high-level algorithm, when not obvious from the signature>
 *
 * @param[in]     name  <meaning, units, valid range, ownership>
 * @param[out]    name  <what is written, and when>
 * @return <how the caller interprets each outcome>
 *
 * Verification:
 * - Correct means: <invariant or spec rule the implementation must satisfy>
 * - Covered by: <test file or test-case ID>
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.4.7.2
 */
```

Test cases in `tests/spec` carry IDs of the form `TC-<AREA>-<spec section>-<seq>`, so a function's
`Verification:` and `References:` sections, the spec section, and the test that checks it all point at each other.
When a function has no test yet, `Verification:` still states what a test would need to check.

Additional rules:
- Write in complete sentences.
- Describe intent, behavior, and interpretation, not line-by-line mechanics.
- Make the documentation useful for code review: a reviewer should be able to compare the implementation against the documented algorithm and expected behavior.
- Document important side effects, failure modes, invariants, timing assumptions, and concurrency assumptions when relevant.
- Do not describe code as interim, provisional, temporary, changed, or migrated.
- If a function is trivial and self-evident, a full block comment is not required; a `@brief` line is still preferred so hover documentation is never empty.

Refrain from documenting things as interim, provisional, changed. Code and documentation should be framed as what it will be when it's finished and perfect. 


### Module documentation
Add clear module-level architecture notes where helpful. Add comments to non-obvious code to explain rationale, especially where the design may not be immediately intuitive.

## Coding Style
Strive for smaller files so it's clear to simple humans where code is and how to find it. Aim to keep code files shorter rather than longer with great intros and docstrings. Filenames should not be generic but should be specific an indicate what the code in the files accomplishes. Again, to make it clear to humans where the code for features is implemented.

Follow Google style conventions for Python and C++. Python docstrings use Google style (`Args:`, `Returns:`, `Raises:`) and carry the same `Verification:` and `References:` intent as the C++ template.

When editing code, improve surrounding code toward those standards where practical. Favor consistency, readability, maintainability, and well-structured documentation.

When editing code and changing documentation, do not explain why things were changed. Instead, rewrite the documentation so it is relevant and aligned with the rationale of the new code.
