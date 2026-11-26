# Code style

Google style for C++ and Python, with these project rules taking precedence. `.clang-format` (clang-format 18.1.8)
encodes what it can; `tools/style/` checks the rest.

## Files

Keep files short, each opening with an intro to what it holds. Name a file for what it implements
(`stateful_writer_replay.cpp`, not `utils.cpp`) so a reader finds a feature by its filename.

## Blocks and vertical whitespace

- Every `if`, `else`, `for`, `while`, and `do` body goes on its own line, in braces, even a single statement.
- A blank line separates the logical steps of a function.
- A `}` alone on its line is followed by a blank line, unless the next line is another `}`, `else`, the `while` of a
  `do` loop, or a `case` label.

```cpp
for (auto& proxy : proxies_) {
    if (proxy.is_matched()) {
        send_heartbeat(proxy);
    }

    proxy.age_out(now);
}

return proxies_.size();
```

## Wrapped expressions

Break before the operator, so each continuation line starts with `&&`, `||`, `+`, or `?` (PEP 8, after Knuth).

```cpp
if (reader.is_reliable()
    && proxy.highest_acked < change.sequence_number
    && !proxy.is_lost(change.sequence_number)) {
    resend(change);
}
```

## Function length

At most 100 lines, signature to closing brace. Below that, extract a piece only when it is a named concept (a spec
procedure, a state-machine transition); a straight-line procedure stays in one function rather than being scattered
across small helpers. Private helpers are tested through their caller.

## Namespace aliases

A `.cpp` file with three or more `rosette::rtps` references may add `namespace rtps = rosette::rtps;` after its
includes. Headers always spell out `rosette::rtps::`, since an alias there leaks into every includer.

## Enforcement

`tools/style/run_style_checks.sh [BASE_REF]` runs locally and in the `style` workflow:

| Check | Scope |
| --- | --- |
| clang-format | lines changed since the merge base with `main` |
| blank line after `}` | lines changed since the merge base |
| function length | whole tree; `function_length_baseline.txt` lists older long functions, which may only shrink |
| port boundaries | whole tree, against `port_boundary_baseline.txt` (see `PORTING.md`) |
| license headers | `reuse lint` |

Install the tools with `python3 -m pip install -r tools/style/requirements.txt`.
