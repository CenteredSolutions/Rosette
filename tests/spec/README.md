# milkDDS Spec-Conformance Tests

This directory hosts standards-first conformance tests for `milkDDS`.

The current pilot targets:

- RTPS parser implementation: `lib/milkDDS/rtps/message_parser.cpp`
- RTPS builder implementation: `lib/milkDDS/rtps/message_builder.hpp`
- RTPS fragment assembler implementation: `lib/milkDDS/rtps/fragment_assembler.hpp`
- SPDP discovery implementation: `lib/milkDDS/rtps/spdp.cpp`
- SEDP endpoint-discovery implementation: `lib/milkDDS/rtps/sedp.cpp`
- RTPS history-cache implementation: `lib/milkDDS/rtps/history_cache.hpp`
- Stateful writer reliability implementation: `lib/milkDDS/rtps/stateful_writer.hpp`
- Stateful reader reliability implementation: `lib/milkDDS/rtps/stateful_reader.hpp`
- RTPS participant orchestration implementation: `lib/milkDDS/rtps/participant.cpp`
- Specification baseline: OMG DDSI-RTPS v2.5 (`formal/2022-04-01`)

## Design

- Coverage is tracked by **spec statement IDs**, not source lines.
- Each test case carries a **spec-facing docstring** (normative statement
  paraphrase) printed on pass/fail output.
- The traceability matrix is machine-readable:
  - `spec/ddsi_rtps_2_5/message_parser.matrix.json`
  - `spec/ddsi_rtps_2_5/message_builder.matrix.json`
  - `spec/ddsi_rtps_2_5/fragment_assembler.matrix.json`
  - `spec/ddsi_rtps_2_5/spdp.matrix.json`
  - `spec/ddsi_rtps_2_5/sedp.matrix.json`
  - `spec/ddsi_rtps_2_5/history_cache.matrix.json`
  - `spec/ddsi_rtps_2_5/stateful_writer.matrix.json`
  - `spec/ddsi_rtps_2_5/stateful_reader.matrix.json`
  - `spec/ddsi_rtps_2_5/participant.matrix.json`
- Tests emit passed statement-linked test IDs:
  - `message_parser_passed.json` in build directory
  - `message_builder_passed.json` in build directory
  - `fragment_assembler_passed.json` in build directory
  - `spdp_passed.json` in build directory
  - `sedp_passed.json` in build directory
  - `history_cache_passed.json` in build directory
  - `stateful_writer_passed.json` in build directory
  - `stateful_reader_passed.json` in build directory
  - `participant_passed.json` in build directory
- A small report tool computes standards coverage from matrix + pass set.
- Test implementation is split by RTPS section (`src/test_9_4_*.cpp`) to keep
  review and expansion aligned with the standard.

## Build And Run

```bash
cmake -S lib/milkDDS/tests/spec -B lib/milkDDS/tests/spec/build
cmake --build lib/milkDDS/tests/spec/build
ctest --test-dir lib/milkDDS/tests/spec/build --output-on-failure
```

Coverage report:

```bash
cmake --build lib/milkDDS/tests/spec/build --target message_parser_spec_report
cmake --build lib/milkDDS/tests/spec/build --target message_builder_spec_report
cmake --build lib/milkDDS/tests/spec/build --target fragment_assembler_spec_report
cmake --build lib/milkDDS/tests/spec/build --target spdp_spec_report
cmake --build lib/milkDDS/tests/spec/build --target sedp_spec_report
cmake --build lib/milkDDS/tests/spec/build --target history_cache_spec_report
cmake --build lib/milkDDS/tests/spec/build --target stateful_writer_spec_report
cmake --build lib/milkDDS/tests/spec/build --target stateful_reader_spec_report
cmake --build lib/milkDDS/tests/spec/build --target participant_spec_report
```

## Fuzzing

Build with fuzz target enabled (Clang required):

```bash
cmake -S lib/milkDDS/tests/spec -B lib/milkDDS/tests/spec/build-fuzz \
  -DMILKDDS_SPEC_ENABLE_FUZZ=ON \
  -DCMAKE_CXX_COMPILER=clang++
cmake --build lib/milkDDS/tests/spec/build-fuzz -j
cmake --build lib/milkDDS/tests/spec/build-fuzz --target \
  message_parser_fuzz_corpus \
  fragment_assembler_fuzz_corpus \
  spdp_fuzz_corpus \
  sedp_fuzz_corpus \
  endpoint_match_fuzz_corpus
```

Run with generated corpus:

```bash
lib/milkDDS/tests/spec/build-fuzz/message_parser_fuzz \
  lib/milkDDS/tests/spec/build-fuzz/fuzz/corpus/message_parser
lib/milkDDS/tests/spec/build-fuzz/fragment_assembler_fuzz \
  lib/milkDDS/tests/spec/build-fuzz/fuzz/corpus/fragment_assembler
lib/milkDDS/tests/spec/build-fuzz/spdp_fuzz \
  lib/milkDDS/tests/spec/build-fuzz/fuzz/corpus/spdp
lib/milkDDS/tests/spec/build-fuzz/sedp_fuzz \
  lib/milkDDS/tests/spec/build-fuzz/fuzz/corpus/sedp
lib/milkDDS/tests/spec/build-fuzz/endpoint_match_fuzz \
  lib/milkDDS/tests/spec/build-fuzz/fuzz/corpus/endpoint_match
```

Convenience smoke pass over all replay harnesses:

```bash
cmake --build lib/milkDDS/tests/spec/build-fuzz --target fuzz_smoke_all
```

Recommended longer fuzz campaigns (example 5-minute runs each):

```bash
ASAN_OPTIONS=detect_leaks=0 lib/milkDDS/tests/spec/build-fuzz/message_parser_fuzz -max_total_time=300 lib/milkDDS/tests/spec/build-fuzz/fuzz/corpus/message_parser
ASAN_OPTIONS=detect_leaks=0 lib/milkDDS/tests/spec/build-fuzz/fragment_assembler_fuzz -max_total_time=300 lib/milkDDS/tests/spec/build-fuzz/fuzz/corpus/fragment_assembler
ASAN_OPTIONS=detect_leaks=0 lib/milkDDS/tests/spec/build-fuzz/spdp_fuzz -max_total_time=300 lib/milkDDS/tests/spec/build-fuzz/fuzz/corpus/spdp
ASAN_OPTIONS=detect_leaks=0 lib/milkDDS/tests/spec/build-fuzz/sedp_fuzz -max_total_time=300 lib/milkDDS/tests/spec/build-fuzz/fuzz/corpus/sedp
ASAN_OPTIONS=detect_leaks=0 lib/milkDDS/tests/spec/build-fuzz/endpoint_match_fuzz -max_total_time=300 lib/milkDDS/tests/spec/build-fuzz/fuzz/corpus/endpoint_match
```

If Clang/libFuzzer is unavailable, you can still run corpus replay smoke tests:

```bash
cmake -S lib/milkDDS/tests/spec -B lib/milkDDS/tests/spec/build-fuzz \
  -DMILKDDS_SPEC_ENABLE_FUZZ=ON
cmake --build lib/milkDDS/tests/spec/build-fuzz -j \
  --target \
  message_parser_fuzz_replay message_parser_fuzz_corpus message_parser_fuzz_smoke \
  fragment_assembler_fuzz_replay fragment_assembler_fuzz_corpus fragment_assembler_fuzz_smoke \
  spdp_fuzz_replay spdp_fuzz_corpus spdp_fuzz_smoke \
  sedp_fuzz_replay sedp_fuzz_corpus sedp_fuzz_smoke \
  endpoint_match_fuzz_replay endpoint_match_fuzz_corpus endpoint_match_fuzz_smoke
```

## Scope Notes

- This is a host-side suite; Pico SDK and FreeRTOS are not required.
- The current matrices include statements implemented by parser, builder, fragment assembler, SPDP, SEDP, history_cache, stateful writer, stateful reader, and participant modules.
- Rows marked `deferred` are intentionally tracked but not yet fully tested.
