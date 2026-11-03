// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_common.hpp"
#include "spec_test_registry.hpp"

#include <stdio.h>

namespace {

const spec_test::TestCase* find_case(const std::vector<spec_test::TestCase>& tests,
                                     const char* id) {
    for (const auto& tc : tests) {
        if (std::string(tc.id) == id) return &tc;
    }
    return nullptr;
}

} // namespace

int main(int argc, char** argv) {
    std::string run_only{};
    std::string pass_json{};

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--run" && (i + 1) < argc) {
            run_only = argv[++i];
        } else if (std::string(argv[i]) == "--emit-pass-json" && (i + 1) < argc) {
            pass_json = argv[++i];
        }
    }

    std::vector<spec_test::TestCase> tests{};
    spec_test::register_all_participant_tests(tests);

    std::vector<std::string> passed_ids{};
    int failures = 0;

    if (!run_only.empty()) {
        const spec_test::TestCase* tc = find_case(tests, run_only.c_str());
        if (!tc) {
            fprintf(stderr, "Unknown test id: %s\n", run_only.c_str());
            return 2;
        }
        std::string err{};
        if (!tc->fn(err)) {
            fprintf(stderr, "[FAIL] %s\n  Spec: %s\n  Error: %s\n", tc->id, tc->doc, err.c_str());
            failures++;
        } else {
            fprintf(stdout, "[PASS] %s\n  Spec: %s\n", tc->id, tc->doc);
            passed_ids.push_back(tc->id);
        }
    } else {
        for (const auto& tc : tests) {
            std::string err{};
            if (!tc.fn(err)) {
                fprintf(stderr, "[FAIL] %s\n  Spec: %s\n  Error: %s\n", tc.id, tc.doc, err.c_str());
                failures++;
            } else {
                fprintf(stdout, "[PASS] %s\n  Spec: %s\n", tc.id, tc.doc);
                passed_ids.push_back(tc.id);
            }
        }
    }

    if (!pass_json.empty()) {
        if (!spec_test::write_passed_json(pass_json, passed_ids)) {
            fprintf(stderr, "Failed to write pass JSON: %s\n", pass_json.c_str());
            failures++;
        }
    }

    if (failures != 0) {
        fprintf(stderr, "Participant spec tests failed: %d\n", failures);
        return 1;
    }

    fprintf(stdout, "All participant spec tests passed: %zu\n", passed_ids.size());
    return 0;
}
