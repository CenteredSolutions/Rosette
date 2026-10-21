/**
 * @file replay_driver.cpp
 * @brief Feeds every file of a corpus directory to a fuzz harness.
 *
 * Each fuzz target is a harness defining LLVMFuzzerTestOneInput(). Linked with
 * libFuzzer it explores new inputs; linked with this driver it replays a fixed
 * corpus, so the same harness code runs under any compiler and under the
 * ROSETTE_SANITIZE sanitizers. A replay passes when every input returns
 * without a sanitizer report, assertion, or crash.
 *
 * Usage: <name>_fuzz_replay <corpus-dir>
 */

#include <stddef.h>
#include <stdint.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " <corpus-dir>\n";
        return 2;
    }

    const std::filesystem::path corpus_dir = argv[1];
    if (!std::filesystem::is_directory(corpus_dir)) {
        std::cerr << "corpus directory not found: " << corpus_dir << "\n";
        return 2;
    }

    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(corpus_dir)) {
        if (entry.is_regular_file()) {
            files.push_back(entry.path());
        }
    }

    // Sorted so a failure reproduces in the same order on every run.
    std::sort(files.begin(), files.end());

    for (const auto& path : files) {
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            std::cerr << "cannot read " << path << "\n";
            return 2;
        }

        const std::vector<uint8_t> data((std::istreambuf_iterator<char>(in)),
                                        std::istreambuf_iterator<char>());
        LLVMFuzzerTestOneInput(data.data(), data.size());
    }

    if (files.empty()) {
        std::cerr << "corpus directory is empty: " << corpus_dir << "\n";
        return 2;
    }

    std::cout << "Replayed " << files.size() << " corpus files from " << corpus_dir << "\n";
    return 0;
}
