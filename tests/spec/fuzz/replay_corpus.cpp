#include "rosette/rtps/message_parser.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace rtps;

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " <corpus-dir>\n";
        return 2;
    }

    const std::filesystem::path corpus_dir = argv[1];
    if (!std::filesystem::exists(corpus_dir) || !std::filesystem::is_directory(corpus_dir)) {
        std::cerr << "corpus directory not found: " << corpus_dir << "\n";
        return 2;
    }

    ParseOps ops{};
    size_t files = 0;
    for (const auto& entry : std::filesystem::directory_iterator(corpus_dir)) {
        if (!entry.is_regular_file()) continue;
        std::ifstream in(entry.path(), std::ios::binary);
        if (!in) continue;

        std::vector<Octet> data((std::istreambuf_iterator<char>(in)),
                                std::istreambuf_iterator<char>());
        parse_message(data.data(),
                      static_cast<uint32_t>(data.size()),
                      GuidPrefix{},
                      ops,
                      nullptr);
        ++files;
    }

    std::cout << "Replayed " << files << " corpus files from " << corpus_dir << "\n";
    return 0;
}

