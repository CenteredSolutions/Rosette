// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "rosette/rtps/message_parser.hpp"

#include <stddef.h>
#include <stdint.h>

using namespace rosette::rtps;

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    ParseOps ops{};
    parse_message(reinterpret_cast<const Octet*>(data),
                  static_cast<uint32_t>(size),
                  GuidPrefix{},
                  ops,
                  nullptr);
    return 0;
}

