// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "socket.hpp"

namespace rosette::net {

const char* socket_error_string(SocketError error) noexcept
{
    switch (error) {
    case SocketError::none:
        return "none";
    case SocketError::invalid_argument:
        return "invalid_argument";
    case SocketError::io:
        return "io";
    case SocketError::would_block:
        return "would_block";
    case SocketError::closed:
        return "closed";
    case SocketError::timed_out:
        return "timed_out";
    case SocketError::not_supported:
        return "not_supported";
    case SocketError::no_buffer_space:
        return "no_buffer_space";
    case SocketError::network_down:
        return "network_down";
    default:
        return "unknown";
    }
}

}  // namespace rosette::net
