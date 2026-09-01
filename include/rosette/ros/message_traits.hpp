#pragma once

#include <stddef.h>

namespace rosette {

template<typename MessageT>
struct message_traits;

template<typename MessageT>
inline constexpr bool has_message_traits_v = false;

template<typename MessageT>
struct default_reader_options {
    static constexpr MessageT make() noexcept { return MessageT{}; }
};

} // namespace rosette
