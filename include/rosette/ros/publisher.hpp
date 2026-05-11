#pragma once

#include "message_traits.hpp"
#include "rosette/rtps/participant.hpp"

namespace milkros {

template<typename MessageT>
class Publisher {
public:
    Publisher() noexcept = default;

    explicit Publisher(rtps::StatefulWriter<1>* writer) noexcept
        : writer_(writer) {}

    bool valid() const noexcept { return writer_ != nullptr; }

    bool publish(const MessageT& message) noexcept {
        if (!writer_) return false;

        using traits = message_traits<MessageT>;
        static_assert(has_message_traits_v<MessageT>,
                      "milkros::message_traits specialization is required");

        uint8_t payload[traits::max_serialized_size] = {};
        const size_t encoded = traits::encode(payload, sizeof(payload), message);
        if (encoded == 0u) return false;

        return writer_->write_reliable(rtps::ChangeKind::ALIVE,
                                       payload,
                                       static_cast<uint32_t>(encoded));
    }

private:
    rtps::StatefulWriter<1>* writer_ = nullptr;
};

} // namespace milkros
