#pragma once

#include "message_traits.hpp"
#include "rosette/rtps/participant.hpp"

namespace milkros {

template<typename MessageT, typename CallbackT>
class Subscription {
public:
    Subscription() noexcept = default;

    Subscription(rtps::StatefulReader* reader,
                 CallbackT callback) noexcept
        : reader_(reader)
        , callback_(callback) {
        if (reader_) {
            reader_->set_on_change_available(&Subscription::on_change, this);
        }
    }

    bool valid() const noexcept { return reader_ != nullptr; }

private:
    static void on_change(const rtps::CacheChange& change,
                          void* user_data) noexcept {
        if (!user_data) return;

        auto* self = static_cast<Subscription*>(user_data);
        using traits = message_traits<MessageT>;
        static_assert(has_message_traits_v<MessageT>,
                      "milkros::message_traits specialization is required");

        MessageT message = traits::make_message();
        if (!traits::decode(change.payload.data,
                            change.payload.size,
                            message)) {
            return;
        }

        self->callback_(message);
    }

    rtps::StatefulReader* reader_ = nullptr;
    CallbackT callback_{};
};

} // namespace milkros
