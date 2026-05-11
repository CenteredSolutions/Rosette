/**
 * @file ros/service.hpp
 * @brief ROS 2-compatible embedded service server and client.
 *
 * Analogues
 * ---------
 *   milkros::Service<T,H>      ≈  rclcpp::Service<T>
 *   milkros::Client<T,N>       ≈  rclcpp::Client<T>
 *   milkros::service_traits<T> ≈  rosidl_typesupport specialisation
 *
 * Wire format
 * -----------
 * Topic names follow the ROS 2 DDS convention:
 *   Request:  rq/<namespace>/<service_name>Request
 *   Response: rr/<namespace>/<service_name>Reply
 *
 * The request/response payload carries only the service_traits-encoded
 * message body.  Request/response correlation uses RTPS sample identity:
 *
 *   Request identity:
 *     Derived from the incoming DATA sample's writer GUID + sequence number.
 *   Response correlation:
 *     Sent via RelatedSampleIdentity InlineQoS in the response DATA submessage.
 *
 * This matches the ROS 2 service wire expectation closely enough for
 * DATA-only service interoperability.  DATA_FRAG InlineQoS is not yet
 * implemented in Rosette.
 *
 * Defining service_traits
 * -----------------------
 *   struct MyService {};
 *
 *   template<> struct milkros::service_traits<MyService> {
 *       using Request  = MyRequest;
 *       using Response = MyResponse;
 *       static const char* request_type_name()  noexcept { return "my_pkg::srv::dds_::MyService_Request_"; }
 *       static const char* response_type_name() noexcept { return "my_pkg::srv::dds_::MyService_Response_"; }
 *       static constexpr size_t max_request_serialized_size  = 64u;
 *       static constexpr size_t max_response_serialized_size = 64u;
 *       static Request  make_request()  noexcept { return {}; }
 *       static Response make_response() noexcept { return {}; }
 *       static size_t encode_request (uint8_t* d, size_t c, const Request&  m) noexcept { ... }
 *       static bool   decode_request (const uint8_t* s, size_t n, Request&  m) noexcept { ... }
 *       static size_t encode_response(uint8_t* d, size_t c, const Response& m) noexcept { ... }
 *       static bool   decode_response(const uint8_t* s, size_t n, Response& m) noexcept { ... }
 *   };
 *   template<> inline constexpr bool milkros::has_service_traits_v<MyService> = true;
 *
 * Pool sizes
 * ----------
 *   MILKROS_MAX_SERVICES      (default 4) — service server slots in Context
 *   MILKROS_MAX_SERVICE_SIZE  (default 128 B) — max sizeof(Service<T,H>)
 *   MILKROS_MAX_CLIENTS       (default 4) — client slots in Context
 *   MILKROS_MAX_CLIENT_SIZE   (default 256 B) — max sizeof(Client<T,N>)
 */

#pragma once

#include "message_traits.hpp"
#include "rosette/rtps/participant.hpp"

#include <stdint.h>
#include <string.h>

#ifndef MILKROS_MAX_SERVICES
#  define MILKROS_MAX_SERVICES 4u
#endif
#ifndef MILKROS_MAX_SERVICE_SIZE
/// Must be >= sizeof(Service<ServiceT, LargestHandlerT>) for all handlers used.
#  define MILKROS_MAX_SERVICE_SIZE 128u
#endif
#ifndef MILKROS_MAX_CLIENTS
#  define MILKROS_MAX_CLIENTS 4u
#endif
#ifndef MILKROS_MAX_CLIENT_SIZE
/// Must be >= sizeof(Client<ServiceT, MaxPending>) for the largest MaxPending used.
/// Default 256 B covers Client<T,4> with typical payload sizes.
#  define MILKROS_MAX_CLIENT_SIZE 256u
#endif

namespace milkros {

// ============================================================
// RequestId — 24-byte correlation identity used by services/actions
// ============================================================

/// Identifies a specific in-flight service request.
/// The wire-equivalent RTPS identity is { writer GUID, sequence number }.
struct RequestId {
    uint8_t client_guid[16];  ///< Writer GUID: GuidPrefix(12B) + EntityId(4B).
    int64_t sequence_number;  ///< Monotonically increasing per client writer.

    bool operator==(const RequestId& o) const noexcept {
        return sequence_number == o.sequence_number &&
               memcmp(client_guid, o.client_guid, 16u) == 0;
    }
    bool operator!=(const RequestId& o) const noexcept { return !(*this == o); }
};

static_assert(sizeof(RequestId) == 24u, "RequestId wire size must be exactly 24 bytes");

inline RequestId request_id_from_sample_identity(
        const rtps::SampleIdentity& sid) noexcept
{
    RequestId rid{};
    memcpy(rid.client_guid, sid.writerGuid.prefix.value, 12u);
    memcpy(rid.client_guid + 12u, sid.writerGuid.entityId.entityKey, 3u);
    rid.client_guid[15] = sid.writerGuid.entityId.entityKind;
    rid.sequence_number = sid.sequenceNumber.to_int64();
    return rid;
}

inline RequestId request_id_from_guid_sn(const rtps::Guid&           writer_guid,
                                         const rtps::SequenceNumber& sn) noexcept
{
    return request_id_from_sample_identity(rtps::SampleIdentity{writer_guid, sn});
}

inline rtps::SampleIdentity sample_identity_from_request_id(
        const RequestId& rid) noexcept
{
    rtps::SampleIdentity sid{};
    memcpy(sid.writerGuid.prefix.value, rid.client_guid, 12u);
    memcpy(sid.writerGuid.entityId.entityKey, rid.client_guid + 12u, 3u);
    sid.writerGuid.entityId.entityKind = rid.client_guid[15];
    sid.sequenceNumber = rtps::SequenceNumber::from_int64(rid.sequence_number);
    return sid;
}

// ============================================================
// service_traits<ServiceT> — user specializes this
// ============================================================

template<typename ServiceT>
struct service_traits;  ///< Must be specialised before instantiating Service<T> or Client<T>.

template<typename ServiceT>
inline constexpr bool has_service_traits_v = false;

// ============================================================
// ServiceOptions
// ============================================================

struct ServiceOptions {
    rtps::ReliabilityKind reliability = rtps::ReliabilityKind::RELIABLE;
    rtps::DurabilityKind  durability  = rtps::DurabilityKind::VOLATILE;
};

// ============================================================
// IService — type-erased base stored in Context pool
// ============================================================

class IService {
protected:
    ~IService() = default;
};

// ============================================================
// Service<ServiceT, HandlerT> — server side
// ============================================================

/**
 * @brief ROS 2-compatible service server.
 *
 * @tparam ServiceT   Service tag type with a service_traits<ServiceT> specialisation.
 * @tparam HandlerT   Callable with signature:
 *                      void(const Request& request, Response& response) noexcept
 *
 * Instances are created via Node::create_service() and owned by the Context
 * placement-new pool.  Never move or copy after creation.
 *
 * The handler is called synchronously from the RTPS RX task when a request
 * arrives, so it must complete quickly and not block.
 */
template<typename ServiceT, typename HandlerT>
class Service final : public IService {
    static_assert(has_service_traits_v<ServiceT>,
                  "milkros::service_traits specialization required for ServiceT");

public:
    using traits   = service_traits<ServiceT>;
    using Request  = typename traits::Request;
    using Response = typename traits::Response;

    Service() noexcept = default;

    Service(rtps::StatefulReader* req_reader,
            rtps::StatefulWriter<1>* resp_writer,
            HandlerT handler) noexcept
        : req_reader_ (req_reader)
        , resp_writer_(resp_writer)
        , handler_    (static_cast<HandlerT&&>(handler))
    {
        if (req_reader_) {
            req_reader_->set_on_change_available(&Service::on_request, this);
        }
    }

    // Move-only: after a move the old object is invalid; the callback this-pointer
    // must be re-registered on the new address.
    Service(const Service&)            = delete;
    Service& operator=(const Service&) = delete;

    Service(Service&& o) noexcept
        : req_reader_ (o.req_reader_)
        , resp_writer_(o.resp_writer_)
        , handler_    (static_cast<HandlerT&&>(o.handler_))
    {
        o.req_reader_  = nullptr;
        o.resp_writer_ = nullptr;
        if (req_reader_) {
            req_reader_->set_on_change_available(&Service::on_request, this);
        }
    }

    Service& operator=(Service&& o) noexcept {
        if (this != &o) {
            req_reader_  = o.req_reader_;
            resp_writer_ = o.resp_writer_;
            handler_     = static_cast<HandlerT&&>(o.handler_);
            o.req_reader_  = nullptr;
            o.resp_writer_ = nullptr;
            if (req_reader_) {
                req_reader_->set_on_change_available(&Service::on_request, this);
            }
        }
        return *this;
    }

    bool valid() const noexcept {
        return req_reader_ != nullptr && resp_writer_ != nullptr;
    }

private:
    static void on_request(const rtps::CacheChange& change,
                            void* user_data) noexcept {
        if (!user_data) return;
        auto* self = static_cast<Service*>(user_data);

        const uint8_t* src = change.payload.data;
        const size_t   len = change.payload.size;
        const RequestId rid =
            request_id_from_guid_sn(change.writerGuid, change.sequenceNumber);

        Request req = traits::make_request();
        if (!traits::decode_request(src, len, req)) {
            return;
        }

        // Call the user handler.
        Response resp = traits::make_response();
        self->handler_(req, resp);

        constexpr size_t kMaxPayload = traits::max_response_serialized_size;
        uint8_t buf[kMaxPayload];

        const size_t encoded = traits::encode_response(
            buf,
            sizeof(buf),
            resp);
        if (encoded == 0u) return;

        rtps::InlineQos inline_qos{};
        inline_qos.hasRelatedSampleIdentity = true;
        inline_qos.relatedSampleIdentity = sample_identity_from_request_id(rid);

        self->resp_writer_->write_reliable(rtps::ChangeKind::ALIVE,
                                            buf,
                                            static_cast<uint32_t>(encoded),
                                            inline_qos);
    }

    rtps::StatefulReader*  req_reader_  = nullptr;
    rtps::StatefulWriter<1>* resp_writer_ = nullptr;
    HandlerT               handler_{};
};

// ============================================================
// Client<ServiceT, MaxPending> — client side
// ============================================================

/**
 * @brief ROS 2-compatible service client.
 *
 * @tparam ServiceT   Service tag type with a service_traits<ServiceT> specialisation.
 * @tparam MaxPending Maximum number of concurrent in-flight requests (default 4).
 *
 * Response callbacks use a plain function-pointer + void* pattern for
 * zero-overhead embedded use.  To use a lambda, wrap it in a static context:
 *
 *   static MyState ctx;
 *   client->async_send_request(req,
 *       [](const Response& r, void* ud) {
 *           static_cast<MyState*>(ud)->handle(r);
 *       }, &ctx);
 *
 * Instances are created via Node::create_client() and owned by the Context
 * placement-new pool.  Never move or copy after creation.
 */
template<typename ServiceT, uint8_t MaxPending = 4>
class Client final : public IService {
    static_assert(has_service_traits_v<ServiceT>,
                  "milkros::service_traits specialization required for ServiceT");
    static_assert(MaxPending >= 1u, "MaxPending must be at least 1");

public:
    using traits   = service_traits<ServiceT>;
    using Request  = typename traits::Request;
    using Response = typename traits::Response;

    /// Callback type for async_send_request: (response, user_data).
    using ResponseCallback = void(*)(const Response&, void*) noexcept;

    Client() noexcept = default;

    Client(rtps::StatefulWriter<1>* req_writer,
           rtps::StatefulReader*    resp_reader) noexcept
        : req_writer_ (req_writer)
        , resp_reader_(resp_reader)
    {
        if (resp_reader_) {
            resp_reader_->set_on_change_available(&Client::on_response, this);
        }
        // Capture writer GUID for request correlation.
        if (req_writer_) {
            const rtps::Guid& g = req_writer_->guid();
            memcpy(writer_guid_, g.prefix.value,   12u);
            memcpy(writer_guid_ + 12u, g.entityId.entityKey, 3u);
            writer_guid_[15] = g.entityId.entityKind;
        }
    }

    // Move-only
    Client(const Client&)            = delete;
    Client& operator=(const Client&) = delete;

    Client(Client&& o) noexcept
        : req_writer_ (o.req_writer_)
        , resp_reader_(o.resp_reader_)
    {
        memcpy(writer_guid_, o.writer_guid_, 16u);
        memcpy(pending_,     o.pending_,     sizeof(pending_));
        o.req_writer_  = nullptr;
        o.resp_reader_ = nullptr;
        if (resp_reader_) {
            resp_reader_->set_on_change_available(&Client::on_response, this);
        }
    }

    Client& operator=(Client&& o) noexcept {
        if (this != &o) {
            req_writer_  = o.req_writer_;
            resp_reader_ = o.resp_reader_;
            memcpy(writer_guid_, o.writer_guid_, 16u);
            memcpy(pending_,     o.pending_,     sizeof(pending_));
            o.req_writer_  = nullptr;
            o.resp_reader_ = nullptr;
            if (resp_reader_) {
                resp_reader_->set_on_change_available(&Client::on_response, this);
            }
        }
        return *this;
    }

    bool valid() const noexcept {
        return req_writer_ != nullptr && resp_reader_ != nullptr;
    }

    /// Number of currently pending (un-answered) requests.
    uint8_t pending_count() const noexcept {
        uint8_t n = 0u;
        for (uint8_t i = 0u; i < MaxPending; ++i) {
            if (pending_[i].in_use) ++n;
        }
        return n;
    }

    /**
     * @brief Send a request asynchronously.
     *
     * @param req     Request to send.
     * @param cb      Called when the matching response arrives (may be nullptr).
     * @param ud      Passed verbatim to cb.
     * @returns true if the request was enqueued and sent, false if no pending
     *          slot was free or the writer is invalid.
     */
    bool async_send_request(const Request&   req,
                             ResponseCallback cb  = nullptr,
                             void*            ud  = nullptr) noexcept {
        if (!req_writer_) return false;

        // Find a free pending slot.
        int slot = -1;
        for (uint8_t i = 0u; i < MaxPending; ++i) {
            if (!pending_[i].in_use) { slot = static_cast<int>(i); break; }
        }
        if (slot < 0) return false;

        constexpr size_t kMaxPayload = traits::max_request_serialized_size;
        uint8_t buf[kMaxPayload];

        const size_t encoded = traits::encode_request(
            buf,
            sizeof(buf),
            req);
        if (encoded == 0u) return false;

        rtps::SequenceNumber req_sn{};
        if (!req_writer_->write_reliable(
                rtps::ChangeKind::ALIVE, buf,
                static_cast<uint32_t>(encoded),
                rtps::InlineQos{}, &req_sn)) {
            return false;
        }

        RequestId rid{};
        memcpy(rid.client_guid, writer_guid_, 16u);
        rid.sequence_number = req_sn.to_int64();

        // Record pending entry.
        pending_[slot].id      = rid;
        pending_[slot].cb      = cb;
        pending_[slot].ud      = ud;
        pending_[slot].in_use  = true;
        return true;
    }

private:
    struct PendingEntry {
        RequestId        id{};
        ResponseCallback cb      = nullptr;
        void*            ud      = nullptr;
        bool             in_use  = false;
    };

    static void on_response(const rtps::CacheChange& change,
                              void* user_data) noexcept {
        if (!user_data) return;
        auto* self = static_cast<Client*>(user_data);

        const uint8_t* src = change.payload.data;
        const size_t   len = change.payload.size;
        if (!change.inlineQos.hasRelatedSampleIdentity) return;
        const RequestId rid =
            request_id_from_sample_identity(change.inlineQos.relatedSampleIdentity);

        // Find the matching pending slot.
        for (uint8_t i = 0u; i < MaxPending; ++i) {
            PendingEntry& e = self->pending_[i];
            if (!e.in_use || e.id != rid) continue;

            // Decode the response body.
            Response resp = traits::make_response();
            if (traits::decode_response(src, len, resp)) {
                if (e.cb) e.cb(resp, e.ud);
            }

            e.in_use = false;
            return;
        }
        // Unmatched response — ignore (may be for a different client).
    }

    rtps::StatefulWriter<1>* req_writer_  = nullptr;
    rtps::StatefulReader*    resp_reader_ = nullptr;
    uint8_t                  writer_guid_[16] = {};
    PendingEntry             pending_[MaxPending] = {};
};

} // namespace milkros
