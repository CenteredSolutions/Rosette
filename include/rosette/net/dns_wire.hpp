#pragma once

/**
 * @file net/dns_wire.hpp
 * @brief Minimal DNS message reader and writer for one-shot mDNS / DNS-SD queries.
 *
 * Design goal
 * -----------
 * Give the mDNS resolver just enough of the DNS wire format to ask one question
 * and walk the answer records of the reply. This is not a general resolver: it
 * parses forward through a single UDP datagram and never allocates.
 *
 * Scope
 * -----
 * - Encodes a query: header + one question, with the mDNS "unicast response"
 *   bit optionally set in the question class.
 * - Decodes a reply: header counts, then each resource record in order, with
 *   support for the 0xC0 compression pointers that Avahi uses in SRV targets
 *   and PTR data.
 * - Typed readers for the three record types DNS-SD needs: A, SRV, PTR.
 *
 * References
 * ----------
 * - RFC 1035 sec. 4.1: message format, question and resource-record layout.
 * - RFC 1035 sec. 4.1.4: message compression (0xC0 pointer labels).
 * - RFC 2782: SRV record rdata (priority, weight, port, target).
 * - RFC 6762 sec. 5.4: the top bit of the question QCLASS requests a unicast
 *   response to a one-shot multicast query.
 * - RFC 6763 sec. 4: DNS-SD maps a service to PTR -> SRV/TXT -> A records.
 */

#include "rosette/net/inet.h"

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <cstring>

namespace rosette::net::dns {

/// Resource-record and question TYPE values (RFC 1035 sec. 3.2.2, RFC 2782).
inline constexpr uint16_t kTypeA = 1u;
inline constexpr uint16_t kTypePtr = 12u;
inline constexpr uint16_t kTypeTxt = 16u;
inline constexpr uint16_t kTypeSrv = 33u;
inline constexpr uint16_t kTypeAaaa = 28u;

/// The only CLASS this code uses, plus the mDNS unicast-response request bit.
inline constexpr uint16_t kClassIn = 1u;
inline constexpr uint16_t kQuUnicastResponseBit = 0x8000u;
/// Cache-flush bit lives in the same position on a response record's class.
inline constexpr uint16_t kClassMask = 0x7FFFu;

/// Standard query header: QR=0, opcode=0, RD=0. mDNS ignores the ID, so 0 is fine.
inline constexpr uint16_t kFlagsStandardQuery = 0x0000u;

inline constexpr size_t kHeaderSize = 12u;
/// A dotted name never expands past 255 bytes on the wire (RFC 1035 sec. 3.1).
inline constexpr size_t kMaxNameLength = 256u;

/**
 * @brief Parsed fixed-size DNS message header (RFC 1035 sec. 4.1.1).
 */
struct Header {
    uint16_t id = 0u;
    uint16_t flags = 0u;
    uint16_t question_count = 0u;
    uint16_t answer_count = 0u;
    uint16_t authority_count = 0u;
    uint16_t additional_count = 0u;

    /// Total records after the question section: answers + authority + additional.
    uint16_t record_count() const noexcept
    {
        return static_cast<uint16_t>(answer_count + authority_count +
                                     additional_count);
    }
};

/**
 * @brief One resource record's framing, with its rdata left in place.
 *
 * `owner` is the decompressed record-owner name. `rdata_offset` points at the
 * first rdata byte within the original message so the typed readers below can
 * resolve compression pointers against the whole datagram.
 */
struct Record {
    char owner[kMaxNameLength] = {};
    uint16_t type = 0u;
    uint16_t klass = 0u;
    uint32_t ttl = 0u;
    uint16_t rdata_length = 0u;
    size_t rdata_offset = 0u;
};

/// Case-insensitive comparison of two DNS names (RFC 1035 sec. 2.3.3).
inline bool names_equal(const char* a, const char* b) noexcept
{
    if ((a == nullptr) || (b == nullptr)) {
        return false;
    }
    for (; (*a != '\0') && (*b != '\0'); ++a, ++b) {
        char ca = *a;
        char cb = *b;
        if ((ca >= 'A') && (ca <= 'Z')) {
            ca = static_cast<char>(ca + ('a' - 'A'));
        }
        if ((cb >= 'A') && (cb <= 'Z')) {
            cb = static_cast<char>(cb + ('a' - 'A'));
        }
        if (ca != cb) {
            return false;
        }
    }
    return (*a == '\0') && (*b == '\0');
}

/// SRV rdata: the agent's port and the host name that owns it (RFC 2782).
struct Srv {
    uint16_t priority = 0u;
    uint16_t weight = 0u;
    uint16_t port = 0u;
    char target[kMaxNameLength] = {};
};

/**
 * @brief Forward-only reader over one received DNS datagram.
 *
 * The reader keeps a cursor for sequential access (header, then records) and
 * also exposes offset-addressed helpers so record rdata can be parsed without
 * disturbing the record walk.
 */
class Reader {
public:
    Reader(const uint8_t* message, size_t length) noexcept
        : message_(message)
        , length_(length)
    {}

    bool ok() const noexcept { return ok_; }

    /**
     * @brief Read the 12-byte header and leave the cursor at the question section.
     *
     * @param[out] out Parsed header counts and flags.
     * @return True when 12 bytes were available.
     */
    bool read_header(Header& out) noexcept
    {
        if (!have(kHeaderSize)) {
            return false;
        }
        out.id = inet::load_be16(message_ + 0u);
        out.flags = inet::load_be16(message_ + 2u);
        out.question_count = inet::load_be16(message_ + 4u);
        out.answer_count = inet::load_be16(message_ + 6u);
        out.authority_count = inet::load_be16(message_ + 8u);
        out.additional_count = inet::load_be16(message_ + 10u);
        cursor_ = kHeaderSize;
        return true;
    }

    /**
     * @brief Skip @p count question entries (name + QTYPE + QCLASS each).
     *
     * @param[in] count Question count from the header.
     * @return True when every question was well-formed and in bounds.
     */
    bool skip_questions(uint16_t count) noexcept
    {
        char discard[kMaxNameLength];
        for (uint16_t i = 0u; i < count; ++i) {
            if (!consume_name(discard, sizeof(discard)) || !advance(4u)) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Read the next resource record's framing and step over its rdata.
     *
     * Fills `out.owner` with the decompressed record-owner name and leaves
     * `out.rdata_offset` pointing into the message so read_a() / read_srv() /
     * read_ptr() can be called for the same record afterwards.
     *
     * @param[out] out Owner name, type, class, TTL, and rdata location.
     * @return True when a complete record was read; false at end of message or
     *         on any malformed field.
     */
    bool read_record(Record& out) noexcept
    {
        if (!consume_name(out.owner, sizeof(out.owner)) ||
            !have_from(cursor_, 10u)) {
            return false;
        }
        out.type = inet::load_be16(message_ + cursor_);
        out.klass = inet::load_be16(message_ + cursor_ + 2u);
        out.ttl = inet::load_be32(message_ + cursor_ + 4u);
        out.rdata_length = inet::load_be16(message_ + cursor_ + 8u);
        cursor_ += 10u;
        out.rdata_offset = cursor_;
        return advance(out.rdata_length);
    }

    /**
     * @brief Extract the IPv4 address from an A record.
     *
     * @param[in]  record A record framing from read_record().
     * @param[out] out    Address in network order (out[0] is the first octet).
     * @return True when the record is type A with a 4-byte rdata.
     */
    bool read_a(const Record& record, std::array<uint8_t, 4>& out) const noexcept
    {
        if ((record.type != kTypeA) || (record.rdata_length != 4u) ||
            !have_from(record.rdata_offset, 4u)) {
            return false;
        }
        std::memcpy(out.data(), message_ + record.rdata_offset, 4u);
        return true;
    }

    /**
     * @brief Extract priority, weight, port, and target host from an SRV record.
     *
     * @param[in]  record SRV record framing from read_record().
     * @param[out] out    Parsed SRV fields; @c out.target is NUL-terminated.
     * @return True when the record is type SRV and its rdata parsed cleanly.
     */
    bool read_srv(const Record& record, Srv& out) const noexcept
    {
        if ((record.type != kTypeSrv) || !have_from(record.rdata_offset, 6u)) {
            return false;
        }
        out.priority = inet::load_be16(message_ + record.rdata_offset + 0u);
        out.weight = inet::load_be16(message_ + record.rdata_offset + 2u);
        out.port = inet::load_be16(message_ + record.rdata_offset + 4u);

        return decode_name_at(record.rdata_offset + 6u, out.target,
                              sizeof(out.target));
    }

    /**
     * @brief Extract the pointed-to name from a PTR record.
     *
     * @param[in]  record PTR record framing from read_record().
     * @param[out] out    Destination buffer for the NUL-terminated name.
     * @param[in]  out_capacity Size of @p out.
     * @return True when the record is type PTR and the name fit in @p out.
     */
    bool read_ptr(const Record& record,
                  char* out,
                  size_t out_capacity) const noexcept
    {
        if (record.type != kTypePtr) {
            return false;
        }
        return decode_name_at(record.rdata_offset, out, out_capacity);
    }

private:
    bool have(size_t count) const noexcept { return have_from(cursor_, count); }

    bool have_from(size_t offset, size_t count) const noexcept
    {
        return ok_ && (message_ != nullptr) && (offset <= length_) &&
               ((length_ - offset) >= count);
    }

    bool advance(size_t count) noexcept
    {
        if (!have(count)) {
            ok_ = false;
            return false;
        }
        cursor_ += count;
        return true;
    }

    /**
     * @brief Decode the name at the cursor and advance past its on-wire bytes.
     *
     * The decoded (compression-expanded) name is written to @p out. The cursor
     * then steps over the literal labels until a terminating zero byte or a
     * compression pointer, which is always the last thing in a name.
     *
     * @param[out] out          Destination for the NUL-terminated dotted name.
     * @param[in]  out_capacity Size of @p out.
     * @return True when the name decoded and the cursor stayed in bounds.
     */
    bool consume_name(char* out, size_t out_capacity) noexcept
    {
        if (!decode_name_at(cursor_, out, out_capacity)) {
            ok_ = false;
            return false;
        }
        for (;;) {
            if (!have(1u)) {
                ok_ = false;
                return false;
            }
            const uint8_t label = message_[cursor_];
            if (label == 0u) {
                ++cursor_;
                return true;
            }
            if ((label & 0xC0u) == 0xC0u) {
                return advance(2u);
            }
            if ((label & 0xC0u) != 0u) {
                ok_ = false;
                return false;
            }
            if (!advance(1u + label)) {
                return false;
            }
        }
    }

    /**
     * @brief Decode a (possibly compressed) name into dotted text.
     *
     * Follows compression pointers within the message. A bounded follow count
     * rejects pointer loops. @p cursor is only used as a starting offset; the
     * caller's record walk is driven by read_record(), not this function.
     *
     * @param[in]  start        Offset of the first label byte.
     * @param[out] out          Destination for the NUL-terminated dotted name.
     * @param[in]  out_capacity Size of @p out.
     * @return True when the whole name fit in @p out and stayed in bounds.
     */
    bool decode_name_at(size_t start, char* out, size_t out_capacity) const noexcept
    {
        if ((out == nullptr) || (out_capacity == 0u)) {
            return false;
        }

        size_t offset = start;
        size_t written = 0u;
        unsigned pointer_follows = 0u;

        for (;;) {
            if (!have_from(offset, 1u)) {
                return false;
            }
            const uint8_t label = message_[offset];

            if (label == 0u) {
                break;
            }

            if ((label & 0xC0u) == 0xC0u) {
                if (!have_from(offset, 2u) || (++pointer_follows > 8u)) {
                    return false;
                }
                offset = (static_cast<size_t>(label & 0x3Fu) << 8u) |
                         message_[offset + 1u];
                continue;
            }

            if ((label & 0xC0u) != 0u) {
                return false;
            }

            const size_t label_length = label;
            if (!have_from(offset + 1u, label_length)) {
                return false;
            }
            // Room for a separating '.' before every label but the first.
            const size_t needed = label_length + ((written == 0u) ? 0u : 1u);
            if ((written + needed + 1u) > out_capacity) {
                return false;
            }
            if (written != 0u) {
                out[written++] = '.';
            }
            std::memcpy(out + written, message_ + offset + 1u, label_length);
            written += label_length;
            offset += 1u + label_length;
        }

        out[written] = '\0';
        return true;
    }

    const uint8_t* message_ = nullptr;
    size_t length_ = 0u;
    size_t cursor_ = 0u;
    bool ok_ = true;
};

/**
 * @brief Builds a standard query holding exactly one question.
 *
 * mDNS one-shot discovery only ever needs a single-question query, so the
 * builder is deliberately fixed at that shape.
 */
class QueryBuilder {
public:
    QueryBuilder(uint8_t* buffer, size_t capacity) noexcept
        : buffer_(buffer)
        , capacity_(capacity)
    {}

    /**
     * @brief Encode the query into the buffer.
     *
     * @param[in] qname            Dotted name to ask about, e.g.
     *                             "_microrosagent._udp.local".
     * @param[in] qtype            Question TYPE (kTypePtr, kTypeA, ...).
     * @param[in] unicast_response When true, set the RFC 6762 QU bit so the
     *                             responder replies to our source port.
     * @return Encoded length in bytes, or 0 if the name was malformed or the
     *         buffer was too small.
     */
    size_t build(const char* qname,
                 uint16_t qtype,
                 bool unicast_response) noexcept
    {
        if ((buffer_ == nullptr) || (capacity_ < kHeaderSize) ||
            (qname == nullptr)) {
            return 0u;
        }

        std::memset(buffer_, 0, kHeaderSize);
        inet::store_be16(buffer_ + 2u, kFlagsStandardQuery);
        inet::store_be16(buffer_ + 4u, 1u);  // one question
        size_t pos = kHeaderSize;

        if (!encode_name(qname, pos)) {
            return 0u;
        }

        const uint16_t qclass = static_cast<uint16_t>(
            kClassIn | (unicast_response ? kQuUnicastResponseBit : 0u));
        if ((capacity_ - pos) < 4u) {
            return 0u;
        }
        inet::store_be16(buffer_ + pos, qtype);
        inet::store_be16(buffer_ + pos + 2u, qclass);
        return pos + 4u;
    }

private:
    /**
     * @brief Append @p name as a sequence of length-prefixed labels plus a
     *        terminating zero byte.
     */
    bool encode_name(const char* name, size_t& pos) noexcept
    {
        const char* label = name;
        while (*label != '\0') {
            const char* dot = label;
            while ((*dot != '\0') && (*dot != '.')) {
                ++dot;
            }
            const size_t label_length = static_cast<size_t>(dot - label);
            if ((label_length == 0u) || (label_length > 63u) ||
                ((capacity_ - pos) < (1u + label_length))) {
                return false;
            }
            buffer_[pos++] = static_cast<uint8_t>(label_length);
            std::memcpy(buffer_ + pos, label, label_length);
            pos += label_length;
            label = (*dot == '.') ? (dot + 1u) : dot;
        }
        if ((capacity_ - pos) < 1u) {
            return false;
        }
        buffer_[pos++] = 0u;
        return true;
    }

    uint8_t* buffer_ = nullptr;
    size_t capacity_ = 0u;
};

}  // namespace rosette::net::dns
