// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_rosette/cdr_stream.hpp
 * @brief Bounds-checked little-endian XCDR1 (PLAIN_CDR) byte streams.
 *
 * ROS 2 DDS middlewares exchange message payloads as OMG CDR version 1
 * ("classic" / PLAIN_CDR, encapsulation identifier CDR_LE = 0x0001). Every
 * primitive is aligned to its own size (1, 2, 4, or 8 bytes) relative to the
 * first byte after the 4-byte encapsulation header; structures add no padding
 * of their own. CdrWriter and CdrReader implement exactly those rules over a
 * caller-owned buffer, and the introspection codec (introspection_c_codec.hpp)
 * drives them field by field.
 *
 * Both classes report overflow through a sticky `ok()` flag instead of
 * exceptions, matching the no-exceptions posture of rosette::rtps: once a
 * write or read would pass the end of the buffer, every later call is a
 * no-op and `ok()` stays false.
 *
 * The streams always use little-endian byte order. Readers accept only
 * CDR_LE payloads (see introspection_c_codec.cpp), which is what every stock
 * ROS 2 RMW emits on little-endian hosts.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace rmw_rosette {

/**
 * @brief Appends CDR-aligned little-endian primitives to a fixed buffer.
 *
 * Alignment is computed from the start of `buf`, so `buf` must point at the
 * first byte of the CDR body (immediately after the encapsulation header).
 * A writer over a null buffer measures: it stores nothing, and length()
 * reports the octets the same writes take in a real buffer, so an encoder
 * can size its buffer exactly before encoding into it.
 */
class CdrWriter {
public:
    /**
     * @brief Creates a writer over a caller-owned buffer.
     *
     * @param[out] buf Destination; must stay valid for the writer's lifetime.
     *                 Null measures instead of writing.
     * @param[in]  cap Capacity of `buf` in bytes; SIZE_MAX when measuring.
     */
    CdrWriter(uint8_t* buf, size_t cap) noexcept : buf_(buf), cap_(cap) {}

    /// @brief Writes one primitive of `sizeof(T)` bytes, aligned to its size.
    template <typename T>
    void put(T value) noexcept {
        align(sizeof(T));
        put_bytes(&value, sizeof(T));
    }

    /**
     * @brief Pads with zero bytes until the offset is a multiple of `n`.
     *
     * @param[in] n Alignment in bytes; 1, 2, 4, or 8.
     */
    void align(size_t n) noexcept {
        while (ok_ && (pos_ % n) != 0) put_raw_byte(0);
    }

    /**
     * @brief Copies raw bytes with no alignment.
     *
     * @param[in] src Source bytes; may be null only when `len` is 0.
     * @param[in] len Number of bytes.
     */
    void put_bytes(const void* src, size_t len) noexcept {
        if (!ok_ || len > cap_ - pos_) { ok_ = false; return; }
        if (len && buf_) {
            std::memcpy(buf_ + pos_, src, len);
        }

        pos_ += len;
    }

    /// @brief True until any write overflowed the buffer.
    bool ok() const noexcept { return ok_; }
    /// @brief Number of bytes written so far, including padding.
    size_t length() const noexcept { return pos_; }

private:
    void put_raw_byte(uint8_t b) noexcept { put_bytes(&b, 1); }

    uint8_t* buf_;
    size_t   cap_;
    size_t   pos_ = 0;
    bool     ok_ = true;
};

/**
 * @brief Consumes CDR-aligned little-endian primitives from a fixed buffer.
 *
 * Alignment is computed from the start of `buf`, which must point at the first
 * byte of the CDR body (immediately after the encapsulation header).
 */
class CdrReader {
public:
    /**
     * @brief Creates a reader over caller-owned bytes.
     *
     * @param[in] buf Source; must stay valid for the reader's lifetime.
     * @param[in] len Number of readable bytes in `buf`.
     */
    CdrReader(const uint8_t* buf, size_t len) noexcept : buf_(buf), len_(len) {}

    /// @brief Reads one primitive of `sizeof(T)` bytes after aligning to its size.
    template <typename T>
    T get() noexcept {
        T value{};
        align(sizeof(T));
        get_bytes(&value, sizeof(T));
        return value;
    }

    /**
     * @brief Skips padding until the offset is a multiple of `n`.
     *
     * @param[in] n Alignment in bytes; 1, 2, 4, or 8.
     */
    void align(size_t n) noexcept {
        const size_t pad = (n - (pos_ % n)) % n;
        if (!ok_ || pad > len_ - pos_) { ok_ = false; return; }
        pos_ += pad;
    }

    /**
     * @brief Copies raw bytes out with no alignment.
     *
     * @param[out] dst Destination; untouched when the read would overflow.
     * @param[in]  len Number of bytes.
     */
    void get_bytes(void* dst, size_t len) noexcept {
        if (!ok_ || len > len_ - pos_) { ok_ = false; return; }
        if (len) std::memcpy(dst, buf_ + pos_, len);
        pos_ += len;
    }

    /**
     * @brief Returns a pointer to the next `len` bytes and consumes them.
     *
     * @param[in] len Number of bytes.
     * @return Pointer into the source buffer, or `nullptr` on overflow.
     */
    const uint8_t* take_view(size_t len) noexcept {
        if (!ok_ || len > len_ - pos_) { ok_ = false; return nullptr; }
        const uint8_t* p = buf_ + pos_;
        pos_ += len;
        return p;
    }

    /// @brief Bytes left before the end of the buffer.
    size_t remaining() const noexcept { return len_ - pos_; }
    /// @brief True until any read overflowed or a decoder called fail().
    bool ok() const noexcept { return ok_; }
    /// @brief Marks the stream invalid, e.g. on a semantic decode error.
    void fail() noexcept { ok_ = false; }

private:
    const uint8_t* buf_;
    size_t         len_;
    size_t         pos_ = 0;
    bool           ok_ = true;
};

}  // namespace rmw_rosette
