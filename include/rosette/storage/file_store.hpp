/**
 * @file storage/file_store.hpp
 * @brief A content-agnostic keyed blob store over one directory of a volume.
 *
 * `FileStore` maps a short key to a file at `<dir>/<key><extension>` and offers
 * the four operations an over-the-wire management interface needs: put, get,
 * list, erase. It does not interpret file contents at all — JSON, CBOR, a
 * packed struct, or plain text are equally fine. Applications layer their own
 * schema and parsing on top.
 *
 * Typical uses: per-unit configuration keyed by serial number, calibration
 * blobs, retained credentials, small data captures. The management ROS service
 * added in a later phase is a thin wrapper over one `FileStore`.
 *
 * Writes go through `Volume::write_file()`, so they are atomic at the file
 * level and incur the flash-program stall documented in `platform/flash.hpp`.
 */

#pragma once

#include "rosette/storage/volume.hpp"

#include <stddef.h>
#include <stdint.h>

namespace rosette::storage {

/// Longest key a `FileStore` accepts, excluding the null terminator.
inline constexpr uint8_t kFileStoreKeyMax = 40u;

/**
 * @brief Outcome of a `FileStore` operation.
 */
enum class FileStoreStatus : uint8_t {
    ok = 0,
    not_mounted,   ///< The backing volume is not available.
    bad_key,       ///< The key is empty, too long, or has characters outside [A-Za-z0-9._-].
    too_large,     ///< The blob exceeds the store's configured size limit.
    not_found,     ///< No blob is stored under that key.
    io_error,      ///< The volume rejected the read or write.
};

/**
 * @brief Returns a short human-readable label for @p status.
 */
const char* file_store_status_string(FileStoreStatus status) noexcept;

/**
 * @brief Keyed blob store rooted at one directory.
 */
class FileStore {
public:
    /**
     * @brief Binds a store to a directory on @p volume.
     *
     * @param[in] volume Mounted volume; must outlive the store.
     * @param[in] dir Absolute directory path, for example `/motors` or `/cal`.
     * Created on first `put()`.
     * @param[in] extension Suffix appended to the key to form the filename
     * (include the dot), or empty for none.
     * @param[in] max_blob Largest blob `put()` will accept, in bytes.
     */
    FileStore(Volume& volume, const char* dir, const char* extension = "",
              size_t max_blob = 8192u) noexcept;

    /**
     * @brief Stores @p data under @p key, replacing any existing blob.
     *
     * @param[in] key Blob key.
     * @param[in] data Bytes to store (opaque).
     * @param[in] len Number of bytes.
     * @return `FileStoreStatus::ok` or the reason it failed.
     */
    FileStoreStatus put(const char* key, const void* data, size_t len) noexcept;

    /**
     * @brief Reads the blob stored under @p key.
     *
     * @param[in] key Blob key.
     * @param[out] out Destination buffer.
     * @param[in] cap Capacity of @p out.
     * @param[out] out_len Number of bytes written to @p out.
     * @return `FileStoreStatus::ok`, `not_found`, or an error. `too_large` when
     * the stored blob does not fit in @p cap.
     */
    FileStoreStatus get(const char* key, void* out, size_t cap, size_t& out_len) noexcept;

    /**
     * @brief Reports whether a blob is stored under @p key.
     */
    bool exists(const char* key) noexcept;

    /**
     * @brief Removes the blob stored under @p key.
     *
     * @return `FileStoreStatus::ok` (also when nothing was stored) or an error.
     */
    FileStoreStatus erase(const char* key) noexcept;

    /**
     * @brief Lists the keys currently in the store.
     *
     * The configured extension is stripped from each name.
     *
     * @param[out] keys Row-major buffer of `count` slots, each
     * `kFileStoreKeyMax + 1` bytes.
     * @param[in] count Number of slots available.
     * @return Number of keys written.
     */
    uint8_t list(char keys[][kFileStoreKeyMax + 1], uint8_t count) noexcept;

    /// The directory this store manages.
    const char* directory() const noexcept { return dir_; }

    /// The largest blob `put()` accepts.
    size_t max_blob() const noexcept { return max_blob_; }

private:
    /**
     * @brief Validates @p key and builds `<dir>/<key><ext>` in @p out.
     *
     * @param[in] key Candidate key.
     * @param[out] out Path buffer of at least `kMaxPath` bytes.
     * @return `true` when the key is valid and the path fit.
     */
    bool path_for(const char* key, char* out) noexcept;

    Volume& volume_;
    const char* dir_;
    const char* ext_;
    size_t ext_len_;
    size_t dir_len_;
    size_t max_blob_;
};

}  // namespace rosette::storage
