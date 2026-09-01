/**
 * @file storage/file_store.cpp
 * @brief Implementation of the content-agnostic keyed blob store.
 *
 * Every call validates the key, composes `<dir>/<key><ext>`, and delegates the
 * bytes to `Volume`. `list()` reads the directory and strips the configured
 * extension so callers see keys, not filenames.
 */

#include "rosette/storage/file_store.hpp"

#include <string.h>

namespace rosette::storage {

namespace {

/**
 * @brief Returns whether @p c is allowed in a key.
 *
 * Keys become path components, so the set is restricted to characters that are
 * safe and portable: alphanumerics, dot, dash, underscore.
 */
bool key_char_ok(char c) noexcept
{
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
           (c >= 'a' && c <= 'z') || c == '.' || c == '-' || c == '_';
}

}  // namespace

const char* file_store_status_string(FileStoreStatus status) noexcept
{
    switch (status) {
    case FileStoreStatus::ok:          return "ok";
    case FileStoreStatus::not_mounted: return "storage volume not mounted";
    case FileStoreStatus::bad_key:     return "invalid key";
    case FileStoreStatus::too_large:   return "blob exceeds the size limit";
    case FileStoreStatus::not_found:   return "no blob stored under that key";
    case FileStoreStatus::io_error:    return "storage read or write failed";
    }
    return "unknown file store status";
}

FileStore::FileStore(Volume& volume, const char* dir, const char* extension,
                     size_t max_blob) noexcept
    : volume_(volume)
    , dir_(dir)
    , ext_((extension != nullptr) ? extension : "")
    , ext_len_(strlen(ext_))
    , dir_len_(strlen(dir))
    , max_blob_(max_blob)
{}

bool FileStore::path_for(const char* key, char* out) noexcept
{
    if (key == nullptr) {
        return false;
    }
    const size_t n = strnlen(key, kFileStoreKeyMax + 1u);
    if (n == 0u || n > kFileStoreKeyMax) {
        return false;
    }
    for (size_t i = 0; i < n; ++i) {
        if (!key_char_ok(key[i])) {
            return false;
        }
    }

    // dir + '/' + key + ext + NUL
    const size_t total = dir_len_ + 1u + n + ext_len_ + 1u;
    if (total > kMaxPath) {
        return false;
    }

    char* w = out;
    memcpy(w, dir_, dir_len_);
    w += dir_len_;
    *w++ = '/';
    memcpy(w, key, n);
    w += n;
    memcpy(w, ext_, ext_len_);
    w += ext_len_;
    *w = '\0';
    return true;
}

FileStoreStatus FileStore::put(const char* key, const void* data, size_t len) noexcept
{
    if (!volume_.ok()) {
        return FileStoreStatus::not_mounted;
    }
    char path[kMaxPath];
    if (!path_for(key, path)) {
        return FileStoreStatus::bad_key;
    }
    if (len > max_blob_) {
        return FileStoreStatus::too_large;
    }
    if (!volume_.make_dir(dir_)) {
        return FileStoreStatus::io_error;
    }
    return volume_.write_file(path, data, len) ? FileStoreStatus::ok
                                               : FileStoreStatus::io_error;
}

FileStoreStatus FileStore::get(const char* key, void* out, size_t cap, size_t& out_len) noexcept
{
    out_len = 0u;
    if (!volume_.ok()) {
        return FileStoreStatus::not_mounted;
    }
    char path[kMaxPath];
    if (!path_for(key, path)) {
        return FileStoreStatus::bad_key;
    }
    if (!volume_.exists(path)) {
        return FileStoreStatus::not_found;
    }
    const size_t n = volume_.read_file(path, out, cap);
    if (n == 0u) {
        // read_file returns 0 for both "empty" and "does not fit"; the exists()
        // check above rules out "missing", so treat 0 as a capacity failure.
        return FileStoreStatus::too_large;
    }
    out_len = n;
    return FileStoreStatus::ok;
}

bool FileStore::exists(const char* key) noexcept
{
    if (!volume_.ok()) {
        return false;
    }
    char path[kMaxPath];
    return path_for(key, path) && volume_.exists(path);
}

FileStoreStatus FileStore::erase(const char* key) noexcept
{
    if (!volume_.ok()) {
        return FileStoreStatus::not_mounted;
    }
    char path[kMaxPath];
    if (!path_for(key, path)) {
        return FileStoreStatus::bad_key;
    }
    return volume_.remove_file(path) ? FileStoreStatus::ok : FileStoreStatus::io_error;
}

uint8_t FileStore::list(char keys[][kFileStoreKeyMax + 1], uint8_t count) noexcept
{
    if (!volume_.ok() || count == 0u) {
        return 0u;
    }

    const uint8_t found = volume_.list_dir(dir_, &keys[0][0], kFileStoreKeyMax + 1u, count);
    if (ext_len_ == 0u) {
        return found;
    }

    uint8_t kept = 0u;
    for (uint8_t i = 0; i < found; ++i) {
        const char* name = keys[i];
        const size_t n = strnlen(name, kFileStoreKeyMax + 1u);
        if (n > ext_len_ && strcmp(name + n - ext_len_, ext_) == 0) {
            if (kept != i) {
                memmove(keys[kept], name, n - ext_len_);
            }
            keys[kept][n - ext_len_] = '\0';
            ++kept;
        }
    }
    return kept;
}

}  // namespace rosette::storage
