/**
 * @file proxy_pool.cpp
 * @brief ProxyPool free list and admission, compiled once for every proxy type and pool size.
 *
 * See rtps/proxy_pool.hpp. This file works on proxy indices and links only;
 * the proxies themselves belong to the ProxyPoolStorage that derives from
 * ProxyPool.
 */

#include "rosette/rtps/proxy_pool.hpp"

namespace rosette::rtps {

ProxyPool::ProxyPool(ProxyIndex* links, uint16_t capacity) noexcept
    : links_(links), capacity_(capacity), free_head_(0u), free_count_(capacity) {
    // Chain every proxy into the free list in index order.
    for (uint16_t i = 0u; i + 1u < capacity; ++i) {
        links_[i] = static_cast<ProxyIndex>(i + 1u);
    }

    links_[capacity - 1u] = kNoProxy;
}

bool ProxyPool::reserve(uint16_t n) noexcept {
    if (free_count_ - reserved_ < n) {
        return false;
    }

    reserved_ = static_cast<uint16_t>(reserved_ + n);
    return true;
}

void ProxyPool::unreserve(uint16_t n) noexcept {
    reserved_ = n < reserved_ ? static_cast<uint16_t>(reserved_ - n) : 0u;
}

bool ProxyPool::may_bind(const ProxyList& list) const noexcept {
    if (list.count >= list.cap || free_head_ == kNoProxy) {
        return false;
    }

    return list.count < list.floor || free_count_ > reserved_;
}

ProxyIndex ProxyPool::bind(ProxyList& list) noexcept {
    if (!may_bind(list)) {
        return kNoProxy;
    }

    // A list below its floor spends one of the proxies reserved for it.
    if (list.count < list.floor) {
        --reserved_;
    }

    const ProxyIndex index = free_head_;
    free_head_ = links_[index];
    --free_count_;
    links_[index] = kNoProxy;

    if (list.head == kNoProxy) {
        list.head = index;
    } else {
        ProxyIndex tail = list.head;
        while (links_[tail] != kNoProxy) {
            tail = links_[tail];
        }

        links_[tail] = index;
    }

    ++list.count;
    return index;
}

bool ProxyPool::release(ProxyList& list, ProxyIndex index) noexcept {
    ProxyIndex* at = &list.head;
    while (*at != kNoProxy && *at != index) {
        at = &links_[*at];
    }

    if (*at == kNoProxy) {
        return false;
    }

    *at = links_[index];
    links_[index] = free_head_;
    free_head_ = index;
    ++free_count_;
    --list.count;

    // Below its floor, the list is promised this proxy back.
    if (list.count < list.floor) {
        ++reserved_;
    }

    return true;
}

}  // namespace rosette::rtps
