/**
 * @file support/inplace_function.hpp
 * @brief A fixed-capacity, non-allocating callable — `std::function` without the heap.
 *
 * The entity wrappers store user callbacks (service handlers, timer callbacks)
 * without being templated on the callback's type, so an application can hand
 * them a capturing lambda and still keep the wrapper as a plain, movable member
 * of a module. `InplaceFunction` holds the callable in an inline buffer; a
 * callable that does not fit fails a `static_assert` at the call site rather
 * than silently allocating.
 *
 * Constraints: the target must be nothrow-move-constructible and trivially
 * destructible-or-close-enough for MCU use — captures of pointers, references,
 * small PODs, and `this` all qualify. Copying is not supported; move only.
 */

#pragma once

#include <stddef.h>

#include <new>
#include <type_traits>
#include <utility>

namespace rosette {

template<typename Signature, size_t Capacity = 16u>
class InplaceFunction;

/**
 * @brief Owns one callable of at most @p Capacity bytes with signature
 * `R(Args...)`.
 */
template<typename R, typename... Args, size_t Capacity>
class InplaceFunction<R(Args...), Capacity> {
public:
    InplaceFunction() noexcept = default;

    /**
     * @brief Stores @p fn by move.
     *
     * @tparam F Callable type; must fit the inline buffer and be nothrow-move
     * constructible.
     * @param[in] fn Callable to adopt.
     */
    template<typename F,
             typename = std::enable_if_t<
                 !std::is_same_v<std::decay_t<F>, InplaceFunction> &&
                 std::is_invocable_r_v<R, std::decay_t<F>&, Args...>>>
    InplaceFunction(F&& fn) noexcept
    {
        using Fn = std::decay_t<F>;
        static_assert(sizeof(Fn) <= Capacity,
                      "callable too large for InplaceFunction — raise Capacity");
        static_assert(std::is_nothrow_move_constructible_v<Fn>,
                      "InplaceFunction target must be nothrow-move-constructible");
        ::new (static_cast<void*>(&storage_)) Fn(std::forward<F>(fn));
        invoke_ = [](void* self, Args... args) -> R {
            return (*static_cast<Fn*>(self))(std::forward<Args>(args)...);
        };
        move_ = [](void* dst, void* src) noexcept {
            ::new (dst) Fn(std::move(*static_cast<Fn*>(src)));
            static_cast<Fn*>(src)->~Fn();
        };
        destroy_ = [](void* self) noexcept { static_cast<Fn*>(self)->~Fn(); };
    }

    InplaceFunction(InplaceFunction&& other) noexcept { move_from(other); }

    InplaceFunction& operator=(InplaceFunction&& other) noexcept
    {
        if (this != &other) {
            reset();
            move_from(other);
        }
        return *this;
    }

    InplaceFunction(const InplaceFunction&) = delete;
    InplaceFunction& operator=(const InplaceFunction&) = delete;

    ~InplaceFunction() { reset(); }

    /**
     * @brief Reports whether a callable is stored.
     */
    explicit operator bool() const noexcept { return invoke_ != nullptr; }

    /**
     * @brief Invokes the stored callable.
     *
     * Calling an empty `InplaceFunction` is undefined; guard with `operator bool`
     * where the slot may be unset.
     */
    R operator()(Args... args) const
    {
        return invoke_(const_cast<void*>(static_cast<const void*>(&storage_)),
                       std::forward<Args>(args)...);
    }

    /**
     * @brief Destroys the stored callable and returns to the empty state.
     */
    void reset() noexcept
    {
        if (destroy_ != nullptr) {
            destroy_(&storage_);
        }
        invoke_ = nullptr;
        move_ = nullptr;
        destroy_ = nullptr;
    }

private:
    void move_from(InplaceFunction& other) noexcept
    {
        if (other.invoke_ != nullptr) {
            other.move_(&storage_, &other.storage_);
            invoke_ = other.invoke_;
            move_ = other.move_;
            destroy_ = other.destroy_;
            other.invoke_ = nullptr;
            other.move_ = nullptr;
            other.destroy_ = nullptr;
        }
    }

    using Storage = std::aligned_storage_t<Capacity, alignof(void*)>;

    Storage storage_{};
    R (*invoke_)(void*, Args...) = nullptr;
    void (*move_)(void*, void*) noexcept = nullptr;
    void (*destroy_)(void*) noexcept = nullptr;
};

}  // namespace rosette
