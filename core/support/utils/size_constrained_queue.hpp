#ifndef CORE_SUPPORT_UTILS_SIZE_CONSTRAINED_QUEUE
#define CORE_SUPPORT_UTILS_SIZE_CONSTRAINED_QUEUE
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <mutex>
#include <optional>
namespace core::utils {

using Clock = std::chrono::steady_clock;
using Timestamp = std::chrono::time_point<Clock>;

template <typename T>
struct Sample
{
    Timestamp time_received;
    T data;
};

template <typename T, std::size_t N>
class SizeConstrainedQueue
{
    static_assert(
        N > 0 && (N & (N - 1)) == 0,
        "SizeConstrainedQueue size N must be a power of 2 for maximum performance.");

  public:
    SizeConstrainedQueue() = default;

    /**
     * @brief Aggressive lock-free read access.
     * Safe to call on single thread. Newest element will be on top of the queue (index 0)
     */
    inline const Sample<T>& operator[](size_t index) const noexcept
    {
        const size_t c = count_.load(std::memory_order_relaxed);
        const size_t h = head_.load(std::memory_order_relaxed);

        assert(index < c && "Out of bounds access!");
        return buffer_[(h - 1 - index) & (N - 1)];
    }

    template <typename U>
    void Push(U&& message)
    {
        static_assert(
            std::is_convertible_v<std::decay_t<U>, T>,
            "Pushing wrong type into SizeConstrainedQueue");

        // C++17: scoped_lock is preferred over lock_guard
        std::scoped_lock lock(mtx_);

        size_t h = head_.load(std::memory_order_relaxed);
        size_t c = count_.load(std::memory_order_relaxed);

        buffer_[h] = {Clock::now(), std::forward<U>(message)};

        head_.store((h + 1) & (N - 1), std::memory_order_relaxed);
        if (c < N)
        {
            count_.store(c + 1, std::memory_order_release);
        }
    }

    std::optional<Sample<T>> GetSample()
    {
        std::scoped_lock lock(mtx_);

        size_t c = count_.load(std::memory_order_relaxed);
        if (c == 0)
            return std::nullopt;

        size_t h = head_.load(std::memory_order_relaxed);

        h = (h - 1) & (N - 1);
        Sample<T> sample = std::move(buffer_[h]);

        head_.store(h, std::memory_order_relaxed);
        count_.store(c - 1, std::memory_order_release);

        return sample;
    }

    inline bool Empty() const noexcept { return Size() == 0; }
    inline size_t Size() const noexcept { return count_.load(std::memory_order_relaxed); }

    /**
     * @brief Bulk transfer from one queue to another.
     * Uses Acquire/Release semantics to ensure data visibility across threads.
     */
    inline void TransferTo(SizeConstrainedQueue<T, N>& other) noexcept
    {
        std::scoped_lock lock(mtx_, other.mtx_);

        size_t current_count = count_.load(std::memory_order_acquire);
        if (current_count == 0)
        {
            other.count_.store(0, std::memory_order_release);
            return;
        }

        other.buffer_ = std::move(this->buffer_);

        other.head_.store(
            head_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        other.count_.store(current_count, std::memory_order_release);

        this->count_.store(0, std::memory_order_release);
        this->head_.store(0, std::memory_order_relaxed);
    }

    constexpr std::size_t capacity() const noexcept { return N; }

  private:
    std::array<Sample<T>, N> buffer_;
    std::atomic<size_t> head_ {0};
    std::atomic<size_t> count_ {0};
    mutable std::mutex mtx_;
};
}  // namespace core::utils
#endif  // CORE_SUPPORT_UTILS_SIZE_CONSTRAINED_QUEUE
