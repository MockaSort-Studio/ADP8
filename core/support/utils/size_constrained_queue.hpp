#ifndef CORE_SUPPORT_UTILS_SIZE_CONSTRAINED_QUEUE
#define CORE_SUPPORT_UTILS_SIZE_CONSTRAINED_QUEUE
#include <array>
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
    static_assert(N > 0, "SizeConstrainedQueue size must be greater than 0");

  public:
    SizeConstrainedQueue() = default;
    ~SizeConstrainedQueue() = default;

    template <typename U>
    void Push(U&& message)
    {
        static_assert(
            std::is_convertible_v<std::decay_t<U>, T>,
            "Pushing wrong type into SizeConstrainedQueue");

        std::lock_guard<std::mutex> lock(mtx_);

        buffer_[head_] = {Clock::now(), std::forward<U>(message)};

        head_ = (head_ + 1) % N;
        if (count_ < N)
        {
            count_++;
        }
    }

    std::optional<Sample<T>> GetSample()
    {
        std::lock_guard<std::mutex> lock(mtx_);

        if (count_ == 0)
            return std::nullopt;

        // The "Newest" is at (head_ - 1)
        // We handle the wrap-around with (+ N) % N
        head_ = (head_ + N - 1) % N;
        Sample<T> sample = std::move(buffer_[head_]);
        count_--;

        return sample;
    }
    bool Empty() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return count_ == 0;
    }

    constexpr std::size_t capacity() const { return N; }

  private:
    std::array<Sample<T>, N> buffer_;
    std::size_t head_ {};
    std::size_t count_ {};
    mutable std::mutex mtx_;
};
}  // namespace core::utils
#endif  // CORE_SUPPORT_UTILS_SIZE_CONSTRAINED_QUEUE
