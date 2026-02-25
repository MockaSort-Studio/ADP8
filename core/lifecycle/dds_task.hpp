#ifndef CORE_LIFECYCLE_DDS_TASK
#define CORE_LIFECYCLE_DDS_TASK

#include "core/lifecycle/input.hpp"
#include "core/lifecycle/output.hpp"
#include "core/lifecycle/task_interface.hpp"
namespace core::lifecycle {

template <typename SubscriptionSpecs, typename PublicationSpecs>
class DDSTask : public TaskInterface
{
    using Subs = SubscriptionSpecs;
    using Pubs = PublicationSpecs;

  public:
    using TaskInterface::TaskInterface;
    virtual ~DDSTask() = default;

    void ExecuteStep() override
    {
        FillInputs();
        Execute();
        FlushOutputs();
    };

  protected:
    virtual void Execute() = 0;
    virtual void Init() {}

    // aggressivily unrolling loops at compile time (inline and noexcept)
    // this return std::optional<Sample<T>>
    template <const char* TopicName>
    inline auto GetInput() noexcept
    {
        return get<TopicName>(inputs_).Get();
    }

    template <typename Type, const char* TopicName>
    inline void SetOutput(Type&& output) noexcept
    {
        get<TopicName>(outputs_).Set(std::forward<Type>(output));
    }

  private:
    inline void FillInputs() noexcept
    {
        std::apply([](auto&... input) constexpr { (input.Fill(), ...); }, inputs_);
    }
    inline void FlushOutputs() noexcept
    {
        std::apply([](auto&... output) constexpr { (output.Flush(), ...); }, outputs_);
    }

    Inputs_t<Subs> inputs_;
    Outputs_t<Pubs> outputs_;
};

}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_DDS_TASK
