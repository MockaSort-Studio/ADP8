#ifndef CORE_LIFECYCLE_DDS_TASK
#define CORE_LIFECYCLE_DDS_TASK

// #include "core/lifecycle/input.hpp"
// #include "core/lifecycle/output.hpp"
#include "core/lifecycle/data_endpoint.hpp"
#include "core/lifecycle/input_source.hpp"
#include "core/lifecycle/output_sink.hpp"
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

    // // this return std::optional<Sample<T>>
    template <const char* TopicName>
    inline auto GetInputSource() noexcept
    {
        return InputSource {get<TopicName>(inputs_)};
    }

    template <const char* TopicName>
    inline auto GetOutputSink() noexcept
    {
        OutputSink {get<TopicName>(outputs_)};
    }

  private:
    inline void FillInputs() noexcept
    {
        std::apply([](auto&... input) constexpr { (input.Sync(), ...); }, inputs_);
    }
    inline void FlushOutputs() noexcept
    {
        std::apply([](auto&... output) constexpr { (output.Sync(), ...); }, outputs_);
    }

    Inputs_t<Subs> inputs_;
    Outputs_t<Pubs> outputs_;
};

}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_DDS_TASK
