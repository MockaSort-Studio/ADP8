#ifndef CORE_LIFECYCLE_DDS_TASK
#define CORE_LIFECYCLE_DDS_TASK

#include "core/lifecycle/data_endpoint.hpp"
#include "core/lifecycle/input_source.hpp"
#include "core/lifecycle/output_sink.hpp"
#include "core/lifecycle/task_interface.hpp"
namespace core::lifecycle {

/// @brief Base class for tasks that communicate over DDS.
///
/// Manages typed input and output @c DataEndpoint collections. On each
/// @c ExecuteStep() call the engine:
///   1. Drains all subscriber queues into the input buffers (@c FillInputs).
///   2. Calls @c Execute() — the user-provided algorithm.
///   3. Publishes all pending output buffers (@c FlushOutputs).
///
/// Access inputs via @c GetInputSource<TopicName>() and outputs via
/// @c GetOutputSink<TopicName>() inside @c Execute().
///
/// @tparam SubscriptionSpecs @c TopicList of @c TopicSpec types for subscribed topics.
/// @tparam PublicationSpecs  @c TopicList of @c TopicSpec types for published topics.
/// @tparam ContextTag        Tag type forwarded to @c Inputs_t / @c Outputs_t for
///                           compile-time DDS context selection. Defaults to @c void
///                           (global context), preserving backward compatibility.
template <typename SubscriptionSpecs, typename PublicationSpecs,
          typename ContextTag = void>
class DDSTask : public TaskInterface {
  using Subs = SubscriptionSpecs;
  using Pubs = PublicationSpecs;

 public:
  using TaskInterface::TaskInterface;
  virtual ~DDSTask() = default;

  void ExecuteStep() override {
    FillInputs();
    Execute();
    FlushOutputs();
  };

 protected:
  /// @brief User algorithm. Called between @c FillInputs and @c FlushOutputs.
  virtual void Execute() = 0;

  /// @brief Optional init hook. Called once before the first @c ExecuteStep(). No-op by default.
  virtual void Init() {}

  /// @brief Returns a read-only view of the input endpoint for @p TopicName.
  /// @tparam TopicName Compile-time string matching a subscribed @c TopicSpec::kName.
  template <const char* TopicName>
  inline auto GetInputSource() noexcept {
    return InputSource{get<TopicName>(inputs_)};
  }

  /// @brief Returns a write-only view of the output endpoint for @p TopicName.
  /// @tparam TopicName Compile-time string matching a published @c TopicSpec::kName.
  template <const char* TopicName>
  inline auto GetOutputSink() noexcept {
    return OutputSink{get<TopicName>(outputs_)};
  }

 private:
  inline void FillInputs() noexcept {
    std::apply([](auto&... input) constexpr { (input.Sync(), ...); }, inputs_);
  }
  inline void FlushOutputs() noexcept {
    std::apply([](auto&... output) constexpr { (output.Sync(), ...); },
               outputs_);
  }

  Inputs_t<Subs, ContextTag> inputs_;
  Outputs_t<Pubs, ContextTag> outputs_;
};

}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_DDS_TASK
