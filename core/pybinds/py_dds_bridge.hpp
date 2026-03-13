#ifndef CORE_PYBINDS_PY_DDS_BRIDGE
#define CORE_PYBINDS_PY_DDS_BRIDGE
#include <string>
#include <vector>

#include "core/communication/dds_context.hpp"
#include "core/lifecycle/data_endpoint.hpp"
#include "core/lifecycle/input_source.hpp"
#include "core/lifecycle/output_sink.hpp"

template <typename SubscriptionSpecs, typename PublicationSpecs>
class PyDDSBridge {
  using Subs = SubscriptionSpecs;
  using Pubs = PublicationSpecs;

 public:
  explicit PyDDSBridge(
      const std::string& /*participant_name*/)  // @TODO: new DDSContextProvider
                                                // API derives the name from the
                                                // tag type, not a runtime
                                                // string. To revisit this once
                                                // the rework is done.
  {
    core::communication::DDSContextProvider<>::Get();  // trigger singleton init
  }

  // Returns the name the live DomainParticipant was created with.
  [[nodiscard]] std::string ParticipantName() const {
    auto participant =
        core::communication::DDSContextProvider<>::Get().GetDomainParticipant();
    eprosima::fastdds::dds::DomainParticipantQos qos;
    participant->get_qos(qos);
    return qos.name().c_str();
  }

  template <const char* TopicName>
  auto GetInputs() {
    auto source =
        core::lifecycle::InputSource{core::lifecycle::get<TopicName>(inputs_)};
    std::vector<typename decltype(source)::T> result;
    result.reserve(source.Size());
    for (size_t i = 0; i < source.Size(); ++i) {
      result.push_back(source[i].data);
    }
    return result;
  }

  template <const char* TopicName, typename MsgType>
  inline void PushOutput(MsgType&& message) noexcept {
    auto sink =
        core::lifecycle::OutputSink{core::lifecycle::get<TopicName>(outputs_)};
    sink.Push(std::forward<MsgType>(message));
  }

  inline void FillInputs() noexcept {
    std::apply([](auto&... input) constexpr { (input.Sync(), ...); }, inputs_);
  }
  inline void FlushOutputs() noexcept {
    std::apply([](auto&... output) constexpr { (output.Sync(), ...); },
               outputs_);
  }

 private:
  core::lifecycle::Inputs_t<Subs> inputs_;
  core::lifecycle::Outputs_t<Pubs> outputs_;
};

#endif  // CORE_PYBINDS_PY_DDS_BRIDGE
