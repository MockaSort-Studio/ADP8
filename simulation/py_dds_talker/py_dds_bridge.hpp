#ifndef SIMULATION_PY_DDS_TALKER_PY_DDS_BRIDGE
#define SIMULATION_PY_DDS_TALKER_PY_DDS_BRIDGE
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
  explicit PyDDSBridge(const std::string& participant_name) {
    core::communication::DDSContextProvider::SetName(participant_name);
  }

  void Init() {}

  // Returns the name the live DomainParticipant was created with.
  [[nodiscard]] std::string ParticipantName() const {
    auto participant =
        core::communication::DDSContextProvider::Get().GetDomainParticipant();
    eprosima::fastdds::dds::DomainParticipantQos qos;
    participant->get_qos(qos);
    return qos.name().c_str();
  }

  // todo: delete register and use something like
  //      Inputs_t<Subs> inputs_;
  //      Outputs_t<Pubs> outputs_;
  // check docstrings,but in sostanza fanno da soli la register

  // Drains all samples from the registered subscriber for topic_name.
  // Returns a Python list of message objects (same type as the subscriber
  // yields). Raises KeyError if topic_name was not registered.

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

  // Publishes message to the registered publisher for topic_name.
  // Returns True if the write succeeded (publisher is matched and write ok).
  // Raises KeyError if topic_name was not registered.
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

// PYBIND11_MODULE(py_dds_bridge, module) {
//   module.doc() = "The pybind11 extension of Javelina";
//   pybind11::class_<PyDDSBridge>(module, "PyDDSBridge")
//       .def(pybind11::init<const std::string&>(),
//            pybind11::arg("participant_name"))
//       .def("participant_name", &PyDDSBridge::ParticipantName)
//       .def("get_inputs", &PyDDSBridge::GetInputs,
//       pybind11::arg("topic_name")) .def("push_output",
//       &PyDDSBridge::PushOutput, pybind11::arg("topic_name"),
//            pybind11::arg("message"));
// }

#endif  // SIMULATION_PY_DDS_TALKER_PY_DDS_BRIDGE
