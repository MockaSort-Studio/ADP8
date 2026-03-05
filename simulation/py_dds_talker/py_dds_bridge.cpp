#include <pybind11/pybind11.h>

#include <string>
#include <tuple>
#include <unordered_map>

#include "core/communication/dds_context.hpp"
#include "core/lifecycle/dds_task.hpp"

// No topics yet — they will be registered in later steps.
using PyDDSBridgeBase = core::lifecycle::DDSTask<std::tuple<>, std::tuple<>>;

class PyDDSBridge : public PyDDSBridgeBase {
 public:
  explicit PyDDSBridge(const std::string& participant_name)
      : PyDDSBridgeBase(participant_name) {
    core::communication::DDSContextProvider::SetName(participant_name);
  }

  // Creates the DDS DomainParticipant. Must be called before any pub/sub.
  void Init() { core::communication::DDSContextProvider::Get(); }

  // Returns the name the live DomainParticipant was created with.
  [[nodiscard]] std::string ParticipantName() const {
    auto participant =
        core::communication::DDSContextProvider::Get().GetDomainParticipant();
    eprosima::fastdds::dds::DomainParticipantQos qos;
    participant->get_qos(qos);
    return qos.name().c_str();
  }

  // Calls start() on the subscriber and stores it under topic_name.
  void RegisterInput(const std::string& topic_name,
                     pybind11::object subscriber) {
    subscriber.attr("start")(topic_name);
    inputs_[topic_name] = std::move(subscriber);
  }

  // Calls start() on the publisher and stores it under topic_name.
  void RegisterOutput(const std::string& topic_name,
                      pybind11::object publisher) {
    publisher.attr("start")(topic_name);
    outputs_[topic_name] = std::move(publisher);
  }

 protected:
  // No topics registered at the DDSTask level — Python owns the pub/sub
  // objects.
  void Execute() override {}

 private:
  std::unordered_map<std::string, pybind11::object> inputs_;
  std::unordered_map<std::string, pybind11::object> outputs_;
};

PYBIND11_MODULE(py_dds_bridge, module) {
  module.doc() = "The pybind11 extension of Javelina";
  pybind11::class_<PyDDSBridge>(module, "PyDDSBridge")
      .def(pybind11::init<const std::string&>(),
           pybind11::arg("participant_name"))
      .def("init", &PyDDSBridge::Init)
      .def("participant_name", &PyDDSBridge::ParticipantName)
      .def("register_input", &PyDDSBridge::RegisterInput,
           pybind11::arg("topic_name"), pybind11::arg("subscriber"))
      .def("register_output", &PyDDSBridge::RegisterOutput,
           pybind11::arg("topic_name"), pybind11::arg("publisher"));
}
