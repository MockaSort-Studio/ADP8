#include <pybind11/pybind11.h>

#include <tuple>

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
  std::string ParticipantName() const {
    auto participant =
        core::communication::DDSContextProvider::Get().GetDomainParticipant();
    eprosima::fastdds::dds::DomainParticipantQos qos;
    participant->get_qos(qos);
    return qos.name().c_str();
  }

 protected:
  // No topics registered — nothing to do per step yet.
  void Execute() override {}
};

PYBIND11_MODULE(py_dds_bridge, module) {
  module.doc() = "The pybind11 extension of Javelina";
  pybind11::class_<PyDDSBridge>(module, "PyDDSBridge")
      .def(pybind11::init<const std::string&>(),
           pybind11::arg("participant_name"))
      .def("init", &PyDDSBridge::Init)
      .def("participant_name", &PyDDSBridge::ParticipantName);
}
