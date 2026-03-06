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

  // todo: delete register and use something like
  //      Inputs_t<Subs> inputs_;
  //      Outputs_t<Pubs> outputs_;
  // check docstrings,but in sostanza fanno da soli la register

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

  // Drains all samples from the registered subscriber for topic_name.
  // Returns a Python list of message objects (same type as the subscriber
  // yields). Raises KeyError if topic_name was not registered.
  pybind11::object GetInputs(const std::string& topic_name) {
    auto it = inputs_.find(topic_name);
    if (it == inputs_.end()) throw pybind11::key_error(topic_name);
    return it->second.attr("drain")();
  }

  // Publishes message to the registered publisher for topic_name.
  // Returns True if the write succeeded (publisher is matched and write ok).
  // Raises KeyError if topic_name was not registered.
  bool PushOutput(const std::string& topic_name, pybind11::object message) {
    auto it = outputs_.find(topic_name);
    if (it == outputs_.end()) throw pybind11::key_error(topic_name);
    return it->second.attr("publish")(message).cast<bool>();
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
           pybind11::arg("topic_name"), pybind11::arg("publisher"))
      .def("get_inputs", &PyDDSBridge::GetInputs, pybind11::arg("topic_name"))
      .def("push_output", &PyDDSBridge::PushOutput, pybind11::arg("topic_name"),
           pybind11::arg("message"));
}
