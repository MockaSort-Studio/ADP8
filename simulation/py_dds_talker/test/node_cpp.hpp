#ifndef SIMULATION_PY_DDS_TALKER_TEST_NODE_CPP
#define SIMULATION_PY_DDS_TALKER_TEST_NODE_CPP

// proudly AI-generated, human-reviewed

#include <iostream>
#include <string>

#include "core/lifecycle/dds_task.hpp"
#include "node_cpp_ports_publications.hpp"
#include "node_cpp_ports_subscriptions.hpp"

namespace py_dds_talker {

// Publishes on channel_a, subscribes on channel_b.
// Kicks off the conversation on first Execute(), then echoes responses.
// Mirrors NodeAlpha from core/examples/simple_chatter — the Python
// DdsTalker plays the NodeBeta role.
class NodeCpp
    : public core::lifecycle::DDSTask<cpp::Subscriptions, cpp::Publications> {
 public:
  using DDSTask<cpp::Subscriptions, cpp::Publications>::DDSTask;

 protected:
  void Execute() override {
    auto in = GetInputSource<cpp::kChannelBTopicName>();
    auto out = GetOutputSink<cpp::kChannelATopicName>();

    if (counter_ == 0) {
      ChannelMessage msg;
      msg.content("Hello from C++ node!");
      msg.counter(counter_++);
      std::cout << "[C++  -> channel_a] " << msg.content() << " #"
                << msg.counter() << "\n";
      out.Push(std::move(msg));
      return;
    }

    if (in.Empty()) return;

    const auto& received = in[0].data;
    std::cout << "[C++  <- channel_b] " << received.content() << " #"
              << received.counter() << "\n";

    ChannelMessage response;
    response.content("C++ ack #" + std::to_string(received.counter()));
    response.counter(counter_++);
    std::cout << "[C++  -> channel_a] " << response.content() << " #"
              << response.counter() << "\n";
    out.Push(std::move(response));
  }

 private:
  uint32_t counter_{0};
};

}  // namespace py_dds_talker

#endif  // SIMULATION_PY_DDS_TALKER_TEST_NODE_CPP
