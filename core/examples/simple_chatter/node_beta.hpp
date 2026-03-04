#ifndef CORE_EXAMPLES_SIMPLE_CHATTER_NODE_BETA
#define CORE_EXAMPLES_SIMPLE_CHATTER_NODE_BETA

#include <iostream>
#include <string>

#include "core/lifecycle/dds_task.hpp"
#include "node_beta_ports_publications.hpp"
#include "node_beta_ports_subscriptions.hpp"

namespace simple_chatter {

// Subscribes on channel_a, publishes on channel_b.
class NodeBeta
    : public core::lifecycle::DDSTask<beta::Subscriptions, beta::Publications> {
 public:
  using DDSTask<beta::Subscriptions, beta::Publications>::DDSTask;

 protected:
  void Execute() override {
    auto in = GetInputSource<beta::kChannelATopicName>();
    auto out = GetOutputSink<beta::kChannelBTopicName>();

    if (in.Empty()) return;

    const auto& received = in[0].data;
    std::cout << "[Beta  <- channel_a] " << received.content() << " #"
              << received.counter() << "\n";

    ChannelBMessage response;
    response.content("Beta ack #" + std::to_string(received.counter()));
    response.counter(counter_++);
    std::cout << "[Beta  -> channel_b] " << response.content() << " #"
              << response.counter() << "\n";
    out.Push(std::move(response));
  }

 private:
  uint32_t counter_{0};
};

}  // namespace simple_chatter

#endif  // CORE_EXAMPLES_SIMPLE_CHATTER_NODE_BETA
