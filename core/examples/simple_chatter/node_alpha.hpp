#ifndef CORE_EXAMPLES_SIMPLE_CHATTER_NODE_ALPHA
#define CORE_EXAMPLES_SIMPLE_CHATTER_NODE_ALPHA

#include <iostream>
#include <string>

#include "node_alpha_params_parameters.hpp"
#include "node_alpha_ports_task_base.hpp"

namespace simple_chatter {

// Publishes on channel_a, subscribes on channel_b.
// if new (counter = 0), kick off chat
class NodeAlpha : public alpha::NodeAlphaTaskBase {
 public:
  using NodeAlphaTaskBase::NodeAlphaTaskBase;

 protected:
  void Execute() override {
    auto in = GetInputSource<alpha::kChannelBTopicName>();
    auto out = GetOutputSink<alpha::kChannelATopicName>();

    const bool porkify = params_.GetParameterValue<gen::PorkifyTag>();

    if (counter_ == 0) {
      ChannelAMessage msg;
      msg.content(porkify ? "Hello, pork alpha here! Hoink!"
                          : "Hello from Alpha!");
      msg.counter(counter_++);
      std::cout << "[Alpha -> channel_a] " << msg.content() << " #"
                << msg.counter() << "\n";
      out.Push(std::move(msg));
      return;
    }

    if (in.Empty()) return;

    const auto& received = in[0].data;
    std::cout << "[Alpha <- channel_b] " << received.content() << " #"
              << received.counter() << "\n";

    ChannelAMessage response;
    response.content(porkify
                         ? "Alpha Hoink #" + std::to_string(received.counter())
                         : "Alpha ack #" + std::to_string(received.counter()));
    response.counter(counter_++);
    std::cout << "[Alpha -> channel_a] " << response.content() << " #"
              << response.counter() << "\n";
    out.Push(std::move(response));
  }

 private:
  gen::NodeAlphaParamsProvider params_;
  uint32_t counter_{0};
};

}  // namespace simple_chatter

#endif  // CORE_EXAMPLES_SIMPLE_CHATTER_NODE_ALPHA
