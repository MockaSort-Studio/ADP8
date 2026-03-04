# Simple Chatter Example

Two nodes, `node_alpha` and `node_beta`, chat over two DDS channels. One initiates, the other responds. Both typed, both generated.

The flow is: write IDL + YAML + BUILD → run `bazel build` → Bazel generates the C++ headers → write your tasks against them.

Source: [`core/examples/simple_chatter/`](../../core/examples/simple_chatter/)

---

## Messages

Each channel has its own IDL type. FastDDS compiles them to C++ at build time.

```idl
// messages/ChannelAMessage.idl
struct ChannelAMessage
{
    string content;
    unsigned long counter;
};
```

```idl
// messages/ChannelBMessage.idl
struct ChannelBMessage
{
    string content;
    unsigned long counter;
};
```

Two distinct types, two distinct topics. They happen to have the same shape — that's coincidence for the example sake, not a requirement.

---

## Declare topics (YAML)

Each node gets its own ports file. Each gets its own generated namespace.

```yaml
# node_alpha_ports.yml
subscriptions:
  - channel_b:
      type: ChannelBMessage
publications:
  - channel_a:
      type: ChannelAMessage
```

```yaml
# node_beta_ports.yml
subscriptions:
  - channel_a:
      type: ChannelAMessage
publications:
  - channel_b:
      type: ChannelBMessage
```

`cc_dds_ports` in the BUILD (see below) processes each YAML and generates five headers per node. The header names are `{bazel_target_name}_*.hpp`. With `name = "node_alpha_ports"` and `namespace = "alpha"`, you get:

```cpp
// node_alpha_ports_sub_ids.hpp
namespace alpha {
static constexpr char kChannelBTopicName[] = "ChannelBTopic";
} // namespace alpha

// node_alpha_ports_subscriptions.hpp
namespace alpha {
using Subscriptions = core::lifecycle::TopicList<
    core::communication::TopicSpec< ChannelBMessagePubSubType, kChannelBTopicName, 1 >
>;
} // namespace alpha

// node_alpha_ports_pub_ids.hpp
namespace alpha {
static constexpr char kChannelATopicName[] = "ChannelATopic";
} // namespace alpha

// node_alpha_ports_publications.hpp
namespace alpha {
using Publications = core::lifecycle::TopicList<
    core::communication::TopicSpec< ChannelAMessagePubSubType, kChannelATopicName, 1 >
>;
} // namespace alpha
```

Beta gets the same treatment under `namespace beta`. The DDS topic name strings — `"ChannelATopic"`, `"ChannelBTopic"` — are identical on both sides, which is how FastDDS matches publishers to subscribers.

---

## Declare parameters (YAML)

```yaml
# params.yaml
node_alpha_params:
  - porkify:
      type: bool
      value: true
```

`cc_parameters(name = "node_alpha_params")` generates `node_alpha_params_parameters.hpp`:

```cpp
// node_alpha_params_parameters.hpp
namespace gen {

struct PorkifyTag {};

using NodeAlphaParamsTable = core::utils::LookupTable<
    core::utils::TableItem< PorkifyTag, bool>
>;

struct NodeAlphaParamsGeneratedInitializer {
    static constexpr auto values = core::utils::TableDefaults<NodeAlphaParamsTable>(
        core::utils::Init{PorkifyTag{}, true}
    );
};

using NodeAlphaParamsProvider = core::lifecycle::ParametersProvider<
    NodeAlphaParamsTable, NodeAlphaParamsGeneratedInitializer>;

} // namespace gen
```

The root key (`node_alpha_params`) becomes the provider name prefix. The parameter name (`porkify`) becomes the tag (`PorkifyTag`). Accessed at compile time — wrong tag is a `static_assert`, not a runtime error.

---

## Implement the tasks

### NodeAlpha

Publishes on `channel_a`, subscribes on `channel_b`. Kicks off the conversation on first execute.

```cpp
// node_alpha.hpp
#include "core/lifecycle/dds_task.hpp"
#include "node_alpha_params_parameters.hpp"
#include "node_alpha_ports_publications.hpp"
#include "node_alpha_ports_subscriptions.hpp"

namespace simple_chatter {

class NodeAlpha : public core::lifecycle::DDSTask<alpha::Subscriptions,
                                                  alpha::Publications> {
 public:
  using DDSTask<alpha::Subscriptions, alpha::Publications>::DDSTask;

 protected:
  void Execute() override {
    auto in  = GetInputSource<alpha::kChannelBTopicName>();
    auto out = GetOutputSink<alpha::kChannelATopicName>();

    const bool porkify = params_.GetParameterValue<gen::PorkifyTag>();

    if (counter_ == 0) {
      ChannelAMessage msg;
      msg.content(porkify ? "Hello, pork alpha here! Hoink!" : "Hello from Alpha!");
      msg.counter(counter_++);
      out.Push(std::move(msg));
      return;
    }

    if (in.Empty()) return;

    const auto& received = in[0].data;
    ChannelAMessage response;
    response.content(porkify
        ? "Alpha Hoink #" + std::to_string(received.counter())
        : "Alpha ack #"   + std::to_string(received.counter()));
    response.counter(counter_++);
    out.Push(std::move(response));
  }

 private:
  gen::NodeAlphaParamsProvider params_;
  uint32_t counter_{0};
};

} // namespace simple_chatter
```

### NodeBeta

Subscribes on `channel_a`, publishes on `channel_b`. Just responds.

```cpp
// node_beta.hpp
#include "core/lifecycle/dds_task.hpp"
#include "node_beta_ports_publications.hpp"
#include "node_beta_ports_subscriptions.hpp"

namespace simple_chatter {

class NodeBeta : public core::lifecycle::DDSTask<beta::Subscriptions,
                                                 beta::Publications> {
 public:
  using DDSTask<beta::Subscriptions, beta::Publications>::DDSTask;

 protected:
  void Execute() override {
    auto in  = GetInputSource<beta::kChannelATopicName>();
    auto out = GetOutputSink<beta::kChannelBTopicName>();

    if (in.Empty()) return;

    const auto& received = in[0].data;
    ChannelBMessage response;
    response.content("Beta ack #" + std::to_string(received.counter()));
    response.counter(counter_++);
    out.Push(std::move(response));
  }

 private:
  uint32_t counter_{0};
};

} // namespace simple_chatter
```

`GetInputSource` and `GetOutputSink` take a topic name constant as a template parameter. Lookup is a compile-time FNV-1a hash. A wrong name is a `static_assert`, not a runtime crash.

---

## Wire up the application

```cpp
// main.cpp
#include "core/examples/simple_chatter/node_alpha.hpp"
#include "core/examples/simple_chatter/node_beta.hpp"
#include "core/lifecycle/dds_application.hpp"
#include "core/support/utils/lookup_table.hpp"

using namespace core::lifecycle;
using namespace core::utils;

using ChatterConfig =
    LookupTable<TableItem<simple_chatter::NodeAlpha, TaskSpec<100>>,
                TableItem<simple_chatter::NodeBeta,  TaskSpec<100>>>;

int main() {
  DDSAPPlication<ChatterConfig> app{"simple_chatter"};
  app.Start();          // starts the execution engine
  app.Run();            // blocks until SIGINT or SIGTERM
  return 0;
}
```

`TaskSpec<100>` means each node runs every 100 ms (10 Hz). Both tasks run on the same single-threaded executor — no shared state, no locks needed.

---

## BUILD

The example uses `cc_fastdds_types` once for the shared IDL types, then `cc_dds_ports` separately for each node. This avoids compiling the same IDL twice.

```python
# core/examples/simple_chatter/BUILD
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//core/generators:dds_ports_gen.bzl", "cc_dds_ports")
load("//core/generators:defs.bzl", "cc_parameters")
load("//core/generators:fastdds_types_gen.bzl", "cc_fastdds_types")

cc_fastdds_types(
    name = "chatter_types",
    idl_srcs = glob(["messages/*.idl"]),
)

# node alpha
cc_dds_ports(
    name = "node_alpha_ports",
    idls = glob(["messages/*.idl"]),
    namespace = "alpha",
    yaml_config = "node_alpha_ports.yml",
    deps = [
        ":chatter_types",
        "//core/communication:dds",
    ],
)

cc_parameters(
    name = "node_alpha_params",
    yaml_parameters = "params.yaml",
)

cc_library(
    name = "node_alpha",
    hdrs = ["node_alpha.hpp"],
    deps = [
        ":node_alpha_params",
        ":node_alpha_ports",
        "//core/lifecycle:dds",
    ],
)

# node beta
cc_dds_ports(
    name = "node_beta_ports",
    idls = glob(["messages/*.idl"]),
    namespace = "beta",
    yaml_config = "node_beta_ports.yml",
    deps = [
        ":chatter_types",
        "//core/communication:dds",
    ],
)

cc_library(
    name = "node_beta",
    hdrs = ["node_beta.hpp"],
    deps = [
        ":node_beta_ports",
        "//core/lifecycle:dds",
    ],
)

# launch both nodes
cc_binary(
    name = "simple_chatter",
    srcs = ["main.cpp"],
    deps = [
        ":node_alpha",
        ":node_beta",
        "//core/lifecycle:dds",
        "//core/support/utils",
    ],
)
```

When there's a single node, `cc_dds_components` is a shorthand that wraps both `cc_fastdds_types` and `cc_dds_ports` into one call. With multiple nodes sharing the same IDL types, the split shown above avoids redundant compilation.

Build and run:

```
bazel build //core/examples/simple_chatter:simple_chatter
bazel run   //core/examples/simple_chatter:simple_chatter
```

---

```cpp
// Hamlet 🐗 — this compiles. and runs.
```