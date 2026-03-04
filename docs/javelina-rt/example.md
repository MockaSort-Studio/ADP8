# End-to-End Example

A velocity controller task: subscribes to IMU readings, publishes velocity commands, reads typed parameters.

The flow is: write IDL + YAML + BUILD → run `bazel build` → Bazel generates the C++ headers → write your task against them.

---

## Messages

IDL files define the data structures. FastDDS compiles them to C++ at build time.

```idl
// messages/ImuReading.idl
struct ImuReading
{
    double acc_x;
    double acc_y;
    double acc_z;
    double gyro_z;
};
```

```idl
// messages/VelocityCommand.idl
struct VelocityCommand
{
    double linear;
    double angular;
};
```

---

## Declare topics (YAML)

```yaml
# controller/ports.yaml
subscriptions:
  - imu_data:
      type: ImuReading
      queue_size: 5

publications:
  - velocity_cmd:
      type: VelocityCommand
```

`cc_dds_components` in the BUILD (see below) takes this YAML and the IDL files and produces — at `bazel build` time — the following headers under the `controller_ports` sub-target:

```cpp
// controller_ports_sub_ids.hpp
namespace gen {
static constexpr char kImuDataTopicName[] = "ImuDataTopic";
} // namespace gen

// controller_ports_subscriptions.hpp
namespace gen {
using Subscriptions = core::lifecycle::TopicList<
    core::communication::TopicSpec< ImuReadingPubSubType, kImuDataTopicName, 5 >
>;
} // namespace gen

// controller_ports_pub_ids.hpp
namespace gen {
static constexpr char kVelocityCmdTopicName[] = "VelocityCmdTopic";
} // namespace gen

// controller_ports_publications.hpp
namespace gen {
using Publications = core::lifecycle::TopicList<
    core::communication::TopicSpec< VelocityCommandPubSubType, kVelocityCmdTopicName, 1 >
>;
} // namespace gen
```

The file names are `{bazel_target_name}_*.hpp`. The target is `controller_ports` because `cc_dds_components(name="controller")` internally creates a `controller_ports` sub-target for the spec headers.

---

## Declare parameters (YAML)

```yaml
# controller/params.yaml
controller_params:
  - gain:
      type: float
      value: 1.5
  - max_speed:
      type: float
      value: 2.0
  - enabled:
      type: bool
      value: true
```

`cc_parameters(name="controller_params")` generates — at `bazel build` time — one header named `controller_params_parameters.hpp`:

```cpp
// controller_params_parameters.hpp
namespace gen {

struct GainTag {};
struct MaxSpeedTag {};
struct EnabledTag {};

using ControllerParamsTable = core::utils::LookupTable<
    core::utils::TableItem< GainTag,     float>,
    core::utils::TableItem< MaxSpeedTag, float>,
    core::utils::TableItem< EnabledTag,  bool>
>;

struct ControllerParamsGeneratedInitializer {
    static constexpr auto values = core::utils::TableDefaults<ControllerParamsTable>(
        core::utils::Init{GainTag{},     1.5f},
        core::utils::Init{MaxSpeedTag{}, 2.0f},
        core::utils::Init{EnabledTag{},  true}
    );
};

using ControllerParamsProvider = core::lifecycle::ParametersProvider<
    ControllerParamsTable, ControllerParamsGeneratedInitializer>;

} // namespace gen
```

---

## Implement the task

The generated header names come from the Bazel target names. Include them directly:

```cpp
// controller/controller_task.hpp
#pragma once

#include "controller_params_parameters.hpp"    // gen::ControllerParamsProvider
#include "controller_ports_publications.hpp"   // gen::Publications
#include "controller_ports_pub_ids.hpp"        // gen::kVelocityCmdTopicName
#include "controller_ports_sub_ids.hpp"        // gen::kImuDataTopicName
#include "controller_ports_subscriptions.hpp"  // gen::Subscriptions
#include "core/lifecycle/dds_task.hpp"

class ControllerTask
    : public core::lifecycle::DDSTask<gen::Subscriptions, gen::Publications> {
 public:
  using DDSTask::DDSTask;

 protected:
  void Execute() override {
    auto imu = GetInputSource<gen::kImuDataTopicName>();
    if (imu.Empty()) return;

    // imu[0] is the newest sample; imu[0].data is ImuReading
    const auto& reading = imu[0].data;

    const float gain = params_.GetParameterValue<gen::GainTag>();
    const float max_speed = params_.GetParameterValue<gen::MaxSpeedTag>();

    VelocityCommand cmd{};
    cmd.linear = std::clamp(gain * reading.acc_x, -max_speed, max_speed);
    cmd.angular = gain * reading.gyro_z;

    auto out = GetOutputSink<gen::kVelocityCmdTopicName>();
    out.Push(std::move(cmd));
  }

 private:
  gen::ControllerParamsProvider params_;
};
```

`GetInputSource` and `GetOutputSink` take a topic name constant as a template parameter. Lookup is a compile-time FNV-1a hash. A wrong name is a `static_assert`, not a runtime crash.

---

## Wire up the application

```cpp
// controller/main.cpp
#include "controller_task.hpp"
#include "core/lifecycle/dds_application.hpp"
#include "core/support/utils/lookup_table.hpp"

using AppConfig = core::utils::LookupTable<
    core::utils::TableItem<ControllerTask, core::lifecycle::TaskSpec<20>>  // 50 Hz
>;

int main() {
  core::lifecycle::DDSAPPlication<AppConfig> app("controller_node");
  app.Run();  // blocks until SIGINT or SIGTERM
  return 0;
}
```

---

## BUILD

```python
# controller/BUILD
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//core/generators:defs.bzl", "cc_dds_components", "cc_parameters")

# Compiles the IDL files (FastDDS types) AND generates the port spec headers.
# Produces two sub-targets used downstream:
#   :controller_types  — ImuReadingPubSubType, VelocityCommandPubSubType
#   :controller_ports  — controller_ports_*.hpp (topic ids + specs)
cc_dds_components(
    name = "controller",
    idls = ["//messages:controller_idl"],
    ports_yaml = "ports.yaml",
    namespace = "gen",
)

# Generates controller_params_parameters.hpp
cc_parameters(
    name = "controller_params",
    yaml_parameters = "params.yaml",
    namespace = "gen",
)

cc_library(
    name = "controller_task",
    hdrs = ["controller_task.hpp"],
    deps = [
        ":controller_params",
        ":controller_ports",
        "//core/lifecycle:dds",
    ],
)

cc_binary(
    name = "controller_node",
    srcs = ["main.cpp"],
    deps = [":controller_task"],
)
```

Build and run:

```
bazel build //controller:controller_node
bazel run //controller:controller_node
```

---

```cpp
// Hamlet 🐗 — this compiles. probably.
```
