# Javelina-RT — Humble Desert Pork

---

## What's in this version

### Run periodic tasks over DDS

Define a task as a C++ class, declare its frequency, and the framework handles when it runs. Each task has typed subscriptions and publications declared at compile time. Data flows in before `Execute()` and out after — no manual pub/sub calls, no scheduler logic in application code.

### Typed publish/subscribe

Topics are declared with a type, a name, and a queue depth. Type mismatches and unknown topic names are caught at compile time. Incoming samples are timestamped and accessible in reverse-chronological order. The data path has no runtime string comparison.

### Typed parameters with static defaults

Parameters are declared as tag types with typed defaults. A task inherits a `ParametersProvider` and calls `GetParameterValue<Tag>()` — returns a scalar or an array, depending on the declaration. No string keys, no dynamic lookup.

### Declare topics and parameters in YAML, get C++ headers

Topics and parameters are declared in YAML. The generator produces typed C++ headers: topic name constants, `TopicList` type aliases, and a fully instantiated `ParametersProvider` with defaults. BUILD integration is a one-liner macro.

---

## What comes next

### Real-time scheduling

`ExecutionEngine` is a cooperative scheduler — a single background thread sleeping on a `condition_variable`. It's the equivalent of a ROS2 timer callback: adequate for most use cases, not hard real-time. Integration with [cactus-rt](https://github.com/cactusdynamics/cactus-rt) is the path to actual real-time guarantees: SCHED_FIFO, CPU isolation, and latency tracking.

### Parameter updates over the network

`SetParameterValue` is intentionally protected. The hook for a parameter server is in place; the transport is not. A future version will allow parameters to be updated at runtime over a lightweight RPC channel — without restarting the application.

### DDS context thread safety

`DDSContextProvider` is safe under the current assumption of single-threaded application startup. Before extraction as a standalone library, the contract needs to be either enforced explicitly or the implementation made thread-safe.


---

```cpp
// Hamlet 🐗 — listed the features, didn't pick the codename, would've picked a better one
```
