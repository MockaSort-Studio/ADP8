# Javelina-RT

FastDDS-based real-time communication and lifecycle framework. Not a full robotics middleware stack — that's the point.

## Why not ROS2

ROS2 is the default choice for robotics. We evaluated it and ruled it out for concrete reasons:

- **Not Bazel-friendly.** ROS2 is built around colcon and ament. There is no first-class Bazel support; integrating it into a Bazel monorepo means maintaining a parallel build system or relying on fragile wrappers.
- **Heavy dependency.** Using ROS2 for pub/sub and a scheduler pulls in a large ecosystem. Most of it is unused. The surface area for breakage grows with it.
- **Thick abstraction layers.** Customizing QoS, lifecycle management, or transport behavior requires working through multiple layers of framework abstractions. Advanced use cases — anything below the standard executor model — push back.
- **You still write the scheduler.** ROS2's executor doesn't remove the need to reason about scheduling; it adds a model you work around rather than with. Real-time constraints require understanding what's running when.
- **Same total effort.** Once you account for the friction above, building cleanly on top of FastDDS directly costs the same as mastering ROS2 internals. With the added benefit that you own what you built.

Javelina-RT provides **communication and lifecycle only**. Algorithms live outside it, in `applications/`. The framework does not define how you write your logic; it defines when it runs and what data it sees.

## What it is not

- Not a full robotics framework (no transforms, no sensor fusion, no logging infrastructure)
- Not a replacement for DDS itself — FastDDS does the wire protocol, Javelina wraps it cleanly
- Not stable API yet — it grows with ADP8 and gets extracted when ready to stand alone

## Architecture

```
  Your Application
         │
         ▼
  DDSApplication<Config>
  │  reads a LookupTable of Task → TaskSpec
  │  handles SIGINT / SIGTERM
         │
         ▼
  TasksManager
  │  registers tasks, owns the engine
         │
         ▼
  ExecutionEngine
  │  single background thread
  │  min-heap scheduler, condition_variable sleep
         │
         ▼ (calls ExecuteStep at period)
  DDSTask<Subs, Pubs>
  │  FillInputs → Execute → FlushOutputs
  │  typed access via GetInputSource / GetOutputSink
         │
         ▼
  DataEndpoint<Spec, Direction>
  │  SizeConstrainedQueue (task-local circular buffer)
  │  compile-time topic lookup via FNV-1a hash
         │
         ▼
  DDSContext  (domain participant, topic registry)
         │
         ▼
  FastDDS  (wire protocol, DDS transport)
```

## Subsystems

| Subsystem | Location | Responsibility |
|-----------|----------|----------------|
| [Lifecycle](lifecycle.md) | `core/lifecycle/` | Task scheduling, data endpoints, parameters |
| [Communication](communication.md) | `core/communication/` | FastDDS wrappers, topic specs, context |
| [Support](support.md) | `core/support/` | LookupTable, SizeConstrainedQueue, Visitor |
| [Generators](generators.md) | `core/generators/` | Python codegen from YAML → C++ headers |

## Keeping it lightweight

The data path avoids runtime overhead: topic lookup is a compile-time FNV-1a hash, `FillInputs`/`FlushOutputs` are fold expressions over endpoint tuples, parameters are tag-dispatched with no string comparison. `if constexpr` and template specialization are used throughout in place of runtime branching. The goal is a framework that contributes nothing measurable to task latency. Benchmarks and profiling data will follow.

---

```cpp
// Hamlet 🐗 — proudly AI-generated, human-reviewed
```
