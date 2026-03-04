# Lifecycle

`core/lifecycle/` — Task scheduling, data flow, and parameters.

---

## ExecutionEngine

`execution_engine.hpp` / `execution_engine.cpp`

A single background thread that runs scheduled callbacks at fixed periods. Internally a min-heap priority queue (`std::priority_queue` with `std::greater<Job>`) ordered by next-fire time.

```cpp
ExecutionEngine engine;
engine.Start();
engine.Schedule(std::chrono::milliseconds(10), []() { /* 100 Hz callback */ });
engine.Stop();  // also called in destructor
```

The scheduler is self-rescheduling: each `Job` captures a `shared_ptr` to the task and re-enqueues itself after execution. The thread sleeps via `condition_variable::wait_until(next_job_time)` — no spin, no polling.

Copy and move are deleted. One engine per instance.

---

## TaskInterface

`task_interface.hpp`

Minimal base class. A task has a name and one virtual method:

```cpp
class TaskInterface {
 public:
  explicit TaskInterface(std::string name);
  virtual void ExecuteStep() = 0;
  const std::string& Name();
};
```

The framework calls `ExecuteStep()` at the scheduled frequency. Everything else is your problem.

---

## TasksManager

`tasks_manager.hpp`

Bridges `TaskInterface` subclasses and `ExecutionEngine`. Owns the engine via `unique_ptr`. Tasks are kept alive by `shared_ptr` in a register vector.

```cpp
TasksManager manager;
manager.AddTask<MyTask, 20>("my_task");  // 20 ms period = 50 Hz
manager.Start();
// ...
manager.Stop();
```

`AddTask<T, Ms>` is constrained: `T` must derive from `TaskInterface`, `Ms` must be positive. Checked at compile time.

**`TaskSpec`** is the config type for `DDSApplication`:

```cpp
template <std::chrono::milliseconds::rep Ms>
struct TaskSpec {
  static constexpr std::chrono::milliseconds::rep kFrequency = Ms;
};

using MySpec = TaskSpec<20>;  // 20 ms period
```

---

## DDSApplication

`dds_application.hpp`

Top-level entry point. Takes a `LookupTable` as its `ApplicationConfig` template parameter. Each entry maps a `TaskInterface` subclass to a `TaskSpec`.

```cpp
using AppConfig = core::utils::LookupTable<
    core::utils::TableItem<PlannerTask,  TaskSpec<20>>,
    core::utils::TableItem<ControlTask,  TaskSpec<10>>
>;

DDSAPPlication<AppConfig> app("my_participant");
app.Run();  // blocks until SIGINT or SIGTERM
```

On construction: registers SIGINT/SIGTERM handlers, sets the FastDDS participant name, builds the `TasksManager` by iterating the config table.

On `Run()`: blocks the main thread in a 100ms sleep loop until the signal flag is set.

On destruction: calls `tasks_manager_->Stop()`.

---

## DDSTask

`dds_task.hpp`

The base class for tasks that read from and write to DDS topics. Inherits `TaskInterface`. Parameterized by two `TopicList` types:

```cpp
template <typename SubscriptionSpecs, typename PublicationSpecs>
class DDSTask : public TaskInterface;
```

`ExecuteStep()` is sealed:

```
FillInputs()   — drains subscriber queues into DataEndpoints
Execute()      — your logic (pure virtual)
FlushOutputs() — publishes pending outputs via FastDDS
```

Access data inside `Execute()`:

```cpp
void Execute() override {
    auto imu = GetInputSource<kImuTopicName>();
    if (!imu.Empty()) {
        auto& sample = imu[0];  // newest
        // sample.data, sample.time_received

        auto out = GetOutputSink<kControlTopicName>();
        out.Push(ComputeControl(sample.data));
    }
}
```

`FillInputs` and `FlushOutputs` are fold expressions over the endpoint tuples — zero branching, zero virtual dispatch.

---

## DataEndpoint

`data_endpoint.hpp`

Templated on a `TopicSpec` and a `DataDirection` (In or Out). Holds a `SizeConstrainedQueue<T, N>` plus either a `DDSSubscriber` or `DDSPublisher` (the other member is `int` via `std::conditional_t` — no wasted allocation).

On construction: starts the subscriber or publisher.

`Sync()`:

- **In**: drains the subscriber queue into the local `SizeConstrainedQueue`
- **Out**: if the queue is non-empty, publishes `queue_[0]` (latest sample, since outputs are restricted to `kQueueSize == 1`)

`operator[]` gives reverse-chronological access: `[0]` is newest, `[N-1]` is oldest. Checked at compile time to be In-only.

**Topic lookup** uses a compile-time FNV-1a hash of the topic name string (see `type_traits.hpp`). `get<TopicName>(inputs_)` resolves to the correct tuple index at compile time — no string comparison at runtime.

---

## InputSource / OutputSink

`input_source.hpp` / `output_sink.hpp`

Read-only and write-only views over a `DataEndpoint`. Handed to `Execute()` via `GetInputSource<name>()` and `GetOutputSink<name>()`. Prevents tasks from writing to inputs or reading from outputs.

```cpp
// InputSource: read only
auto src = GetInputSource<kImuTopicName>();
src[0].data;      // newest sample
src.Size();
src.Empty();

// OutputSink: write only
auto sink = GetOutputSink<kControlTopicName>();
sink.Push(std::move(msg));
```

---

## ParametersProvider

`parameters_provider.hpp`

Mixin that gives a task typed, named parameters backed by a `LookupTable`. Single or array values, resolved at compile time via tag types.

```cpp
template <typename Params, typename ParamsDefaults>
class ParametersProvider {
 public:
  template <typename Tag>
  constexpr auto GetParameterValue() const;

 protected:
  template <typename Tag, typename... Args>
  void SetParameterValue(Args&&... args);
};
```

`collapse_tuple`: if the underlying `TableItem` has one value, returns it directly as a scalar. If it has multiple, returns `std::array<T, N>`. This is purely compile-time — no runtime branch.

Typical usage is through generated headers (see [Generators](generators.md)):

```cpp
class MyTask : public DDSTask<Subs, Pubs>,
               public gen::MyParamsProvider {
  void Execute() override {
    float gain = GetParameterValue<gen::GainTag>();
  }
};
```

`SetParameterValue` is protected — public mutation is intentionally blocked. The design leaves room for a parameter server that updates values over RPC.

---

## Type Traits (FNV-1a hash lookup)

`type_traits.hpp`

The mechanism that makes string-based topic names compile-time safe. `Hash(const char*)` is a `constexpr` FNV-1a implementation. `get<TopicName>(tuple)` finds the endpoint in the tuple whose `kHash` matches at compile time.

If a topic name doesn't exist in the spec list, `find_by_hash` triggers a `static_assert` at compile time — you get a clear error, not a runtime crash.

`TopicList<Topics...>` is just `std::tuple<Topics...>` — an alias for clarity in task declarations.

---

```cpp
// Hamlet 🐗 — documented every fold expression. Enjoyed it more than is reasonable.
```
