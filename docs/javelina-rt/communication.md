# Communication

`core/communication/` — FastDDS wrappers, topic specs, domain context.

---

## TopicSpec

`topic_spec.hpp`

The contract type that connects the communication layer to the lifecycle layer. All topic configuration lives here.

```cpp
template <typename T, const char* TopicName, size_t max_queue_size = 1>
struct TopicSpec {
  using type = T;                             // FastDDS PubSubType
  static constexpr const char* kName;         // topic name string
  static constexpr size_t kQueueSize;         // subscriber queue depth
};
```

`T` must derive from `eprosima::fastdds::dds::TopicDataType`. Enforced by `static_assert`.

`is_topic_spec_v<T>` — type trait for checking at instantiation sites.

In practice, `TopicSpec` instances are declared in generated headers (see [Generators](generators.md)) — you don't write them by hand.

---

## DDSContext

`dds_context.hpp`

Manages the FastDDS `DomainParticipant` and the topic registry. Two classes:

**`DDSContext`** — owns one `DomainParticipant`, creates and caches `Topic` objects:

```cpp
DDSContext ctx("my_participant");
dds::Topic* topic = ctx.GetDDSTopic<MyTopicSpec>();
```

Topics are created on first request and cached by name. The participant and all topics are cleaned up via RAII deleters — no manual lifecycle calls.

**`DDSContextProvider`** — lazy singleton wrapper. Initialized by `DDSAPPlication` before any task starts:

```cpp
DDSContextProvider::SetName("my_participant");  // call before any GetDDSTopic
DDSContext& ctx = DDSContextProvider::Get();    // creates instance on first call
```

`SetName` must be called before the first `Get()`. After that, the name is fixed.

Note: `DDSContextProvider` is not thread-safe on construction. The design assumption is single-threaded setup — tasks start only after the application is constructed.

---

## DDSPublisher

`dds_publisher.hpp`

Thin wrapper over a FastDDS `DataWriter`. Called by `DataEndpoint<Spec, DataDirection::Out>` during `Sync()`.

```cpp
DDSPublisher<MyPubSubType> pub;
pub.Start("my_topic");
pub.Publish(my_message);
```

---

## DDSSubscriber

`dds_subscriber.hpp`

Thin wrapper over a FastDDS `DataReader`. Accumulates incoming messages internally. Called by `DataEndpoint<Spec, DataDirection::In>` during `FillInputs()`.

```cpp
DDSSubscriber<MyPubSubType, kQueueSize> sub;
sub.Start("my_topic");
sub.DrainQueue(queue);  // moves buffered samples into a SizeConstrainedQueue
```

`DrainQueue` transfers everything accumulated since the last drain into the task-local `SizeConstrainedQueue`.

---

## SizeConstrainedQueue (legacy)

`core/communication/size_constrained_queue.hpp`

**Outdated.** The canonical implementation is `core/support/utils/size_constrained_queue.hpp`. This file is kept for reference and will be removed. Do not use it in new code.

---

```cpp
// Hamlet 🐗 — proudly AI-generated, human-reviewed
```
