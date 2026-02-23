#ifndef CORE_LIFECYCLE_DDS_TASK
#define CORE_LIFECYCLE_DDS_TASK

#include <string>

#include "core/communication/dds_context.hpp"
#include "core/communication/dds_publisher.hpp"
#include "core/communication/dds_subscriber.hpp"
#include "core/lifecycle/task_interface.hpp"
#include "core/support/utils/size_constrained_queue.hpp"
namespace core::lifecycle {

// TaskChain = LookupTable<PippoTask, 10ms>
// PippoTaskPubSub = tuple<Subscribers, Publisher>
// AllTopics = MagiaNeraAummaAumma<PippoTaskPubSub, DioCaneTaskPubSub>
// DDSProvider<AllTopics>
// pippo:
//  - frequency_ms: 10ms
//  - subscribers:
//      - {dio,10, type}
//      - lupo,11
//      - serpente,12
//  - publishers:
//      - madonna, type
//      - ladra, type
//      - assassina, type

// Subscribers = tuple<DDSPublisher/DDSSubscribner<PorcoddioTopic>, PippoTopic>
// class PippoTask : public DDSTask<PippoTaskPubSub>
//  public:
//

template <typename Spec>
class Input
{
  public:
    using T = typename Spec::type;
    static constexpr size_t N = Spec::Queue();

    Input(communication::DDSContext& ctx) {}
    void Push(T&& msg) { queue_.Push(std::move(msg)); }

    std::optional<utils::Sample<T>> Get() { return queue_.GetSample(); }

  private:
    utils::SizeConstrainedQueue<T, N> queue_;
};

template <typename Spec>
class Output
{
  public:
    using T = typename Spec::type;

    Output(communication::DDSContext& ctx) {}

    // Zero-copy: take ownership of the data to be sent
    void Set(T&& msg)
    {
        // Here you would call your DDS DataWriter::write(std::move(msg))
        // or store it temporarily if the task flushes later.
        last_val_ = std::move(msg);
    }

  private:
    T last_val_;
};

template <typename T>
struct Inputs;
template <typename... Specs>
struct Inputs<std::tuple<Specs...>>
{
    using type = std::tuple<Input<Specs>...>;
};

template <typename T>
struct Outputs;
template <typename... Specs>
struct Outputs<std::tuple<Specs...>>
{
    using type = std::tuple<Output<Specs>...>;
};

template <typename T>
using Inputs_t = typename Inputs<T>::type;

template <typename T>
using Outputs_t = typename Outputs<T>::type;

template <typename... Topics>
using TopicList = std::tuple<Topics...>;

template <typename SubscriptionSpecs, typename PublicationSpecs>
class DDSTask : public TaskInterface
{
    using Subs = SubscriptionSpecs;
    using Pubs = PublicationSpecs;

  public:
    explicit DDSTask(std::string name) : TaskInterface(name)
    {
        AddSubscribers(std::make_index_sequence<std::tuple_size_v<Subs>> {});
        AddPublishers(std::make_index_sequence<std::tuple_size_v<Pubs>> {});
    }
    virtual ~DDSTask() = default;

    void ExecuteStep() override
    {
        // Fill inputs
        Execute();
        // Flush outputs
    };

    virtual void Execute() = 0;

    virtual void Init() {}

    // GetInput<Type>(n. samples)
    // SetOutput<Type>(value)
    template <typename Type>
    void AddPublisher(const std::string& name)
    {
        auto& ctx = communication::DDSContextProvider::Get();
    }

    template <typename Type, uint8_t queue_size>
    void AddSubscriber(const std::string& name)
    {
        auto& ctx = communication::DDSContextProvider::Get();
    }

  private:
    template <std::size_t... Is>
    void AddSubscribers(std::index_sequence<Is...>)
    {
        (AddSubscriber<
             typename std::tuple_element_t<Is, Subs>::type,
             typename std::tuple_element_t<Is, Subs>::Queue()>(
             typename std::tuple_element_t<Is, Subs>::Name()),
         ...);
    }

    template <std::size_t... Is>
    void AddPublishers(std::index_sequence<Is...>)
    {
        (AddPublisher<typename std::tuple_element_t<Is, Pubs>::type>(
             typename std::tuple_element_t<Is, Pubs>::Name()),
         ...);
    }

    Inputs_t<Subs> inputs_;
    Outputs_t<Pubs> outputs_;
    // DDSSubscriber<Ticks, 10>
    // DDSProvider.GetTopic<DDSSubscriber<Ticks, 10>::TopicType> ===== oggetto topic
    // input_ (tuple(DDSSubscriber<Ticks, 10>, SizeConstrainedQueue<TicksMessages,10>),
    // output_ (tuple(DDSPublisher<PorcoDiddio>, PorcoDiddioMessages),
    // .......)
};

}  // namespace core::lifecycle
// DDSTask : TaskInterface -> PippoAlg : public DDSTask
//  |--> Publisher e Subscribers

#endif  // CORE_LIFECYCLE_DDS_TASK
