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
    // Add static assert on template type
  public:
    static constexpr size_t N = Spec::kQueueSize;
    using Sub = communication::DDSSubscriber<typename Spec::type, N>;
    using T = typename Sub::DDSDataType;

    Input() { sub_.Start(Spec::kName); }
    void Fill() {}
    // void Push(T&& msg) { queue_.Push(std::move(msg)); }

    std::optional<utils::Sample<T>> Get() { return queue_.GetSample(); }

  private:
    utils::SizeConstrainedQueue<T, N> queue_;
    Sub sub_;
};
// Output_t = std::tuple<Output<Spec>>
template <typename Spec>
class Output
{
    // Add static assert on template type
  public:
    using Pub = communication::DDSPublisher<typename Spec::type>;
    using T = typename Pub::DDSDataType;

    Output() { pub_.Start(Spec::kName); }
    void Flush() { pub_.Publish(last_val_); }
    void Set(T&& msg) { last_val_ = std::move(msg); }

  private:
    T last_val_;
    Pub pub_;
};
template <uint64_t TargetHash, typename Tuple, size_t Index = 0>
struct find_by_hash
{
    static_assert(Index < std::tuple_size_v<Tuple>, "Topic Hash not found in List!");

    using CurrentWrapper = std::tuple_element_t<Index, Tuple>;
    using CurrentSpec = typename CurrentWrapper::spec_type;  // Get Spec from Input<Spec>

    static constexpr size_t value =
        (CurrentSpec::kHash == TargetHash)
            ? Index
            : find_by_hash<TargetHash, Tuple, Index + 1>::value;
};

template <typename T>
struct Inputs;
template <typename... Specs>
struct Inputs<std::tuple<Specs...>>
{
    using type = std::tuple<Input<Specs>...>;

    template <const char* TopicName>
    static constexpr size_t index_of =
        find_by_hash<communication::Hash(TopicName), type>::value;

    template <const char* TopicName>
    using msg_t = typename std::tuple_element_t<index_of<TopicName>, type>::T::type;

    template <const char* TopicName>
    static auto& get(type& storage)
    {
        return std::get<index_of<TopicName>>(storage);
    }
};

template <typename T>
struct Outputs;
template <typename... Specs>
struct Outputs<std::tuple<Specs...>>
{
    using type = std::tuple<Output<Specs>...>;

    template <const char* TopicName>
    static constexpr size_t index_of =
        find_by_hash<communication::Hash(TopicName), type>::value;

    template <const char* TopicName>
    using msg_t = typename std::tuple_element_t<index_of<TopicName>, type>::T::type;

    template <const char* TopicName>
    static auto& get(type& storage)
    {
        return std::get<index_of<TopicName>>(storage);
    }
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
    using TaskInterface::TaskInterface;
    virtual ~DDSTask() = default;

    void ExecuteStep() override
    {
        FillInputs();
        Execute();
        FlushOutputs();
    };

  protected:
    virtual void Execute() = 0;
    virtual void Init() {}

    // this return std::optional<Sample<T>>
    template <const char* TopicName>
    auto GetInput()
    {
        return decltype(inputs_)::get<TopicName>(inputs_).Get();
    }

    template <typename Type, const char* TopicName>
    void SetOutput(Type&& output)
    {  // find a way to add a static assert on the type
        decltype(outputs_)::get<TopicName>(outputs_).Set(std::forward<Type>(output));
    }

  private:
    void FillInputs()
    {
        std::apply([](auto&... input) { (input.Fill(), ...); }, inputs_);
    }
    void FlushOutputs()
    {
        std::apply([](auto&... output) { (output.Flush(), ...); }, outputs_);
    }
    Inputs_t<Subs> inputs_;
    Outputs_t<Pubs> outputs_;
};

// tuple<TopicSpecs>
// tuple<Input<TopicSpecs>>
// tuple<Output<TopicSpecs>>
}  // namespace core::lifecycle
// DDSTask : TaskInterface -> PippoAlg : public DDSTask
//  |--> Publisher e Subscribers

#endif  // CORE_LIFECYCLE_DDS_TASK
