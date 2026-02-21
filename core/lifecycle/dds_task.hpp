#ifndef CORE_LIFECYCLE_DDS_TASK
#define CORE_LIFECYCLE_DDS_TASK

#include <string>

#include "core/lifecycle/task_interface.hpp"

namespace core::lifecycle {

// TaskChain = LookupTable<PippoTask, 10ms>
// PippoTaskPubSub = tuple<Subscribers, Publisher>
// AllTopics = MagiaNeraAummaAumma<PippoTaskPubSub, DioCaneTaskPubSub>
// DDSProvider<AllTopics>
// pippo:
//  - frequency_ms: 10ms
//  - subscribers:
//      - {dio,10}
//      - lupo,11
//      - serpente,12
//  - publishers:
//      - madonna
//      - ladra
//      - assassina

// Subscribers = tuple<DDSPublisher/DDSSubscribner<PorcoddioTopic>, PippoTopic>
// class PippoTask : public DDSTask<PippoTaskPubSub>
//  public:
//
class DDSTask : public TaskInterface
{
  public:
    explicit DDSTask(std::string name) : TaskInterface(name) {}
    virtual ~DDSTask() = default;

    void ExecuteStep() override
    {
        // Fill inputs
        Execute();
        // Flush outputs
    };

    virtual void Execute() = 0;

    // GetInputQueue()

  private:
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
