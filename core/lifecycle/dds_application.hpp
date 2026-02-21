#ifndef CORE_LIFECYCLE_DDS_APPLICATION
#define CORE_LIFECYCLE_DDS_APPLICATION

#include <string>

#include "core/lifecycle/tasks_manager.hpp"

namespace core::lifecycle {
// template<typename TaskTable>
// PippoSubscriber = tuple<DDSSubscriber<Pippotto>>
// PippoPublisher = tuple<DDSPublisher<Pippetto>>
// TaskTable = LookupTable<TableItem<PippoTask, 10ms, PippoSubscribers, PippoPublishers>
// TopicTypesList = BlackMagicTupleMerger<PippoSubscriber, PippoPublishers>::topics
class DDSAPPlication final
{
  public:
    DDSAPPlication(const std::string domain_participant_name)
        : domain_participant_name_(domain_participant_name)
    {}
    ~DDSAPPlication() = default;
    // Init DDSProvider<TopicList>
  private:
    TasksManager tasks_manager_;
    std::string domain_participant_name_;
};
}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_DDS_APPLICATION
