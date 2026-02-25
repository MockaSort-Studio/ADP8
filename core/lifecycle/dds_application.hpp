#ifndef CORE_LIFECYCLE_DDS_APPLICATION
#define CORE_LIFECYCLE_DDS_APPLICATION

#include <atomic>
#include <csignal>
#include <iostream>
#include <string>
#include <tuple>

#include <boost/core/demangle.hpp>

#include "core/communication/dds_context.hpp"
#include "core/lifecycle/tasks_manager.hpp"
#include "core/support/utils/black_magic_tuple_merger.hpp"
#include "core/support/utils/lookup_table.hpp"

namespace core::lifecycle {
namespace detail {
inline std::atomic<bool> shutdown_requested {false};
inline void signal_handler(int) { shutdown_requested.store(true); }
}  // namespace detail

// template<typename TaskTable>
// PippoSubscriber = tuple<DDSSubscriber<Pippotto>>
// PippoPublisher = tuple<DDSPublisher<Pippetto>>
// TaskTable = LookupTable<TableItem<PippoTask, 10ms, PippoSubscribers, PippoPublishers>
// TopicTypesList = BlackMagicTupleMerger<DioMerdaExtractor<PippoSubscriber>::type,
// DioMerdaExtractor<PippoPublishers>::type>::type
template <typename ApplicationConfig>
class DDSAPPlication final
{
    static_assert(
        core::utils::is_lookup_table_v<ApplicationConfig>,
        "ApplicationConfig must be a LookupTable: core/support/utils/lookup_table.hpp");

  public:
    DDSAPPlication(const std::string domain_participant_name)
    {
        std::signal(SIGINT, detail::signal_handler);
        std::signal(SIGTERM, detail::signal_handler);

        BuildTaskManager();
    }
    ~DDSAPPlication() { tasks_manager_->Stop(); }

    /**
     * @brief Blocks the main thread until SIGINT or SIGTERM.
     */
    void Run()
    {
        // Block until the atomic signal flag is set
        while (!detail::shutdown_requested.load(std::memory_order_relaxed))
        {
            // Sleep for a short duration to yield CPU to the ExecutionEngine
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        tasks_manager_->Stop();
    }

    // Init DDSProvider<TopicList>
  private:
    // ApplicationConfig<TableItem<TaskSpec<Task,Xms(ex.10)>,PublisherList,SubscriberList>>
    void BuildTaskManager()
    {
        std::unique_ptr<TasksManager> manager = std::make_unique<TasksManager>();

        ApplicationConfig::for_each_element(
            [&manager](auto type)
            {
                using TaskSpec = typename decltype(type)::Key;
                // using Values = typename decltype(type)::Values;
                // Static assert values size

                static_assert(
                    is_task_spec_v<TaskSpec>,
                    "Key of LookupTable must be a TaskSpec: "
                    "core/lifecycle/tasks_manager.hpp");

                const auto name {
                    boost::core::demangle(typeid(typename TaskSpec::TaskType).name())};

                manager->AddTask<typename TaskSpec::TaskType, TaskSpec::kFrequency>(name);
                // RegisterPubs<Pubs>(dds_context)
                // RegisterSubs<Subs>(dds_context)
            });

        tasks_manager_ = std::move(manager);
    }
    std::unique_ptr<TasksManager> tasks_manager_;
};
}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_DDS_APPLICATION
