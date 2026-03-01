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
#include "core/support/utils/lookup_table.hpp"

namespace core::lifecycle {
namespace detail {
inline std::atomic<bool> shutdown_requested {false};
inline void signal_handler(int) { shutdown_requested.store(true); }
}  // namespace detail

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

        communication::DDSContextProvider::SetName(domain_participant_name);

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

  private:
    void BuildTaskManager()
    {
        std::unique_ptr<TasksManager> manager = std::make_unique<TasksManager>();
        //LookUpTable here is used like list, a tuple could suffice.
        //Although in future we may want to propagate informations (e.g. task manager options for that specific task)
        ApplicationConfig::for_each_element(
            [&manager](auto Element)
            {
                using TaskSpec = typename decltype(Element)::Key;

                static_assert(
                    is<TaskSpec>,
                    "Key of LookupTable must be a TaskSpec: "
                    "core/lifecycle/tasks_manager.hpp");

                const auto name {
                    boost::core::demangle(typeid(typename TaskSpec::TaskType).name())};

                manager->AddTask<typename TaskSpec::TaskType, TaskSpec::kFrequency>(name);
            });

        tasks_manager_ = std::move(manager);
    }
    std::unique_ptr<TasksManager> tasks_manager_;
};
}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_DDS_APPLICATION
