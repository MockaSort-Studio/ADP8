#ifndef CORE_LIFECYCLE_DDS_APPLICATION
#define CORE_LIFECYCLE_DDS_APPLICATION

#include <atomic>
#include <boost/core/demangle.hpp>
#include <csignal>
#include <string>
#include <tuple>

#include "core/communication/dds_context.hpp"
#include "core/lifecycle/tasks_manager.hpp"
#include "core/support/utils/lookup_table.hpp"

namespace core::lifecycle {
namespace detail {
inline std::atomic<bool> shutdown_requested{false};
inline void signal_handler(int) { shutdown_requested.store(true); }
}  // namespace detail

/// @brief Top-level application class for a DDS-based Javelina process.
///
/// Constructs the @c DDSContext, registers SIGINT/SIGTERM handlers, and builds
/// the @c TasksManager from @p ApplicationConfig. Call @c Run() to block the
/// main thread until a shutdown signal is received.
///
/// @p ApplicationConfig must be a @c LookupTable mapping task types (@c
/// TaskInterface subclasses) to @c TaskSpec values. The engine starts
/// automatically at construction.
///
/// @tparam ApplicationConfig A @c LookupTable of @c {TaskType -> TaskSpec}
/// entries.
template <typename ApplicationConfig>
class DDSAPPlication final {
  static_assert(core::utils::is_lookup_table_v<ApplicationConfig>,
                "ApplicationConfig must be a LookupTable: "
                "core/support/utils/lookup_table.hpp");

 public:
  /// @brief Constructs the application: sets up DDS, installs signal handlers,
  /// builds tasks.
  /// @param domain_participant_name Name assigned to the FastDDS
  /// DomainParticipant.
  DDSAPPlication([[maybe_unused]] const std::string domain_participant_name) {
    std::signal(SIGINT, detail::signal_handler);
    std::signal(SIGTERM, detail::signal_handler);

    BuildTaskManager();
  }
  ~DDSAPPlication() { tasks_manager_->Stop(); }

  /**
   * @brief Blocks the main thread until SIGINT or SIGTERM.
   */
  void Run() {
    // Block until the atomic signal flag is set
    while (!detail::shutdown_requested.load(std::memory_order_relaxed)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    tasks_manager_->Stop();
  }

 private:
  void BuildTaskManager() {
    std::unique_ptr<TasksManager> manager = std::make_unique<TasksManager>();
    manager->Start();
    ApplicationConfig::for_each([&manager](auto Element) {
      using Task = typename decltype(Element)::Key;
      using TaskSpec =
          typename std::tuple_element_t<0, typename decltype(Element)::Values>;

      static_assert(std::is_base_of_v<TaskInterface, Task>,
                    "Key of LookupTable must be a TaskInterface: "
                    "core/lifecycle/tasks_interface.hpp");

      static_assert(is_task_spec_v<TaskSpec>,
                    "Value of LookupTable must be a TaskSpec: "
                    "core/lifecycle/tasks_manager.hpp");

      const auto name{boost::core::demangle(typeid(Task).name())};

      manager->AddTask<Task, TaskSpec::kFrequency>(name);
    });

    tasks_manager_ = std::move(manager);
  }
  std::unique_ptr<TasksManager> tasks_manager_;
};
}  // namespace core::lifecycle

#endif  // CORE_LIFECYCLE_DDS_APPLICATION
