#ifndef CORE_LIFECYCLE_TASKS_MANAGER
#define CORE_LIFECYCLE_TASKS_MANAGER

#include <atomic>
#include <condition_variable>
#include <csignal>

#include "core/lifecycle/execution_engine.hpp"
#include "core/lifecycle/task_interface.hpp"

namespace core::lifecycle {
namespace detail {
inline std::atomic<bool> shutdown_requested {false};
inline void signal_handler(int) { shutdown_requested.store(true); }
}  // namespace detail
class TasksManager final
{
  public:
    TasksManager()
    {
        std::signal(SIGINT, detail::signal_handler);
        std::signal(SIGTERM, detail::signal_handler);

        engine_->Start();
    }
    ~TasksManager() = default;

    TasksManager(const TasksManager&) = delete;
    TasksManager& operator=(const TasksManager&) = delete;

    TasksManager(TasksManager&& other) noexcept
        : tasks_register_(std::move(other.tasks_register_)),
          engine_(std::move(other.engine_))
    {}

    TasksManager& operator=(TasksManager&& other) noexcept
    {
        if (this != &other)
        {
            tasks_register_ = std::move(other.tasks_register_);
            engine_ = std::move(other.engine_);
        }
        return *this;
    }

    /**
     * @brief Add new tasks with a given frequency in ms
     */
    template <
        class T,
        std::chrono::milliseconds::rep Ms,
        std::enable_if_t<std::is_base_of_v<TaskInterface, T>, int> = 0>
    void AddTask(const std::string& name)
    {
        static_assert(Ms > 0, "Frequency must be positive.");

        auto task = std::make_shared<T>(name);
        tasks_register_.push_back(task);

        engine_->Schedule(
            std::chrono::milliseconds(Ms), [task]() { task->ExecuteStep(); });
    }

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

        engine_->Stop();
    }

  private:
    std::vector<std::shared_ptr<TaskInterface>> tasks_register_;
    std::unique_ptr<ExecutionEngine> engine_ = std::make_unique<ExecutionEngine>();

    std::condition_variable wait_cv_;
};

using TasksManagerPtr = std::unique_ptr<TasksManager>;

// template <typename StaticConfiguration>
// TasksManagerPtr BuildTasksManager()
// {
//     TasksManagerPtr manager = std::make_unique<TasksManager>();
//     StaticConfiguration::for_each_element(
//         [&manager](auto type)
//         {
//             using TaskType = typename decltype(type)::Key;
//             const auto node_name {boost::core::demangle(typeid(TaskType).name())};
//             RCLCPP_INFO(
//                 rclcpp::get_logger("TasksManager"), "Adding task: %s",
//                 node_name.c_str());
//             manager->AddTask<TaskType>(node_name, rclcpp::NodeOptions());
//         });
//     return manager;
// }
}  // namespace core::lifecycle
#endif  // CORE_LIFECYCLE_TASKS_MANAGER
