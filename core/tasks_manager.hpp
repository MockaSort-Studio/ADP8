#ifndef CORE_TASKS_MANAGER
#define CORE_TASKS_MANAGER
#include <boost/core/demangle.hpp>

#include "core/task_interface.hpp"

namespace sert::core {
class TasksManager
{
  public:
    TasksManager()
    {
        RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "TasksManager initialized.");
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    }
    ~TasksManager() = default;

    TasksManager(const TasksManager&) = delete;
    TasksManager& operator=(const TasksManager&) = delete;

    TasksManager(TasksManager&& other) noexcept
        : tasks_register_(std::move(other.tasks_register_)),
          executor_(std::move(other.executor_))
    {}

    TasksManager& operator=(TasksManager&& other) noexcept
    {
        if (this != &other)
        {
            tasks_register_ = std::move(other.tasks_register_);
            executor_ = std::move(other.executor_);
        }
        return *this;
    }

    template <typename TaskType>
    void AddTask(const std::string& name, rclcpp::NodeOptions options)
    {
        auto task = std::make_shared<TaskType>(name, options);
        tasks_register_.push_back(task);
        executor_->add_node(task);
        RCLCPP_INFO(task->get_logger(), "Task added: %s", name.c_str());
    }

    void Execute()
    {
        RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "Starting task execution...");
        executor_->spin();
        RCLCPP_INFO(rclcpp::get_logger("TasksManager"), "Task execution finished.");
    }

  private:
    std::vector<std::shared_ptr<TaskInterface>> tasks_register_;
    rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;
};

using TasksManagerPtr = std::unique_ptr<TasksManager>;

template <typename StaticConfiguration>
TasksManagerPtr BuildTasksManager()
{
    TasksManagerPtr manager = std::make_unique<TasksManager>();
    StaticConfiguration::for_each_element(
        [&manager](auto type)
        {
            using TaskType = typename decltype(type)::Key;
            const auto node_name {boost::core::demangle(typeid(TaskType).name())};
            RCLCPP_INFO(
                rclcpp::get_logger("TasksManager"), "Adding task: %s", node_name.c_str());
            manager->AddTask<TaskType>(node_name, rclcpp::NodeOptions());
        });
    return manager;
}
}  // namespace sert::core
#endif  // CORE_TASKS_MANAGER
