#ifndef CORE_LIFECYCLE_TASKS_MANAGER
#define CORE_LIFECYCLE_TASKS_MANAGER

#include <csignal>

#include "core/lifecycle/execution_engine.hpp"
#include "core/lifecycle/task_interface.hpp"

namespace core::lifecycle {

/// @brief Specifies the execution period for a task.
/// @tparam Ms Period in milliseconds. Must be > 0.
template <std::chrono::milliseconds::rep Ms>
struct TaskSpec {
  static constexpr std::chrono::milliseconds::rep kFrequency = Ms;
};

namespace {

template <typename T, typename = void>
struct is_task_spec : std::false_type {};

template <typename T>
struct is_task_spec<
    T,
    std::void_t<decltype(T::kFrequency),
                std::enable_if_t<std::is_integral_v<decltype(T::kFrequency)>>>>
    : std::true_type {};

template <typename T>
constexpr bool is_task_spec_v = is_task_spec<T>::value;
}  // namespace

/// @brief Owns and schedules a collection of @c TaskInterface instances.
///
/// Tasks are registered via @c AddTask() before the engine starts.
/// The underlying @c ExecutionEngine handles periodic scheduling on a dedicated
/// worker thread. Non-copyable.
class TasksManager final {
 public:
  TasksManager() = default;
  ~TasksManager() { engine_->Stop(); }

  TasksManager(const TasksManager&) = delete;
  TasksManager& operator=(const TasksManager&) = delete;

  TasksManager(TasksManager&& other) noexcept
      : tasks_register_(std::move(other.tasks_register_)),
        engine_(std::move(other.engine_)) {}

  TasksManager& operator=(TasksManager&& other) noexcept {
    if (this != &other) {
      tasks_register_ = std::move(other.tasks_register_);
      engine_ = std::move(other.engine_);
    }
    return *this;
  }

  /// @brief Registers and schedules a new task at the given period.
  /// @tparam T  Task type. Must derive from @c TaskInterface.
  /// @tparam Ms Execution period in milliseconds. Must be > 0.
  /// @param name Task name, used for logging and identification.
  template <class T, std::chrono::milliseconds::rep Ms,
            std::enable_if_t<std::is_base_of_v<TaskInterface, T>, int> = 0>
  void AddTask(const std::string& name) {
    static_assert(Ms > 0, "Frequency must be positive.");

    auto task = std::make_shared<T>(name);
    tasks_register_.push_back(task);

    engine_->Schedule(std::chrono::milliseconds(Ms),
                      [task]() { task->ExecuteStep(); });
  }

  /// @brief Starts the execution engine. Safe to call only once.
  void Start() { engine_->Start(); }

  /// @brief Stops the execution engine and joins the worker thread. Idempotent.
  void Stop() { engine_->Stop(); }

 private:
  std::vector<std::shared_ptr<TaskInterface>> tasks_register_;
  std::unique_ptr<ExecutionEngine> engine_ =
      std::make_unique<ExecutionEngine>();
};

}  // namespace core::lifecycle
#endif  // CORE_LIFECYCLE_TASKS_MANAGER
