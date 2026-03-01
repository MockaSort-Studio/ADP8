#ifndef CORE_LIFECYCLE_TASKS_MANAGER
#define CORE_LIFECYCLE_TASKS_MANAGER

#include <csignal>

#include "core/lifecycle/execution_engine.hpp"
#include "core/lifecycle/task_interface.hpp"

namespace core::lifecycle {

template <class T, std::chrono::milliseconds::rep Ms>
struct TaskSpec
{
    using TaskType = T;
    static constexpr std::chrono::milliseconds::rep kFrequency = Ms;
};

namespace {

template <typename T, typename = void>
struct is_task_spec : std::false_type
{
};

template <typename T>
struct is_task_spec<
    T,
    std::void_t<
        typename T::TaskType,
        decltype(T::kFrequency),
        std::enable_if_t<std::is_integral_v<decltype(T::kFrequency)>>>> : std::true_type
{
};

template <typename T>
constexpr bool is_task_spec_v = is_task_spec<T>::value;
}  // namespace

class TasksManager final
{
  public:
    TasksManager() = default;
    ~TasksManager() { engine_->Stop(); }

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

    void Start() { engine_->Start(); }
    void Stop() { engine_->Stop(); }

  private:
    std::vector<std::shared_ptr<TaskInterface>> tasks_register_;
    std::unique_ptr<ExecutionEngine> engine_ = std::make_unique<ExecutionEngine>();
};

}  // namespace core::lifecycle
#endif  // CORE_LIFECYCLE_TASKS_MANAGER
