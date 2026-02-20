#ifndef CORE_LIFECYCLE_TASK_INTERFACE
#define CORE_LIFECYCLE_TASK_INTERFACE

#include <string>
#include <utility>

namespace core::lifecycle {

class TaskInterface
{
  public:
    explicit TaskInterface(std::string name) : name_(std::move(name)) {}
    virtual ~TaskInterface() = default;

    virtual void ExecuteStep() = 0;

    const std::string& Name() { return name_; }

  protected:
    std::string name_;
};

}  // namespace core::lifecycle
#endif  // CORE_LIFECYCLE_TASK_INTERFACE
