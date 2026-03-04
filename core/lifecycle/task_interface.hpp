#ifndef CORE_LIFECYCLE_TASK_INTERFACE
#define CORE_LIFECYCLE_TASK_INTERFACE

#include <string>
#include <utility>

namespace core::lifecycle {

/// @brief Abstract base for all Javelina tasks.
///
/// Derive from @c DDSTask (for DDS-connected tasks) or directly from this for
/// custom scheduling. @c TasksManager calls @c ExecuteStep() at the configured
/// period on a dedicated worker thread.
///
/// @see DDSTask
/// @see TasksManager
class TaskInterface {
 public:
  explicit TaskInterface(std::string name) : name_(std::move(name)) {}
  virtual ~TaskInterface() = default;

  /// @brief Called by the execution engine at the configured period.
  ///        Implementations must be non-blocking.
  virtual void ExecuteStep() = 0;

  /// @return The task's name, as provided at construction.
  const std::string& Name() { return name_; }

 protected:
  std::string name_;
};

}  // namespace core::lifecycle
#endif  // CORE_LIFECYCLE_TASK_INTERFACE

// DDSTask : TaskInterface -> PippoAlg : public DDSTask
//  |--> Publisher e Subscribers