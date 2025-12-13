#include <cmath>

#include "applications/applications_param.hpp"
#include "applications/common_utils.hpp"
#include "applications/planning/dummy_point_planner.hpp"
#include "core/tasks_manager.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "support/lookup_table.hpp"

class PlanningNode : public sert::core::TaskInterface
{
  public:
    PlanningNode(const std::string& name, rclcpp::NodeOptions options)
        : TaskInterface(
              name, options.append_parameter_override("cycle_time_ms", T_PLANNING_PUB))
    {
        RegisterPublisher<geometry_msgs::msg::Point>("planning_target", QS_PLANNING_PUB);

        start_time_ = this->now().seconds();
    }

  protected:
    void ExecuteStep() override
    {
        double t = this->now().seconds() - start_time_;

        // target_ = point_target();
        target_ = figure8_traj(t);

        // --- Publish target ---
        point_msg_.x = target_.x;
        point_msg_.y = target_.y;
        point_msg_.z = 0.0;
        auto publisher = GetPublisher<decltype(point_msg_)>("planning_target");
        publisher->publish(point_msg_);
    }

  private:
    // ros2 interfaces
    geometry_msgs::msg::Point point_msg_;
    double start_time_;

    // ref
    Point target_;
};

using PlanningNodeConfig = sert::support::LookupTable<
    sert::support::TableItem<PlanningNode, sert::support::UnusedValue>>;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto task_manager = sert::core::BuildTasksManager<PlanningNodeConfig>();
    task_manager->Execute();
    rclcpp::shutdown();
    return 0;
}
