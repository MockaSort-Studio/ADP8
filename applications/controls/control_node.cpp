#include <cmath>

// applications
#include "applications/applications_param.hpp"
#include "applications/common_utils.hpp"
#include "applications/controls/point_follower.hpp"
// messages
#include "car_msgs/msg/car_control.hpp"
#include "car_msgs/msg/car_state.hpp"
// ROS2
#include "core/tasks_manager.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "support/lookup_table.hpp"

class CarControlNode : public sert::core::TaskInterface
{
  public:
    CarControlNode(const std::string& name, rclcpp::NodeOptions options)
        : TaskInterface(
              name, options.append_parameter_override("cycle_time_ms", T_CARCONTROL_PUB))
    {
        RegisterPublisher<car_msgs::msg::CarControl>("car_cmd", QS_CARCONTROL_PUB);

        RegisterSubscriber<car_msgs::msg::CarState>(
            "state_est",
            [this](car_msgs::msg::CarState::UniquePtr msg)
            {
                state_.x = msg->x;
                state_.y = msg->y;
                state_.yaw = msg->yaw;
                state_.d = msg->d;
                state_.v = msg->v;

                state_received_ = true;
            });

        RegisterSubscriber<geometry_msgs::msg::Point>(
            "planning_target",
            [this](geometry_msgs::msg::Point::UniquePtr msg)
            {
                target_.x = msg->x;
                target_.y = msg->y;
                target_received_ = true;
            });

        start_time_ = this->now();
    }

  protected:
    void ExecuteStep() override
    {
        if (!state_received_ || !target_received_)
        {
            return;
        }

        if (!control_enabled_)
        {
            if ((this->now() - start_time_).seconds() < 10.0)
            {
                return;
            }
            control_enabled_ = true;
            RCLCPP_INFO(this->get_logger(), "CarControlNode: control enabled!");
        }

        // carcmd_ = point_follower(state_, target_);
        // carcmd_ = pure_pursuit(state_, target_, state_.v);
        carcmd_ = stanley(state_, target_, state_.v);

        cmd_msg_.v = carcmd_.v;
        cmd_msg_.d = carcmd_.d;

        GetPublisher<car_msgs::msg::CarControl>("car_cmd")->publish(cmd_msg_);
    }

  private:
    car_msgs::msg::CarControl cmd_msg_;

    VehicleState state_;
    CarControl carcmd_;
    Point target_;

    bool state_received_ = false;
    bool target_received_ = false;
    bool control_enabled_ = false;

    rclcpp::Time start_time_;
};

using CarControlNodeConfig = sert::support::LookupTable<
    sert::support::TableItem<CarControlNode, sert::support::UnusedValue>>;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto task_manager = sert::core::BuildTasksManager<CarControlNodeConfig>();
    task_manager->Execute();
    rclcpp::shutdown();
    return 0;
}
