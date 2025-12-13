#include <chrono>
#include <cmath>
#include <memory>

#include "applications/applications_param.hpp"
#include "applications/common_utils.hpp"
#include "applications/controls/point_follower.hpp"
#include "car_msgs/msg/car_control.hpp"
#include "car_msgs/msg/car_state.hpp"
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
        RegisterPublisher<car_msgs::msg::CarControl>("car_control", QS_CARCONTROL_PUB);

        RegisterSubscriber<car_msgs::msg::CarState>(
            "car_state",
            [this](car_msgs::msg::CarState::UniquePtr msg)
            {
                state_.x = msg->x;
                state_.y = msg->y;
                state_.yaw = msg->yaw;
                state_.d = msg->d;
            });

        RegisterSubscriber<geometry_msgs::msg::Point>(
            "planning_target",
            [this](geometry_msgs::msg::Point::UniquePtr msg)
            {
                target_.x = msg->x;
                target_.y = msg->y;
            });

        start_time_ = this->now();
    }

  protected:
    void ExecuteStep() override
    {
        if (!control_enabled_)
        {
            if ((this->now() - start_time_).seconds() < 10.0)
            {
                // do nothing for 10 seconds
                return;
            }
            control_enabled_ = true;
            RCLCPP_INFO(this->get_logger(), "Control enabled after 10 seconds.");
        }

        //
        // carcmd_ = point_follower(state_, target_);
        // carcmd_ = pure_pursuit(state_, target_, carcmd_.d);
        carcmd_ = stanley(state_, target_, carcmd_.d);

        // --- Publish command ---
        cmd_msg_.v = carcmd_.v;
        cmd_msg_.d = carcmd_.d;
        auto publisher = GetPublisher<decltype(cmd_msg_)>("car_control");
        publisher->publish(cmd_msg_);
    }

  private:
    //
    car_msgs::msg::CarControl cmd_msg_;

    //
    CarState state_;
    CarControl carcmd_;
    Point target_;

    //
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