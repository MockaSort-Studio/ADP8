#include <chrono>
#include <cmath>
#include <memory>

#include "applications/common_utils.hpp"
#include "applications/controls/point_follower.hpp"
#include "car_msgs/msg/car_control.hpp"
#include "car_msgs/msg/car_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

#define T_CARCONTROL_PUB 10       // ms
#define T_CARSTATE_SUB 10         // ms
#define T_PLANNINGTARGET_SUB 100  // ms

class CarControlNode : public rclcpp::Node
{
  public:
    CarControlNode() : Node("car_control_publisher")
    {
        // declare parameters
        this->declare_parameter("callback_period_ms", T_CARCONTROL_PUB);
        callback_period_ms_ = this->get_parameter("callback_period_ms").as_int();

        // Publisher
        car_control_publisher_ = this->create_publisher<car_msgs::msg::CarControl>(
            "car_control", T_CARCONTROL_PUB);

        // subscribers
        car_state_sub_ = this->create_subscription<car_msgs::msg::CarState>(
            "car_state",
            T_CARSTATE_SUB,
            std::bind(&CarControlNode::car_state_callback, this, std::placeholders::_1));
        planning_target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "planning_target",
            T_PLANNINGTARGET_SUB,
            std::bind(
                &CarControlNode::planning_target_callback, this, std::placeholders::_1));

        // control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(callback_period_ms_),
            std::bind(&CarControlNode::control_callback, this));

        start_time_ = this->now();
    }

  private:
    void car_state_callback(const car_msgs::msg::CarState::SharedPtr msg)
    {
        state_.x = msg->x;
        state_.y = msg->y;
        state_.yaw = msg->yaw;
        state_.d = msg->d;
    }

    void planning_target_callback(const geometry_msgs::msg::Point msg)
    {
        target_.x = msg.x;
        target_.y = msg.y;
    }

    void control_callback()
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

        carcmd_ = point_follower(state_, target_);

        // --- Publish command ---
        car_msgs::msg::CarControl msg;
        msg.v = carcmd_.v;
        msg.d = carcmd_.d;
        car_control_publisher_->publish(msg);
    }

    // ROS2 interfaces
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<car_msgs::msg::CarControl>::SharedPtr car_control_publisher_;
    rclcpp::Subscription<car_msgs::msg::CarState>::SharedPtr car_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr planning_target_sub_;
    rclcpp::Time start_time_;
    int callback_period_ms_;

    // control variables
    CarState state_;  // internal state
    CarControl carcmd_;
    bool control_enabled_ = false;

    // ref
    Point target_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarControlNode>());
    rclcpp::shutdown();
    return 0;
}