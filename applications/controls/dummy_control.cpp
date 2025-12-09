#include <chrono>
#include <cmath>
#include <memory>

#include "applications/common_utils.hpp"
#include "car_msgs/msg/car_control.hpp"
#include "car_msgs/msg/car_state.hpp"
#include "rclcpp/rclcpp.hpp"

#define T_CARCONTROL_PUB 10  // ms

class CarControl : public rclcpp::Node
{
  public:
    CarControl() : Node("car_control_publisher")
    {
        // declare parameters
        this->declare_parameter("callback_period_ms", T_CARCONTROL_PUB);
        callback_period_ms_ = this->get_parameter("callback_period_ms").as_int();

        // Publisher
        car_control_publisher_ = this->create_publisher<car_msgs::msg::CarControl>(
            "car_control", T_CARCONTROL_PUB);

        car_state_sub_ = this->create_subscription<car_msgs::msg::CarState>(
            "car_state",
            T_CARCONTROL_PUB,
            std::bind(&CarControl::car_state_callback, this, std::placeholders::_1));

        // control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(callback_period_ms_),
            std::bind(&CarControl::control_callback, this));
    }

  private:
    void car_state_callback(const car_msgs::msg::CarState::SharedPtr msg)
    {
        state_.x = msg->x;
        state_.y = msg->y;
        state_.yaw = msg->yaw;
        state_.d = msg->d;
    }

    void control_callback()
    {
        car_msgs::msg::CarControl msg;
        msg.v = 0.2;
        msg.d = 0.0;

        car_control_publisher_->publish(msg);
    }

    // ROS2 interfaces
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<car_msgs::msg::CarControl>::SharedPtr car_control_publisher_;
    rclcpp::Subscription<car_msgs::msg::CarState>::SharedPtr car_state_sub_;

    CarState state_;  // internal state

    int callback_period_ms_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarControl>());
    rclcpp::shutdown();
    return 0;
}