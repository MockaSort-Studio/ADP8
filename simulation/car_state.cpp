#include "car_msgs/msg/car_state.hpp"

#include <chrono>
#include <cmath>
#include <memory>

#include "applications/common_utils.hpp"
#include "car_msgs/msg/car_control.hpp"
#include "rclcpp/rclcpp.hpp"

#define T_CARSTATE_PUB 10  // ms
#define T_CARPOSE_PUB 200  // ms

class CarStatePublisher : public rclcpp::Node
{
  public:
    CarStatePublisher() : Node("car_state_publisher")
    {
        // declare parameters
        this->declare_parameter("callback_period_ms", T_CARSTATE_PUB);
        callback_period_ms_ = this->get_parameter("callback_period_ms").as_int();

        // Publisher
        car_state_publisher_ =
            this->create_publisher<car_msgs::msg::CarState>("car_state", T_CARSTATE_PUB);

        control_sub_ = this->create_subscription<car_msgs::msg::CarControl>(
            "car_control",
            T_CARSTATE_PUB,
            std::bind(&CarStatePublisher::control_callback, this, std::placeholders::_1));

        // Timer callback (simulation loop)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(callback_period_ms_),
            std::bind(&CarStatePublisher::state_update_callback, this));
    }

  private:
    void state_update_callback()
    {
        double dt = callback_period_ms_ / 1000.0;  // convert ms → seconds

        state_.d = d_cmd_;

        // --- Kinematic bicycle model ---
        state_.x += v_cmd_ * std::cos(state_.yaw) * dt;
        state_.y += v_cmd_ * std::sin(state_.yaw) * dt;
        state_.yaw += v_cmd_ * std::tan(state_.d) / L_WHEELBASE * dt;

        // --- Publish ---
        car_msgs::msg::CarState msg;
        msg.x = state_.x;
        msg.y = state_.y;
        msg.yaw = state_.yaw;
        msg.d = state_.d;

        car_state_publisher_->publish(msg);
    }

    void control_callback(const car_msgs::msg::CarControl::SharedPtr msg)
    {
        v_cmd_ = msg->v;
        d_cmd_ = msg->d;
    }

    // ROS2 interfaces
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<car_msgs::msg::CarState>::SharedPtr car_state_publisher_;
    rclcpp::Subscription<car_msgs::msg::CarControl>::SharedPtr control_sub_;

    CarState state_;  // internal state

    double v_cmd_ = 0.0;  // vel command
    double d_cmd_ = 0.0;  // steering wheel command

    int callback_period_ms_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarStatePublisher>());
    rclcpp::shutdown();
    return 0;
}