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

        //
        double dist = std::sqrt(
            (x_r - state_.x) * (x_r - state_.x) + (y_r - state_.y) * (y_r - state_.y));

        double x_f = state_.x + L_WHEELBASE * std::cos(state_.yaw);
        double y_f = state_.y + L_WHEELBASE * std::sin(state_.yaw);
        double desired_heading = std::atan2(y_r - y_f, x_r - x_f);

        double heading_error = desired_heading - state_.yaw;
        while (heading_error > M_PI)
            heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI)
            heading_error += 2.0 * M_PI;

        d_ = 1.2 * heading_error;  // "P control" for the beggar
        double d_max = 0.5;
        d_ = std::clamp(d_, -d_max, d_max);

        v_ = 0.6 * dist;  // proportional speed
        double v_max = 5.0;
        v_ = std::clamp(v_, 0.0, v_max);

        // Stop when close
        if (dist < 0.05)
        {
            v_ = 0.0;
        }

        // --- Publish command ---
        car_msgs::msg::CarControl msg;
        msg.v = v_;
        msg.d = d_;
        car_control_publisher_->publish(msg);
    }

    // ROS2 interfaces
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<car_msgs::msg::CarControl>::SharedPtr car_control_publisher_;
    rclcpp::Subscription<car_msgs::msg::CarState>::SharedPtr car_state_sub_;
    rclcpp::Time start_time_;

    CarState state_;  // internal state

    int callback_period_ms_;

    // control variables
    double v_;
    double d_;
    bool control_enabled_ = false;

    // ref
    double x_r = 5.0;
    double y_r = 5.0;
    double yaw_r = 0.0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarControl>());
    rclcpp::shutdown();
    return 0;
}