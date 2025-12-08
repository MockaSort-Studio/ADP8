#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/float32.hpp"
#include "car_msgs/msg/car_state.hpp"

#define T_CARSTATE_PUB 10  // ms
#define T_CARPOSE_PUB 200  // ms
#define L_WHEELBASE 2.5    // meters

struct CarState
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double d = 0.0;  // steering angle (input)
};

class CarStatePublisher : public rclcpp::Node
{
  public:
    CarStatePublisher() : Node("car_state_publisher")
    {
        // declare parameters
        this->declare_parameter("callback_period_ms", T_CARSTATE_PUB);
        this->declare_parameter("v", 5.0);               // longitudinal speed (m/s)
        this->declare_parameter("steering_angle", 0.2);  // radians

        // Get parameters
        v_ = this->get_parameter("v").as_double();
        state_.d = this->get_parameter("steering_angle").as_double();
        callback_period_ms_ = this->get_parameter("callback_period_ms").as_int();

        // Publisher
        publisher_ =
            this->create_publisher<car_msgs::msg::CarState>("car_state", T_CARSTATE_PUB);

        // Timer callback (simulation loop)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(callback_period_ms_),
            std::bind(&CarStatePublisher::state_update_callback, this));
    }

  private:
    void state_update_callback()
    {
        double dt = callback_period_ms_ / 1000.0;  // convert ms → seconds

        // --- Kinematic bicycle model ---
        state_.x += v_ * std::cos(state_.yaw) * dt;
        state_.y += v_ * std::sin(state_.yaw) * dt;
        state_.yaw += v_ * std::tan(state_.d) / L_WHEELBASE * dt;

        // --- Publish ---
        car_msgs::msg::CarState msg;
        msg.x = state_.x;
        msg.y = state_.y;
        msg.yaw = state_.yaw;
        msg.d = state_.d;

        publisher_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<car_msgs::msg::CarState>::SharedPtr publisher_;

    CarState state_;
    double v_;
    int callback_period_ms_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarStatePublisher>());
    rclcpp::shutdown();
    return 0;
}