#include <chrono>
#include <cmath>
#include <memory>

#include "applications/common_utils.hpp"
#include "applications/planning/dummy_point_planner.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

#define T_PLANNING_PUB 100  // ms

class PlanningNode : public rclcpp::Node
{
  public:
    PlanningNode() : Node("planning_publisher")
    {
        // declare parameters
        this->declare_parameter("callback_period_ms", T_PLANNING_PUB);
        callback_period_ms_ = this->get_parameter("callback_period_ms").as_int();

        // Publisher
        planning_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
            "planning_target", T_PLANNING_PUB);

        // planning loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(callback_period_ms_),
            std::bind(&PlanningNode::planning_callback, this));
    }

  private:
    void planning_callback()
    {
        target_ = point_target();

        // --- Publish command ---
        geometry_msgs::msg::Point msg;
        msg.x = target_.x;
        msg.y = target_.y;
        msg.z = 0.0;
        planning_publisher_->publish(msg);
    }

    // ROS2 interfaces
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr planning_publisher_;
    int callback_period_ms_;

    // ref
    Point target_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanningNode>());
    rclcpp::shutdown();
    return 0;
}