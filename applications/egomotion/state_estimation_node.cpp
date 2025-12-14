
// state estimation node
/*
Sub
    imu_raw
    steering_angle_measured
Pub
    state_est

*/

#include <cmath>

// application
#include "applications/applications_param.hpp"
#include "applications/common_utils.hpp"
// messages
#include "car_msgs/msg/car_state.hpp"
#include "car_msgs/msg/imu_raw.hpp"
#include "car_msgs/msg/steering_angle_mes.hpp"
// ROS2
#include "core/tasks_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "support/lookup_table.hpp"

class StateEstimationNode : public sert::core::TaskInterface
{
  public:
    StateEstimationNode(const std::string& name, rclcpp::NodeOptions options)
        : TaskInterface(
              name,
              options.append_parameter_override("cycle_time_ms", T_STATEESTIMATION_PUB))
    {
        RegisterSubscriber<car_msgs::msg::IMURaw>(
            "imu_raw",
            [this](car_msgs::msg::IMURaw::UniquePtr msg)
            {
                imu_msg_ = *msg;
                imu_received_ = true;
            });

        RegisterSubscriber<car_msgs::msg::SteeringAngleMes>(
            "steering_angle_measured",
            [this](car_msgs::msg::SteeringAngleMes::UniquePtr msg)
            { steering_angle_ = msg->d; });

        RegisterPublisher<car_msgs::msg::CarState>("state_est", QS_STATEESTIMATION_PUB);

        resetState();
    }

  protected:
    void ExecuteStep() override
    {
        if (!imu_received_)
        {
            return;
        }

        predict();
        publishState();
    }

  private:
    static constexpr float dt = T_STATEESTIMATION_PUB * 1e-3f;

    // state
    float x_;
    float y_;
    float yaw_;
    float v_;

    float steering_angle_;

    car_msgs::msg::IMURaw imu_msg_;
    bool imu_received_ = false;

    // TODO: move predict() outside from the node class
    // TODO: add KF to remove noise
    void predict()
    {
        float yaw_rate = imu_msg_.gyro_z;
        float acc_long = imu_msg_.acc_x;

        yaw_ += yaw_rate * dt;
        yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));
        v_ += acc_long * dt;
        x_ += v_ * std::cos(yaw_) * dt;
        y_ += v_ * std::sin(yaw_) * dt;
    }

    // node methods can be left here
    void publishState()
    {
        car_msgs::msg::CarState msg;
        msg.x = x_;
        msg.y = y_;
        msg.yaw = yaw_;
        msg.d = steering_angle_;

        GetPublisher<car_msgs::msg::CarState>("state_est")->publish(msg);
    }

    void resetState()
    {
        x_ = 0.0f;
        y_ = 0.0f;
        yaw_ = 0.0f;
        v_ = 0.0f;
        steering_angle_ = 0.0f;
    }
};

using StateEstimationConfig = sert::support::LookupTable<
    sert::support::TableItem<StateEstimationNode, sert::support::UnusedValue>>;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto task_manager = sert::core::BuildTasksManager<StateEstimationConfig>();
    task_manager->Execute();
    rclcpp::shutdown();
    return 0;
}
