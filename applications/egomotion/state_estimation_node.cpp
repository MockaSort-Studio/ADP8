
// state estimation node

#include <cmath>

// application
#include "applications/applications_param.hpp"
#include "applications/common_utils.hpp"
#include "applications/egomotion/state_estimation.hpp"
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
            [this](car_msgs::msg::SteeringAngleMes::UniquePtr msg) { d_msg_ = *msg; });

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

        car_state_msg_ = state_update(imu_msg_, car_state_msg_, d_msg_, dt);
        publishState();
    }

  private:
    static constexpr float dt = T_STATEESTIMATION_PUB * 1e-3f;

    // state
    car_msgs::msg::CarState car_state_msg_;
    car_msgs::msg::IMURaw imu_msg_;
    car_msgs::msg::SteeringAngleMes d_msg_;

    // flags
    bool imu_received_ = false;

    // node methods can be left here
    void publishState()
    {
        GetPublisher<car_msgs::msg::CarState>("state_est")->publish(car_state_msg_);
    }

    void resetState()
    {
        car_state_msg_.x = 0.0f;
        car_state_msg_.y = 0.0f;
        car_state_msg_.yaw = 0.0f;
        car_state_msg_.v = 0.0f;
        car_state_msg_.d = 0.0f;
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
