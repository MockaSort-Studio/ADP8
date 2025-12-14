
#include <cmath>

#include "applications/common_utils.hpp"
//
#include "car_msgs/msg/actuator_commands.hpp"
#include "car_msgs/msg/car_control.hpp"
#include "car_msgs/msg/car_state.hpp"
#include "car_msgs/msg/imu_raw.hpp"
#include "car_msgs/msg/steering_angle_mes.hpp"
//
#include "simulation/sim_param.hpp"
//
#include "core/tasks_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "support/lookup_table.hpp"

class CarSimulationNode : public sert::core::TaskInterface
{
  public:
    CarSimulationNode(const std::string& name, rclcpp::NodeOptions options)
        : TaskInterface(
              name, options.append_parameter_override("cycle_time_ms", T_SIMULATION_PUB))
    {
        RegisterPublisher<car_msgs::msg::IMURaw>("imu_raw", QS_SIMULATION_PUB);
        RegisterPublisher<car_msgs::msg::SteeringAngleMes>(
            "steering_angle_measured", QS_SIMULATION_PUB);
        RegisterPublisher<car_msgs::msg::CarState>("state_gt", QS_SIMULATION_PUB);

        RegisterSubscriber<car_msgs::msg::ActuatorCommands>(
            "actuator_cmd",
            [this](car_msgs::msg::ActuatorCommands::UniquePtr msg)
            {
                cmd_msg_.dc_pwm = msg->dc_pwm;
                cmd_msg_.dc_direction = msg->dc_direction;
                cmd_msg_.servo_angle = msg->servo_angle;
                cmd_msg_.servo_speed = msg->servo_speed;
            });
    }

  protected:
    void ExecuteStep() override
    {
        updateActuators();
        updateVehicle();
        updateSensors();
        publishMessages();
    }

  private:
    // internal state
    VehicleState veh_state_;
    ActuatorState act_state_;
    float prev_v_;

    // messages
    car_msgs::msg::IMURaw imu_msg_;
    car_msgs::msg::SteeringAngleMes d_mes_msg_;
    car_msgs::msg::CarState state_gt_msg_;
    car_msgs::msg::ActuatorCommands cmd_msg_;

    void publishMessages()
    {
        GetPublisher<car_msgs::msg::IMURaw>("imu_raw")->publish(imu_msg_);

        GetPublisher<car_msgs::msg::SteeringAngleMes>("steering_angle_measured")
            ->publish(d_mes_msg_);

        GetPublisher<car_msgs::msg::CarState>("state_gt")->publish(state_gt_msg_);
    }

    // TODO: move these functions outside this class
    // TODO: add noise to sensors
    void updateActuators()
    {
        // DC motor (first-order system)
        float v_target = cmd_msg_.dc_pwm * V_MAX * cmd_msg_.dc_direction;

        act_state_.motor_speed += (DT / MOTOR_TAU) * (v_target - act_state_.motor_speed);

        // Servo motor
        float d_error = cmd_msg_.servo_angle - act_state_.steering_angle;
        float max_delta = cmd_msg_.servo_speed * DT;
        float delta = std::clamp(d_error, -max_delta, max_delta);

        act_state_.steering_angle += delta;
    }

    void updateVehicle()
    {
        float v = act_state_.motor_speed;
        float d = act_state_.steering_angle;

        veh_state_.x += v * std::cos(veh_state_.yaw) * DT;
        veh_state_.y += v * std::sin(veh_state_.yaw) * DT;
        veh_state_.yaw += (v / L_WHEELBASE) * std::tan(d) * DT;

        veh_state_.v = v;
        veh_state_.d = d;
    }

    void updateSensors()
    {
        //  gyroscope
        float yaw_rate = (veh_state_.v / L_WHEELBASE) * std::tan(veh_state_.d);

        imu_msg_.gyro_x = 0.0f;
        imu_msg_.gyro_y = 0.0f;
        imu_msg_.gyro_z = yaw_rate;

        //  accelerometer in body frame
        float a_long = (veh_state_.v - prev_v_) / DT;
        float a_lat = veh_state_.v * yaw_rate;

        imu_msg_.acc_x = a_long;
        imu_msg_.acc_y = a_lat;
        imu_msg_.acc_z = 9.81f;  // gravity

        prev_v_ = veh_state_.v;

        //  steering angle sensor
        d_mes_msg_.d = act_state_.steering_angle;

        //  GT
        state_gt_msg_.x = veh_state_.x;
        state_gt_msg_.y = veh_state_.y;
        state_gt_msg_.yaw = veh_state_.yaw;
        state_gt_msg_.d = veh_state_.d;
    }
};

using CarSimulationConfig = sert::support::LookupTable<
    sert::support::TableItem<CarSimulationNode, sert::support::UnusedValue>>;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto task_manager = sert::core::BuildTasksManager<CarSimulationConfig>();
    task_manager->Execute();
    rclcpp::shutdown();
    return 0;
}
