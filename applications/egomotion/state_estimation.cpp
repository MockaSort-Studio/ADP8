
#include "applications/egomotion/state_estimation.hpp"

car_msgs::msg::CarState state_update(
    car_msgs::msg::IMURaw imu_msg,
    car_msgs::msg::CarState car_state_msg,
    car_msgs::msg::SteeringAngleMes d_msg,
    float dt)
{
    float yaw_rate = imu_msg.gyro_z;
    float acc_long = imu_msg.acc_x;

    car_state_msg.yaw += yaw_rate * dt;
    car_state_msg.yaw =
        std::atan2(std::sin(car_state_msg.yaw), std::cos(car_state_msg.yaw));
    car_state_msg.v += acc_long * dt;
    car_state_msg.x += car_state_msg.v * std::cos(car_state_msg.yaw) * dt;
    car_state_msg.y += car_state_msg.v * std::sin(car_state_msg.yaw) * dt;

    car_state_msg.d = d_msg.d;

    return car_state_msg;
}