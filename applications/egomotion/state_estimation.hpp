#ifndef APPLICATIONS_EGOMOTION_STATE_ESTIMATION
#define APPLICATIONS_EGOMOTION_STATE_ESTIMATION

// messages
#include "car_msgs/msg/car_state.hpp"
#include "car_msgs/msg/imu_raw.hpp"
#include "car_msgs/msg/steering_angle_mes.hpp"

car_msgs::msg::CarState state_update(
    car_msgs::msg::IMURaw imu_msg,
    car_msgs::msg::CarState car_state_msg,
    car_msgs::msg::SteeringAngleMes d_msg,
    float dt);

#endif  // APPLICATIONS_EGOMOTION_STATE_ESTIMATION
