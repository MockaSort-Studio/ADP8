
#include <algorithm>
#include <cmath>
// application
#include "applications/command_interfaces/map_control2actuators.hpp"
#include "applications/common_utils.hpp"
// msgs
#include "car_msgs/msg/actuator_commands.hpp"
#include "car_msgs/msg/car_control.hpp"

car_msgs::msg::ActuatorCommands mapControlToActuators(car_msgs::msg::CarControl ctrl_msg)
{
    car_msgs::msg::ActuatorCommands act_cmd_msg;
    // velocity
    float v = std::clamp(ctrl_msg.v, -V_MAX, V_MAX);
    act_cmd_msg.dc_direction = (v >= 0.0f) ? 1.0f : -1.0f;
    act_cmd_msg.dc_pwm = std::abs(v) / V_MAX;

    // steering
    act_cmd_msg.servo_angle = std::clamp(ctrl_msg.d, -D_MAX, D_MAX);
    act_cmd_msg.servo_speed = SERVO_SPEED;

    return act_cmd_msg;
}

// TODO: different cars will be handled here differently
