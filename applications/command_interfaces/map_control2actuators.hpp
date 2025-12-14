#ifndef APPLICATIONS_COMMAND_INTERFACES_MAP_CONTROL2ACTUATORS
#define APPLICATIONS_COMMAND_INTERFACES_MAP_CONTROL2ACTUATORS

#include "car_msgs/msg/actuator_commands.hpp"
#include "car_msgs/msg/car_control.hpp"

car_msgs::msg::ActuatorCommands mapControlToActuators(
    car_msgs::msg::CarControl ctrl_msg_);

#endif  // APPLICATIONS_COMMAND_INTERFACES_MAP_CONTROL2ACTUATORS
