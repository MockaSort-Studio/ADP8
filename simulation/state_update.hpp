#ifndef SIMULATION_STATE_UPDATE
#define SIMULATION_STATE_UPDATE

#include "applications/common_utils.hpp"
#include "car_msgs/msg/car_control.hpp"
#include "car_msgs/msg/car_state.hpp"
#include "simulation/sim_param.hpp"

car_msgs::msg::CarState state_update(
    car_msgs::msg::CarState state, car_msgs::msg::CarControl cmd);

#endif  // SIMULATION_STATE_UPDATE
