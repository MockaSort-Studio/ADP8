#include "state_update.hpp"

#include <cmath>

car_msgs::msg::CarState state_update(
    car_msgs::msg::CarState state, car_msgs::msg::CarControl cmd)
{
    double dt = T_CARSTATE_PUB / 1000.0;

    state.d = cmd.d;

    state.x += cmd.v * std::cos(state.yaw) * dt;
    state.y += cmd.v * std::sin(state.yaw) * dt;
    state.yaw += cmd.v * std::tan(state.d) / L_WHEELBASE * dt;

    return state;
}
