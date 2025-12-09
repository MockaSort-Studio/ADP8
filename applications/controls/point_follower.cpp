#include <algorithm>
#include <cmath>

#include "applications/common_utils.hpp"

CarControl point_follower(const CarState& state, const Point& target)
{
    CarControl control;
    //
    double dist = std::sqrt(
        (target.x - state.x) * (target.x - state.x) +
        (target.y - state.y) * (target.y - state.y));

    double x_f = state.x + L_WHEELBASE * std::cos(state.yaw);
    double y_f = state.y + L_WHEELBASE * std::sin(state.yaw);
    double desired_heading = std::atan2(target.y - y_f, target.x - x_f);

    double heading_error = desired_heading - state.yaw;
    while (heading_error > M_PI)
        heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI)
        heading_error += 2.0 * M_PI;

    control.d = 1.2 * heading_error;  // "P control" for the beggar
    double d_max = M_PI / 4;
    control.d = std::clamp(control.d, -d_max, d_max);

    control.v = 0.6 * dist;  // proportional speed
    double v_max = 5.0;
    control.v = std::clamp(control.v, 0.0, v_max);

    // Stop when close
    if (dist < 0.05)
    {
        control.v = 0.0;
    }

    return control;
}