#include <algorithm>
#include <cmath>

#include "applications/common_utils.hpp"

#define V_MAX 10.0
#define D_MAX M_PI / 4

CarControl point_follower(const CarState& state, const Point& target)
{
    CarControl control;
    //
    double dist = std::sqrt(
        (target.x - state.x) * (target.x - state.x) +
        (target.y - state.y) * (target.y - state.y));

    double desired_heading = std::atan2(target.y - state.y, target.x - state.x);

    double heading_error = desired_heading - state.yaw;
    while (heading_error > M_PI)
        heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI)
        heading_error += 2.0 * M_PI;

    control.d = 1.2 * heading_error;  // "P control" for the beggar
    control.d = std::clamp(control.d, -D_MAX, D_MAX);

    control.v = 0.6 * dist;  // proportional speed
    control.v = std::clamp(control.v, 0.0, V_MAX);

    // Stop when close
    if (dist < 0.05)
    {
        control.v = 0.0;
    }

    return control;
}

// pure pursuit
double compute_lookahead(double v)
{
    double L_min = 1.0;  // minimum lookahead in meters
    double k_v = 0.6;    // gain
    return L_min + k_v * v;
}

CarControl pure_pursuit(const CarState& state, const Point& target, double current_speed)
{
    CarControl control;

    // Compute rear-axle distance to target
    double dx = target.x - state.x;
    double dy = target.y - state.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < 1e-6)
        return {0.0, 0.0};

    // Pure pursuit formula uses lookahead point
    // If target is too close, artificially increase distance
    double Ld_temp = compute_lookahead(current_speed);
    double Ld = std::max(dist, Ld_temp);

    // Angle from rear axle to target
    double alpha = std::atan2(dy, dx) - state.yaw;

    // Normalize
    while (alpha > M_PI)
        alpha -= 2 * M_PI;
    while (alpha < -M_PI)
        alpha += 2 * M_PI;

    // Steering control (bicycle model)
    control.d = std::atan2(2.0 * L_WHEELBASE * std::sin(alpha), Ld);

    // Speed control
    control.v = std::clamp(1.5 * dist, 0.0, V_MAX);

    if (dist < 0.05)
        control.v = 0.0;

    return control;
}

// stanley
double compute_lateral_error(const CarState& state, const Point& target)
{
    double dx = target.x - state.x;
    double dy = target.y - state.y;

    // direction perpendicular to heading
    double nx = -std::sin(state.yaw);
    double ny = std::cos(state.yaw);

    return dx * nx + dy * ny;  // signed error
}

CarControl stanley(const CarState& state, const Point& target, double current_speed)
{
    CarControl control;

    // Vector to target
    double dx = target.x - state.x;
    double dy = target.y - state.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // Heading error
    double desired_heading = std::atan2(dy, dx);
    double heading_error = desired_heading - state.yaw;

    while (heading_error > M_PI)
        heading_error -= 2 * M_PI;
    while (heading_error < -M_PI)
        heading_error += 2 * M_PI;

    // Lateral error
    double e = compute_lateral_error(state, target);

    // Stanley controller parameters
    double k = 1.0;
    double eps = 0.1;

    // Avoid zero speed (wobbling)
    double v_for_control = std::max(current_speed, 0.3);

    double correction = std::atan2(k * e, v_for_control);

    control.d = heading_error + correction;

    // Clamp steering
    control.d = std::clamp(control.d, -D_MAX, D_MAX);

    // Simple proportional speed control
    control.v = std::clamp(1.2 * dist, 0.0, V_MAX);

    // Stop when very close
    if (dist < 0.05)
        control.v = 0.0;

    return control;
}
