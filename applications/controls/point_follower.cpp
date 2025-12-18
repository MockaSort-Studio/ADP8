#include <algorithm>
#include <cmath>

#include "applications/common_utils.hpp"

static inline float wrapAngle(float a)
{
    while (a > M_PI)
        a -= 2.0f * M_PI;
    while (a < -M_PI)
        a += 2.0f * M_PI;
    return a;
}

CarControl point_follower(const VehicleState& state, const Point& target)
{
    CarControl control;

    float dx = static_cast<float>(target.x) - state.x;
    float dy = static_cast<float>(target.y) - state.y;

    float dist = std::sqrt(dx * dx + dy * dy);

    float desired_heading = std::atan2(dy, dx);
    float heading_error = wrapAngle(desired_heading - state.yaw);

    // Steering: simple P controller
    control.d = 1.2f * heading_error;
    control.d = std::clamp(control.d, -D_MAX, D_MAX);

    // Speed: proportional to distance
    control.v = 0.6f * dist;
    control.v = std::clamp(control.v, 0.0f, V_MAX);

    // Stop when close
    if (dist < 0.05f)
    {
        control.v = 0.0f;
    }

    return control;
}

static inline float compute_lookahead(float v)
{
    constexpr float L_MIN = 1.0f;  // meters
    constexpr float K_V = 0.6f;
    return L_MIN + K_V * v;
}

CarControl pure_pursuit(
    const VehicleState& state, const Point& target, float current_speed)
{
    CarControl control;

    float dx = static_cast<float>(target.x) - state.x;
    float dy = static_cast<float>(target.y) - state.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 1e-6f)
    {
        return control;  // zero command
    }

    float Ld = std::max(dist, compute_lookahead(current_speed));

    float alpha = wrapAngle(std::atan2(dy, dx) - state.yaw);

    // Bicycle-model pure pursuit steering law
    control.d = std::atan2(2.0f * L_WHEELBASE * std::sin(alpha), Ld);

    control.d = std::clamp(control.d, -D_MAX, D_MAX);

    // Speed control
    control.v = std::clamp(1.5f * dist, 0.0f, V_MAX);

    if (dist < 0.05f)
    {
        control.v = 0.0f;
    }

    return control;
}

static inline float compute_lateral_error(const VehicleState& state, const Point& target)
{
    float dx = static_cast<float>(target.x) - state.x;
    float dy = static_cast<float>(target.y) - state.y;

    // Unit normal to vehicle heading
    float nx = -std::sin(state.yaw);
    float ny = std::cos(state.yaw);

    return dx * nx + dy * ny;  // signed lateral error
}

CarControl stanley(const VehicleState& state, const Point& target, float current_speed)
{
    CarControl control;

    float dx = static_cast<float>(target.x) - state.x;
    float dy = static_cast<float>(target.y) - state.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    float desired_heading = std::atan2(dy, dx);
    float heading_error = wrapAngle(desired_heading - state.yaw);

    float e_lat = compute_lateral_error(state, target);

    // Stanley parameters
    constexpr float K = 1.0f;
    constexpr float MIN_SPEED = 0.3f;

    float v_for_control = std::max(current_speed, MIN_SPEED);

    float correction = std::atan2(K * e_lat, v_for_control);

    control.d = heading_error + correction;
    control.d = std::clamp(control.d, -D_MAX, D_MAX);

    // Speed control
    control.v = std::clamp(1.2f * dist, 0.0f, V_MAX);

    if (dist < 0.05f)
    {
        control.v = 0.0f;
    }

    return control;
}
