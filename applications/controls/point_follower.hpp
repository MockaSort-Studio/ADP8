#ifndef APPLICATIONS_CONTROLS_POINT_FOLLOWER
#define APPLICATIONS_CONTROLS_POINT_FOLLOWER

#include "applications/common_utils.hpp"

// Simple proportional point follower
CarControl point_follower(const VehicleState& state, const Point& target);

// Pure pursuit controller
CarControl pure_pursuit(
    const VehicleState& state, const Point& target, float current_speed);

// Stanley controller
CarControl stanley(const VehicleState& state, const Point& target, float current_speed);

#endif  // APPLICATIONS_CONTROLS_POINT_FOLLOWER
