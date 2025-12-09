#ifndef APPLICATIONS_CONTROLS_POINT_FOLLOWER
#define APPLICATIONS_CONTROLS_POINT_FOLLOWER

CarControl point_follower(const CarState& state, const Point& target);
CarControl stanley(const CarState& state, const Point& target, double current_speed);
CarControl pure_pursuit(const CarState& state, const Point& target, double lookahead);

#endif  // APPLICATIONS_CONTROLS_POINT_FOLLOWER
