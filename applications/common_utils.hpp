#ifndef APPLICATIONS_COMMON_UTILS
#define APPLICATIONS_COMMON_UTILS

struct CarState
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double d = 0.0;  // steering angle (input)
};

#define L_WHEELBASE 2.5  // meters

#endif  // APPLICATIONS_COMMON_UTILS
