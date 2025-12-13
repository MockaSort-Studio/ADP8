#ifndef APPLICATIONS_COMMON_UTILS
#define APPLICATIONS_COMMON_UTILS

#define L_WHEELBASE 2.5  // meters

struct Point
{
    double x;
    double y;
};

struct CarState
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double d = 0.0;  // steering angle (input)
};

struct CarControl
{
    double v = 0.0;
    double d = 0.0;
};

#endif  // APPLICATIONS_COMMON_UTILS
