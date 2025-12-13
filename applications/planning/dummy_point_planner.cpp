#include <cmath>

#include "applications/common_utils.hpp"

Point point_target()
{
    Point target;
    target.x = 10;
    target.y = 20;
    return target;
}

Point figure8_traj(double t)
{
    Point target;

    double A = 10.0;
    double omega = 0.2;

    target.x = A * std::sin(omega * t);
    target.y = A * std::sin(2.0 * omega * t) / 2.0 + t / 10;

    return target;
}
