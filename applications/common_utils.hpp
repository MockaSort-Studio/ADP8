#ifndef APPLICATIONS_COMMON_UTILS
#define APPLICATIONS_COMMON_UTILS

#define L_WHEELBASE 2.5f  // m, inter-axle length
#define V_MAX 10.0f       // m/s, max longitudinal speed
#define D_MAX 0.785375f   // rad, max steering angle pi/4
#define MOTOR_TAU 0.2f    // s, time constant of DC motor
#define SERVO_SPEED 1.5f  // rad/s, servo motor speed

struct Point
{
    double x;
    double y;
};

struct VehicleState
{
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
    float v = 0.0f;  // longitudinal velocity
    float d = 0.0f;  // steering angle
};

struct ActuatorState
{
    float motor_speed = 0.0f;     // m/s
    float steering_angle = 0.0f;  // rad
};

struct CarControl
{
    float v = 0.0;
    float d = 0.0;
};

#endif  // APPLICATIONS_COMMON_UTILS
