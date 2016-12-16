#include "controller.h"
#include "ch.h"
#include "hal.h"
#include "bno055.h"
#include "esc.h"
#include <Eigen/Core>

vector3 orientation;
vector3 rotation;
vector3 vel_p_gains;
vector3 vel_i_gains;
vector3 vel_d_gains;
vector3 vel_ff_gains;
vector3 pos_p_gains;
vector3 pos_i_gains;
vector3 pos_d_gains;

void controller_init(void)
{
    vel_p_gains.x = 0.005;
    vel_p_gains.y = 0.005;
    vel_p_gains.z = -0.005;
    vel_i_gains.x = 0;
    vel_i_gains.y = 0;
    vel_i_gains.z = 0;
    vel_d_gains.x = 0;
    vel_d_gains.y = 0;
    vel_d_gains.z = 0;
    vel_ff_gains.x = 0.05;
    vel_ff_gains.y = 0.05;
    vel_ff_gains.z = 0.01;
    pos_p_gains.x = -0.02;
    pos_p_gains.y = -0.02;
    pos_p_gains.z = 0.01;
    pos_i_gains.x = 0;
    pos_i_gains.y = 0;
    pos_i_gains.z = 0;
    pos_d_gains.x = 0;
    pos_d_gains.y = 0;
    pos_d_gains.z = 0;
}

void controller_update(double throttle)
{
    orientation = bno055_get_vector(VECTOR_EULER);
    rotation = bno055_get_vector(VECTOR_GYROSCOPE);
    vector3 setpoint;
    setpoint.x = 0.0;
    setpoint.y = 0.0;
    setpoint.z = 0.0;
    double errX = setpoint.x - orientation.x; // pitch error
    double errY = setpoint.y - orientation.y; // roll error
    double errZ = setpoint.z - orientation.z; // yaw error
    setpoint.x = errX * pos_p_gains.x;
    setpoint.y = errY * pos_p_gains.y;
    setpoint.z = 0.0;//errZ * pos_p_gains.z;
    velocity_loop_update(setpoint, throttle);
}

void velocity_loop_update(vector3 setpoint, double throttle)
{
    double errX = setpoint.x - rotation.x; // pitch error
    double errY = setpoint.y - rotation.y; // roll error
    double errZ = setpoint.z - rotation.z; // yaw error
    double pitchOut = errX * vel_p_gains.x + setpoint.x * vel_ff_gains.x;
    double rollOut = errY * vel_p_gains.y + setpoint.y * vel_ff_gains.y;
    double yawOut = errZ * vel_p_gains.z + setpoint.z * vel_ff_gains.z;
    esc_set(0, throttle + pitchOut - yawOut);
    esc_set(1, throttle - rollOut + yawOut);
    esc_set(2, throttle - pitchOut - yawOut);
    esc_set(3, throttle + rollOut + yawOut);
}
