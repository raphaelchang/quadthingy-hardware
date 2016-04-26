#include "controller.h"
#include "ch.h"
#include "hal.h"
#include "bno055.h"
#include "esc.h"

vector3 orientation;
vector3 rotation;
vector3 p_gains;
vector3 i_gains;
vector3 d_gains;

void controller_init(void)
{
	p_gains.x = 1;
	p_gains.y = 1;
	p_gains.z = 1;
	i_gains.x = 0;
	i_gains.y = 0;
	i_gains.z = 0;
	d_gains.x = 0;
	d_gains.y = 0;
	d_gains.z = 0;
}

void controller_update(void)
{
	orientation = bno055_get_vector(VECTOR_EULER);
    rotation = bno055_get_vector(VECTOR_GYROSCOPE);
    vector3 setpoint;
    setpoint.x = 0.0;
    setpoint.y = 0.0;
    setpoint.z = 0.0;
    velocity_loop_update(setpoint, 0.0);
}

void velocity_loop_update(vector3 setpoint, double throttle)
{
	double errX = setpoint.x - rotation.x;
	double errY = setpoint.y - rotation.y;
	double errZ = setpoint.z - rotation.z;
}