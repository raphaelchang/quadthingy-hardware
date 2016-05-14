#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "bno055.h"

void controller_init(void);
void controller_update(double throttle);
void velocity_loop_update(vector3 setpoint, double throttle);

#endif /* CONTROLLER_H_ */