#include "ch.h"
#include "hal.h"
#include "bno055.h"
#include "esc.h"

void controller_update(void)
{
	vector3 orientation = bno055_get_vector(VECTOR_EULER);
    vector3 rotation = bno055_get_vector(VECTOR_GYROSCOPE);
}