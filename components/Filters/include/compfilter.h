#include "math.h"

//https://ahrs.readthedocs.io/en/latest/filters/complementary.html

extern void complementary_filter(float accel[3], float gyro[3], float mag[3], float angular_vel[3], float delta_t);