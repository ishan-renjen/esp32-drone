#include "math.h"

//source: https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf

//constants from paper
#define DELTAT .001f //1 KHz sample rate
#define GYRO_MEAS_ERROR 3.14159265358979f *(5.0f/180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define BETA sqrt(3.0f/4.0f) * GYRO_MEAS_ERROR //beta value needed for simplified gradient calculation
#define GYRO_MEAS_DRIFT 3.14159265358979 * (0.2f/180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define ZETA sqrt(3.0f/4.0f) * GYRO_MEAS_DRIFT // compute zeta for gyro gain computation

void Q_est(float *accel[3], float *gyro[3], float *mag[3], float quat_result[4]);