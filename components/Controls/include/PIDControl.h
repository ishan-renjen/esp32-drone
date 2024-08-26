#include "math.h"

#define OUTER_DT 0.01 //100 Hz
#define INNER_DT 0.001 //1 KHz

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float derivative_error;
} PIDController;

extern void ControlLoop(float q_setpoint[4], float q_actual[4], float v_actual[3], float thrust[4], float *throttle);