#include "math.h"

#define OUTER_DT 0.00028 //3.6 KHz
#define INNER_DT 0.00005 //18 KHz

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float derivative_error;
} PIDController;

extern void outerLoop(float q_setpoint[4], float q_actual[4], float v_actual[3]);
extern void innerLoop(float v_actual[3], float v_ideal[3], float torques[3]);
extern void heightLoop(float *height_actual, float *height_ideal, float *throttle);
extern void motorControl(float torques[3], float thrust[4], float *throttle);