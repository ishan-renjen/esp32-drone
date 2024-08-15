#define ACCEL_DT 0.01 //100 Hz
#define VEL_DT 0.001 //1 KHz
#define POS_DT 0.0001 //10 KHz

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float derivative_error;
} PIDController;

float PIDLoop(PIDController *pidData, float ideal, float actual, float dt);