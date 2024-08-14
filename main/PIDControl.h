typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float derivative_error;
} PIDController;

float PIDLoop(PIDController *pidData, float ideal, float actual, float dt);