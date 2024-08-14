#include "PIDControl.h"

float PIDLoop(PIDController *pidData, float ideal, float actual, float dt){
    float current_error = ideal - actual;
    pidData->integral += current_error*dt; //integrate error within bounds of 0 to dt
    float derivative = (current_error - pidData->derivative_error);
    pidData->derivative_error = current_error;
    return (pidData->kp*current_error) + (pidData->ki*pidData->integral)+(pidData->kd*(pidData->derivative_error/dt)); 
}