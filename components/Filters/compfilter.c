#include "compfilter.h"

void complementary_filter(float accel[3], float gyro[3], float mag[3], float angular_vel[3]){
    
}

//alpha = positive num -> [0, 1) -> alpha = dt / (RC + dt)
float lowpass(float previousVal, float currentVal, float dt, float RC){
    float alpha = dt / (RC + dt);
    return alpha*previousVal + (1-alpha)*currentVal;
}

//alpha = positive num -> [0, 1) -> alpha = RC / (RC + dt)
float highpass(float previousVal, float currentVal, float dt, float RC){
    float alpha = RC / (RC + dt);
    return alpha*previousVal + alpha*(currentVal - previousVal);
}

float balanceValues(float bias_low, float low, float high){
    
}