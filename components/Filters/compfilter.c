#include "compfilter.h"

//outputs computed angles for complementary filter
void computeAngles(float accel[3], float mag[3], float angles[3]){
    //pitch
    angles[0] = atan2(accel[1], accel[2]);
    //roll
    angles[1] = atan2(-accel[0], sqrt(accel[1]*accel[2]+accel[2]*accel[2]));

    //compute yaw based on the pitch/roll to compensate for the magnetic field since 
    //measurement is different from standard gyro/accelerometer readings
    angles[2] = atan2(mag[0]*sin(angles[1]) - mag[1]*cos(angles[1]), mag[0]*cos(angles[0]) + 
                      sin(angles[0])*(mag[0]*sin(angles[1]) + mag[1]*cos(angles[1])));
}

//alpha is value in range [0, 1]
void computeCompFilter(float angles[3], float gyro[3], float alpha, float previous_orientation[3], float delta_t, float orientation[3]){
    //numerical integration of angular velocity w/previous orientation
    orientation[0] = previous_orientation[0] + gyro[0]*delta_t;
    orientation[1] = previous_orientation[1] + gyro[1]*delta_t;
    orientation[2] = previous_orientation[2] + gyro[2]*delta_t;

    for(int i=0;i<3;i++){
        orientation[i] = alpha*orientation[i] + (1-alpha)*angles[i];
    }
}

void complementary_filter(float accel[3], float gyro[3], float mag[3], float angular_vel[3], float delta_t){
    float angles[3];
    computeAngles(accel, mag, angles);

    float previous_orientation[3];
    float orientation[3] = {0.0f, 0.0f, 0.0f};
    float alpha = 0.5f;
    computeCompFilter(angles, gyro, alpha, previous_orientation, delta_t, orientation);

    for(int i=0;i<3;i++){
        angular_vel[i] = orientation[i];
        previous_orientation[i] = orientation[i];
    }
}
