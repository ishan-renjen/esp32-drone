#include "PIDControl.h"

float PIDLoop(PIDController *pidData, float ideal, float actual, float dt){
    float current_error = ideal - actual;
    pidData->integral += current_error*dt; //integrate error within bounds of 0 to dt
    float derivative = (current_error - pidData->derivative_error)/dt;
    pidData->derivative_error = current_error;
    return (pidData->kp*current_error) + (pidData->ki*pidData->integral)+(pidData->kd*(pidData->derivative_error)); 
}


//first out of 3 loops, which computes desired velocity based on actual acceleration from IMU and desired acceleration from controller
void AccelPIDLoop(float actual[3], float desired[3], float velocity[3]){
    PIDController pidAccelX;
    pidAccelX.kp = 1;
    pidAccelX.ki = 0.1;
    pidAccelX.kd = 0.01;

    PIDController pidAccelY;
    pidAccelY.kp = 1;
    pidAccelY.ki = 0.1;
    pidAccelY.kd = 0.01;

    PIDController pidAccelZ;
    pidAccelZ.kp = 1;
    pidAccelZ.ki = 0.1;
    pidAccelZ.kd = 0.01;

    velocity[0] = PIDLoop(&pidAccelX, desired[0], actual[0], ACCEL_DT);
    velocity[1] = PIDLoop(&pidAccelY, desired[1], actual[1], ACCEL_DT);
    velocity[2] = PIDLoop(&pidAccelZ, desired[2], actual[2], ACCEL_DT);
}

//second out of 3 loops which computes desired pitch, roll, and yaw based on the output from the first PID stage and velocity from a complementary filter
void VelocityPIDLoop(float actual[3], float desired[3], float position[3]){
    PIDController pidVelX;
    pidVelX.kp = 1;
    pidVelX.ki = 0.1;
    pidVelX.kd = 0.01;

    PIDController pidVelY;
    pidVelY.kp = 1;
    pidVelY.ki = 0.1;
    pidVelY.kd = 0.01;

    PIDController pidVelZ;
    pidVelZ.kp = 1;
    pidVelZ.ki = 0.1;
    pidVelZ.kd = 0.01;

    position[0] = PIDLoop(&pidVelX, desired[0], actual[0], VEL_DT);
    position[1] = PIDLoop(&pidVelY, desired[1], actual[1], VEL_DT);
    position[2] = PIDLoop(&pidVelZ, desired[2], actual[2], VEL_DT);
}

//3rd loop out of 3, innermost loop. computes thrust of motors based on inputted euler angles and actual angles from madgwick filter
void PositionPIDLoop(float actual[3], float desired[3], float thrust[3]){
    PIDController pidPosX;
    pidPosX.kp = 1;
    pidPosX.ki = 0.1;
    pidPosX.kd = 0.01;

    PIDController pidPosY;
    pidPosY.kp = 1;
    pidPosY.ki = 0.1;
    pidPosY.kd = 0.01;

    PIDController pidPosZ;
    pidPosZ.kp = 1;
    pidPosZ.ki = 0.1;
    pidPosZ.kd = 0.01;

    thrust[0] = PIDLoop(&pidPosX, desired[0], actual[0], POS_DT);
    thrust[1] = PIDLoop(&pidPosY, desired[1], actual[1], POS_DT);
    thrust[2] = PIDLoop(&pidPosZ, desired[2], actual[2], POS_DT);
}

//controls entire PID process and sends out thrusts to each motor as a float array
//TODO: write complementary filter fusing gyro and accelerometer to get reliable velocity 
//TODO: compute thrusts per motor based on output of position stage
//TODO: consider an altitude loop as well
void PIDControlLoop(float accelData[3], float controllerAccel[3], float velocityData[3], float angles[3], float output[4]){
    float velocityComputed[3];
    float positionComputed[3];
    float thrustComputed[3];

    AccelPIDLoop(accelData, controllerAccel, velocityComputed);
    VelocityPIDLoop(velocityData, velocityComputed, positionComputed);
    PositionPIDLoop(angles, positionComputed, thrustComputed);
}