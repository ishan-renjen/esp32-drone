#include "PIDControl.h"

float PIDLoop(PIDController *pidData, float ideal, float actual, float dt){
    float current_error = ideal - actual;
    pidData->integral += current_error*dt; //integrate error within bounds of 0 to dt
    float derivative = (current_error - pidData->derivative_error)/dt;
    pidData->derivative_error = current_error;
    return (pidData->kp*current_error) + (pidData->ki*pidData->integral)+(pidData->kd*derivative); 
}

float PIDLoop_quaternion(PIDController *pidData, float current_error, float dt){
    pidData->integral += current_error*dt; //integrate error within bounds of 0 to dt
    float derivative = (current_error - pidData->derivative_error)/dt;
    pidData->derivative_error = current_error;
    return (pidData->kp*current_error) + (pidData->ki*pidData->integral)+(pidData->kd*derivative); 
}

void QuatConjugate(float q[4], float result[4]){
    for(int i=0;i<4;i++){result[i] = q[i];}
    for(int i=1;i<4;i++){ result[i] *= -1;}
}

void QuatMultiply(float q1[4], float q2[4], float result[4]){
    float q2_conj[4];
    QuatConjugate(q2, q2_conj);
    result[0] = q1[0]*q2_conj[0] - q1[1]*q2_conj[1] - q1[2]*q2_conj[2] - q1[3]*q2_conj[3];
    result[1] = q1[0]*q2_conj[1] + q1[1]*q2_conj[0] + q1[2]*q2_conj[3] - q1[3]*q2_conj[2];
    result[2] = q1[0]*q2_conj[2] - q1[1]*q2_conj[3] + q1[2]*q2_conj[0] + q1[3]*q2_conj[1];
    result[3] = q1[0]*q2_conj[3] + q1[1]*q2_conj[2] - q1[2]*q2_conj[1] + q1[3]*q2_conj[0];
}

void QuatNormalize(float q[4], float result[4]){
    float normalize = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for(int i=0;i<4;i++){
        result[i] = q[i]/(normalize + 0.000000000000000000000000000000001f);
    }
}

extern void outerLoop(float q_setpoint[4], float q_actual[4], float v_ideal[3]){
    float quat_error[4];
    QuatMultiply(q_setpoint, q_actual, quat_error); //gives you error of quaternion for PID 

    //normalize error quaternion
    float quat_error_normalized[4];
    QuatNormalize(quat_error, quat_error_normalized);

    PIDController error_loop_pid = {1, 0.1, 0.01, 0.0, 0.0};

    //compute the desired velocities in xyz based on the normalized quaternion error
    for(int i=1; i<4;i++){
        v_ideal[i-1] = PIDLoop_quaternion(&error_loop_pid, quat_error_normalized[i], OUTER_DT);
    }
}

extern void innerLoop(float v_actual[3], float v_ideal[3], float torques[3]){
    PIDController velocity_loop_pid = {1, 0.1, 0.01, 0.0, 0.0};

    //compute torque vector in xyz
    for(int i=0;i<3;i++){
        torques[i] = PIDLoop(&velocity_loop_pid, v_ideal[i], v_actual[i], INNER_DT);
    }
}
extern void heightLoop(float *height_actual, float *height_ideal, float *throttle){
    PIDController height_loop_pid = {1, 0.1, 0.01, 0.0, 0.0};

    //compute throttle based on height PID 
    for(int i=0;i<3;i++){
        *throttle = PIDLoop(&height_loop_pid, *height_ideal, *height_actual, INNER_DT);
    }
}

extern void motorControl(float torques[4], float thrust[4], float *throttle){
     //front left, front right, rear left, rear right
    thrust[0] = *throttle + torques[0] + torques[1] - torques[2];
    thrust[1] = *throttle + torques[0] - torques[1] + torques[2];
    thrust[2] = *throttle - torques[0] + torques[1] + torques[2];
    thrust[3] = *throttle - torques[0] - torques[1] - torques[2];
}
