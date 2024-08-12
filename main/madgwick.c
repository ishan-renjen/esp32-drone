#include "madgwick.h"

void Q_est(float accel[3], float gyro[3], float mag[3], float quat_result[4]){
    //estimated orientation quaternion w/initial conditions setting q1 to a real value
    //future iterations will use this as previous estimation
    float q_se[4] = {1, 0, 0 ,0};
    //reference direction for flux in Earth frame -- TODO: understand this beter
    float bx = 1, bz = 0;

    float normalized;
    //normalize the accelerometer and magnetometer vectors - divide each item by sqrt(vector)
    //accelerometer
    normalized = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[1]*accel[1]);
    for(int i=0; i<3;i++){ accel[i] /= normalized; }

    //magnetometer
    normalized = sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
    for(int i=0; i<3;i++){ mag[i] /= normalized; }

    //compute the objective function and the jacobian
    //objective function - items 1-3 come from eqn. 25, items 4-6 come from eqn. 29
    float objective_func[6];
    objective_func[0] = 2*q_se[1]*q_se[3] - q_se[0]*q_se[2] - accel[0];
    objective_func[1] = 2*q_se[0]*q_se[1] + q_se[2]*q_se[3] - accel[1];
    objective_func[2] = 1.0f - 2*q_se[1]*q_se[1] - 2*q_se[2]*q_se[2] - accel[2];

    objective_func[3] = bx - (q_se[2]*q_se[2] - q_se[3]*q_se[3]) + (2.0f*bz*q_se[1]*q_se[3] - 2.0f*bz*q_se[0]*q_se[2]) - mag[0];
    objective_func[4] = (2.0f*bx*q_se[1]*q_se[2] - 2.0f*bx*q_se[0]*q_se[3]) + (2.0f*bz*q_se[0]*q_se[1] + 2.0f*bz*q_se[2]*q_se[3]) - mag[1];
    objective_func[5] = (2.0f*bx*q_se[0]*q_se[2]+2.0f*bx*q_se[1]*q_se[3]) + (bz - 2.0f*bz*q_se[1]*q_se[1] - 2.0f*bz*q_se[2]*q_se[2]- mag[2]);

    //jacobian - rows 1-3 come from eqn. 26, rows 4-6 come from eqn. 30 -TODO: transpose this
    float jacobian[6][4] = {{-2*q_se[2],                     2*q_se[3],                   -2*q_se[0],                     2*q_se[1]                     },
                            {2*q_se[1],                      2*q_se[0],                   2*q_se[3],                      2*q_se[2]                     },
                            {0,                              -4*q_se[1],                  -4*q_se[2],                     0                             },
                            {-2*bz*q_se[2],                  2*bz*q_se[3],                (-4*bx*q_se[2] - 2*bz*q_se[0]), (-4*bx*q_se[3] + 2*bz*q_se[1])},
                            {(-2*bx*q_se[3] + 2*bz*q_se[1]), (2*bx*q_se[2]+2*bz*q_se[0]), (2*bx*q_se[1]+2*bz*q_se[3]),    (-2*bx*q_se[0]+2*bz*q_se[2])  },
                            {2*bx*q_se[2],                   (2*bx*q_se[3]-4*bz*q_se[1]), (2*bx*q_se[0]-4*bz*q_se[2]),    2*bx*q_se[1]                  }}; 

    //compute gradient by multiplying objective row vector and jacobian, then normalize to get gradient unit vector - eqn. 44, carrying back to eqn. 33
    float gradient[4];
    //compute gradient
    for(int i=0;i<4;i++){
        float temp = 0;
        for(int j=0;j<6;j++){
            temp += objective_func[j]*jacobian[j][i];
        }
        gradient[i] = temp;
    }

    //normalize gradient
    normalized = sqrt(gradient[0]*gradient[0]+gradient[1]*gradient[1]+gradient[2]*gradient[2]*gradient[3]*gradient[3]);
    for(int i=0;i<4;i++){ gradient[i] /= normalized; }

    //gyro bias error - eqn. 47
    float gyro_error[3] = {0, 0, 0}; 
    gyro_error[0] = q_se[0]*gradient[1] - q_se[1]*gradient[0] - q_se[2]*gradient[3] + q_se[3]*gradient[2];
    gyro_error[1] = q_se[0]*gradient[2] + q_se[1]*gradient[3] - q_se[2]*gradient[0] - q_se[3]*gradient[1];
    gyro_error[2] = q_se[0]*gradient[3] - q_se[1]*gradient[2] + q_se[2]*gradient[1] - q_se[3]*gradient[0];

    //calculate and remove gyro biases - eqns. 48 and 49
    //calculate biases with use of zeta - rate of convergence to remove gyro errors > E(0) and time, essentially an integral over deltat weighted by zeta
    float gyro_bias[3];
    for(int i=0; i<3;i++){
        gyro_bias[i] = gyro_error[i]*DELTAT*ZETA;
    }
    //remove gyro biases
    for(int i=0;i<3;i++){
        gyro[i] -= gyro_bias[i];
    }

    //compute rate of change of quaternion measure by gyro post-corrections - look at Fig. 3
    float gyro_quat_rate[4]; 
    gyro_quat_rate[0] = -0.5*q_se[1]*gyro[0] - 0.5*q_se[2]*gyro[1] - 0.5*q_se[3]*gyro[2];
    gyro_quat_rate[1] = 0.5*q_se[0]*gyro[0] + 0.5*q_se[2]*gyro[2] - 0.5*q_se[3]*gyro[1];
    gyro_quat_rate[2] = 0.5*q_se[0]*gyro[1] - 0.5*q_se[1]*gyro[2] + 0.5*q_se[3]*gyro[0];
    gyro_quat_rate[3] = 0.5*q_se[0]*gyro[2] + 0.5*q_se[1]*gyro[2] - 0.5*q_se[2]*gyro[0];

    //compute q_est then integrate it - look at Fig. 3
    q_se[0] += gyro_quat_rate[0] - gradient[0]*BETA*DELTAT;
    q_se[1] += gyro_quat_rate[1] - gradient[1]*BETA*DELTAT;
    q_se[2] += gyro_quat_rate[2] - gradient[2]*BETA*DELTAT;
    q_se[3] += gyro_quat_rate[3] - gradient[3]*BETA*DELTAT;

    //normalize resulting quaternion to get final output array
    normalized = sqrt(q_se[0]*q_se[0] + q_se[1]*q_se[1] + q_se[2]*q_se[2] + q_se[3]*q_se[3]);
    for(int i=0;i<3;i++){
        q_se[i] /= normalized;
        quat_result[i] = q_se[i];
    }
    
    float hx, hy, hz; //computed flux in earth frame - used to recompute flux for next iterations
    hx = 2*mag[0]*(0.5-q_se[2]*q_se[2]-q_se[3]*q_se[3]) + 2*mag[1]*(q_se[1]*q_se[2]-q_se[0]*q_se[3]) + 2*mag[2]*(q_se[1]*q_se[3]+q_se[0]*q_se[2]);
    hy = 2*mag[0]*(q_se[1]*q_se[2]+q_se[0]*q_se[3]) + 2*mag[1]*(0.5-q_se[1]*q_se[1]-q_se[3]*q_se[3]) + 2*mag[2]*(q_se[2]*q_se[3]-q_se[0]*q_se[1]);
    hz = 2*mag[0]*(q_se[1]*q_se[3]-q_se[0]*q_se[2]) + 2*mag[1]*(q_se[2]*q_se[3]+q_se[0]*q_se[1]) + 2*mag[2]*(0.5-q_se[1]*q_se[1]-q_se[2]*q_se[2]);

    //normalize flux to only be in x and z axis -- why??
    bx = sqrt(hx*hx + hy*hy);
    bz = hz;
}