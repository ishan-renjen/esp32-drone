#include "madgwick.h"

void Q_est(float *accel[3], float *gyro[3], float *mag[3]){
    float normalized;
    //normalize the accelerometer and magnetometer vectors - divide each item by sqrt(vector)
    //accelerometer
    float accel_vector[3];
    normalized = sqrt(*accel[0]*(*accel[0]) + *accel[1]*(*accel[1]) + *accel[1]*(*accel[1]));
    for(int i=0; i<3;i++){ accel_vector[i] = *accel[i]/normalized; }

    //magnetometer
    float mag_vector[3];
    normalized = sqrt(*mag[0]*(*mag[0]) + *gyro[1]*(*gyro[1]) + *mag[2]*(*mag[2]));
    for(int i=0; i<3;i++){ mag_vector[i] = *mag[i]/normalized; }

    //compute the objective function and the jacobian
    //objective function - items 1-3 come from eqn. 25, items 4-6 come from eqn. 29
    float objective_func[6];
    objective_func[0] = 2*q_se[1]*q_se[3] - q_se[0]*q_se[2] - accel_vector[0];
    objective_func[1] = 2*q_se[0]*q_se[1] + q_se[2]*q_se[3] - accel_vector[1];
    objective_func[2] = 1.0f - 2*q_se[1]*q_se[1] - 2*q_se[2]*q_se[2] - accel_vector[2];

    objective_func[3] = bx - (q_se[2]*q_se[2] - q_se[3]*q_se[3]) + (2.0f*bz*q_se[1]*q_se[3] - 2.0f*bz*q_se[0]*q_se[2]) - mag_vector[0];
    objective_func[4] = (2.0f*bx*q_se[1]*q_se[2] - 2.0f*bx*q_se[0]*q_se[3]) + (2.0f*bz*q_se[0]*q_se[1] + 2.0f*bz*q_se[2]*q_se[3]) - mag_vector[1];
    objective_func[5] = (2.0f*bx*q_se[0]*q_se[2]+2.0f*bx*q_se[1]*q_se[3]) + (bz - 2.0f*bz*q_se[1]*q_se[1] - 2.0f*bz*q_se[2]*q_se[2]- mag_vector[2]);

    //jacobian - rows 1-3 come from eqn. 26, rows 4-6 come from eqn. 30 -TODO: transpose this
    float jacobian[6][4] = {{-2*q_se[2],                     2*q_se[3],                   -2*q_se[0],                     2*q_se[1]                     },
                            {2*q_se[1],                      2*q_se[0],                   2*q_se[3],                      2*q_se[2]                     },
                            {0,                              -4*q_se[1],                  -4*q_se[2],                     0                             },
                            {-2*bz*q_se[2],                  2*bz*q_se[3],                (-4*bx*q_se[2] - 2*bz*q_se[0]), (-4*bx*q_se[3] + 2*bz*q_se[1])},
                            {(-2*bx*q_se[3] + 2*bz*q_se[1]), (2*bx*q_se[2]+2*bz*q_se[0]), (2*bx*q_se[1]+2*bz*q_se[3]),    (-2*bx*q_se[0]+2*bz*q_se[2])  },
                            {2*bx*q_se[2],                   (2*bx*q_se[3]-4*bz*q_se[1]), (2*bx*q_se[0]-4*bz*q_se[2]),    2*bx*q_se[1]                  }}; 

    //compute gradient and normalize to get gradient unit vector - eqn. 44, carrying back to eqn. 33
    //compute gradient

    //normalize gradient
}