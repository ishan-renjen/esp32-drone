#include "ICM20948.h"

void ICMInit(){
    uint8_t data;
    Read(&data, ICM_WHOAMI_REG, ICM);
    if(data==ICM_WHOAMI){
        data=ICM_BANK_0;
        Write(&data, ICM_REG_BANK_SEL, ICM);
        data=ENABLE_I2C;
        Write(&data, I2C_EN_ADDR, ICM);
        data=ICM_BANK_2;
        Write(&data, ICM_REG_BANK_SEL, ICM);
        data=ICM_SET_GYRO_RANGE;
        Write(&data, ICM_GYRO_CONFIG, ICM);
        data=ICM_SET_ACCEL_RANGE;
        Write(&data, ICM_ACCEL_CONFIG, ICM);
    }
}

//return data as 2d array - accel MSB, LSB in X, Y, Z
void getAccelerometerData(uint16_t *data[3]){
    uint8_t accelData[3][2];
    uint16_t accelXYZ[3];

    uint8_t writeData=ICM_BANK_0;
    Write(&data, ICM_REG_BANK_SEL, ICM);

    uint8_t read_addr = ICM_ACCEL_START_ADDR;
    for(int x=0; x<3; x++){
        for(int y=0; y<2; y++){
            Read(&accelData[x][y], &read_addr, ICM);
            read_addr++;
        }
        accelXYZ[x] = accelData[x][0] << 8;
        accelXYZ[x] = accelData[x][1];
    }
    for(int i=0;i<3;i++){data[i] = accelXYZ[i];}
}

//return data as 2d array - magnetometer MSB, LSB in X, Y, Z
void getMagnetometerData(uint16_t *data[3]){
    uint8_t magnetData[3][2];
    uint16_t magXYZ[3];

    uint8_t read_addr = ICM_MAG_START_ADDR;
    for(int x=0; x<3; x++){
        for(int y=0; y<2; y++){
            Read(&magnetData[x][y], &read_addr, MAG);
            read_addr++;
        }
        magXYZ[x] = magnetData[x][0];
        uint16_t temp = magnetData[x][1] << 8;
        magXYZ[x] |= temp;
    }
    for(int i=0;i<3;i++){data[i] = magXYZ[i];}
}

//return data as 2d array - gyro MSB, LSB in X, Y, Z
void getGyroData(uint16_t *data[3]){
    uint8_t gyroData[3][2];
    uint16_t gyroXYZ[3];

    uint8_t writeData=ICM_BANK_0;
    Write(&writeData, ICM_REG_BANK_SEL, 0);

    uint8_t read_addr = ICM_GYRO_START_ADDR;
    for(int x=0; x<3; x++){
        for(int y=0; y<2; y++){
            Read(&gyroData[x][y], &read_addr, ICM);
            read_addr++;
        }
        gyroXYZ[x] = gyroData[x][0]<< 8;
        gyroXYZ[x] = gyroData[x][1];
    }
    for(int i=0;i<3;i++){data[i] = gyroXYZ[i];}
}