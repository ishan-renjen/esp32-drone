#ifndef I2CDRIVER_H
#define I2CDRIVER_H

#include "IO.h"
#include "driver/i2c_master.h"

extern enum HandleType{
    ICM, BMP, MAG
};

extern void I2CInit(int i2c_num, int icm_slave_addr, int bmp_slave_addr, int mag_slave_addr);
extern void Read(uint8_t *data, uint8_t *devaddr, int handle);
extern void Write(uint8_t *data, uint8_t *devaddr, int handle);

#endif //I2CDRIVER_H