#ifndef I2CDRIVER_H
#define I2CDRIVER_H

#include "IO.h"
#include "driver/i2c_master.h"

extern void I2CInit(int i2c_num, int icm_slave_addr, int bmp_slave_addr, int mag_slave_addr);
extern void Read(uint8_t *data, uint8_t *devaddr, int handle);
extern void Write(uint8_t *data, uint8_t *devaddr, int handle);

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t icm_dev_handle;
i2c_master_dev_handle_t bmp_dev_handle;
i2c_master_dev_handle_t mag_dev_handle;

#endif //I2CDRIVER_H