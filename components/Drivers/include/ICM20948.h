#ifndef ICM20958_H
#define ICM20958_H

#include "IO.h"
#include "driver/i2c.h"
#include "i2cdriver.h"

void ICMInit();
void getAccelerometerData(uint16_t *data[3]);
void getMagnetometerData(uint16_t *data[3]);
void getGyroData(uint16_t *data[3]);

#endif