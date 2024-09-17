#ifndef BMP390_H
#define BMP390_H

#include "stdint.h"
#include "i2cdriver.h"
#include "IO.h"

void BMPInit();
void readPressureData(uint32_t *data);
void readTempData(uint32_t *data);
extern void compensate_pressure_complete(float *comp_pressure);

typedef struct {
    uint16_t par_t1;
    uint16_t par_t2;
    uint8_t par_t3;
    uint16_t par_p1;
    uint16_t par_p2;
    uint8_t par_p3;
    uint8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    uint8_t par_p7;
    uint8_t par_p8;
    uint16_t par_p9;
    uint8_t par_p10;
    uint8_t par_p11;
    float t_lin;
}BMP390_calib_data;

#endif //BMP390_H