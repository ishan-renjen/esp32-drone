#include "BMP390.h"

void BMPInit(){
    data=BMP_PRESS_TEMP_EN_DATA;
    Write(&data, BMP_EN_REG, BMP);
}

void readPressureData(uint32_t *data){
    for(int i=0; i<3; i++){
        uint8_t tempData;
        Read(&tempData, BMP_PRESSURE_STARTING_ADDR+i, BMP);
        data = tempData << (3-(i+1))*8;
    }
}

void readTempData(uint32_t *data){
    for(int i=0; i<3; i++){
        uint8_t tempData;
        Read(&tempData, BMP_TEMP_STARTING_ADDR+i, BMP);
        data = tempData << (3-(i+1))*8;
    }
}

void populateStruct(){
    uint8_t data8;
    uint16_t data16;

    //par_t1
    Read(&data8, 49, BMP);
    coeff_data.par_t1 = data8;
    Read(&data16, 50, BMP);
    coeff_data.par_t1 |= data16 << 8;

    //par_t2
    Read(&data8, 51, BMP);
    coeff_data.par_t2 = data8;
    Read(&data16, 52, BMP);
    coeff_data.par_t2 |= data16 << 8;

    //par_t3
    Read(&data8, 53, BMP);
    coeff_data.par_t3 = data8;

    //par_p1
    Read(&data8, 54, BMP);
    coeff_data.par_p1 = data8;
    Read(&data16, 55, BMP);
    coeff_data.par_p1 |= data16 << 8;

    //par_p2
    Read(&data8, 56, BMP);
    coeff_data.par_p2 = data8;
    Read(&data16, 57, BMP);
    coeff_data.par_p2 |= data16 << 8;

    //par_p3
    Read(&data8, 58, BMP);
    coeff_data.par_p3 = data8;

    //par_p4
    Read(&data8, 59, BMP);
    coeff_data.par_p4 = data8;

    //par_p5
    Read(&data8, 60, BMP);
    coeff_data.par_p5 = data8;
    Read(&data16, 61, BMP);
    coeff_data.par_p5 |= data16 << 8;

    //par_p6
    Read(&data8, 62, BMP);
    coeff_data.par_p6 = data8;
    Read(&data16, 63, BMP);
    coeff_data.par_p6 |= data16 << 8;

    //par_p7
    Read(&data8, 64, BMP);
    coeff_data.par_p7 = data8;

    //par_p8
    Read(&data8, 65, BMP);
    coeff_data.par_p8 = data8;

    //par_p9
    Read(&data8, 66, BMP);
    coeff_data.par_p9 = data8;
    Read(&data16, 67, BMP);
    coeff_data.par_p9 |= data16 << 8;

    //par_p10
    Read(&data8, 68, BMP);
    coeff_data.par_p10 = data8;

    //par_p11
    Read(&data8, 69, BMP);
    coeff_data.par_p11 = data8;
}


//copied from datasheet
static float BMP390_compensate_temperature(uint32_t uncomp_temp, struct BMP390_calib_data *calib_data)
{
    float partial_data1;
    float partial_data2;
    partial_data1 = (float)(uncomp_temp – calib_data->par_t1);
    partial_data2 = (float)(partial_data1 * calib_data->par_t2);
    /* Update the compensated temperature in calib structure since this is
    * needed for pressure calculation */
    calib_data->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data->par_t3;
    /* Returns compensated temperature */
    return calib_data->t_lin;
}

//copied from datasheet
static float BMP390_compensate_pressure(uint32_t uncomp_press, struct BMP390_calib_data *calib_data)
{
/* Variable to store the compensated pressure */
    float comp_press;
    /* Temporary variables used for compensation */
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;
    /* Calibration data */
    partial_data1 = calib_data->par_p6 * calib_data->t_lin;
    partial_data2 = calib_data->par_p7 * (calib_data->t_lin * calib_data->t_lin);
    partial_data3 = calib_data->par_p8 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
    partial_out1 = calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;
    partial_data1 = calib_data->par_p2 * calib_data->t_lin;
    partial_data2 = calib_data->par_p3 * (calib_data->t_lin * calib_data->t_lin);
    partial_data3 = calib_data->par_p4 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
    partial_out2 = (float)uncomp_press *
    (calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);
    partial_data1 = (float)uncomp_press * (float)uncomp_press;
    partial_data2 = calib_data->par_p9 + calib_data->par_p10 * calib_data->t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data->par_p11;
    comp_press = partial_out1 + partial_out2 + partial_data4;
    return comp_press;
}

void compensate_pressure_complete(float comp_pressure){
    uint32_t temp;
    uint32_t pressure;
    readPressureData(&pressure);
    readTempData(&temp);
    populateStruct;
    BMP390_compensate_temperature(temp, &coeff_data);
    comp_pressure = BMP390_compensate_pressure(pressure, &coeff_data);
}