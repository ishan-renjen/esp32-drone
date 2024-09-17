#include "rtosTasks.h"

void uartTask(void *pvParameters){
    for(;;){
        float data;
        uart_read_float(&data);

        QueueHandle_t uartQueue = (QueueHandle_t *) pvParameters;
        xQueueSend(uartQueue, data, 1);
    }
}

void icmTask(void *pvParameters){
    struct IMUdata imuData;
    for(;;){
        getAccelerometerData(&imuData.accelerometer);
        getGyroData(&imuData.gyro);
        getMagnetometerData(&imuData.magnetometer);

        QueueHandle_t icmDataQueue = (QueueHandle_t *) pvParameters;
        xQueueSend(icmDataQueue, imuData, 1);
    }
}

void bmpTask(void *pvParameters){
    for(;;){
        float pressure;
        compensate_pressure_complete(&pressure);

        QueueHandle_t bmpDataQueue = (QueueHandle_t *) pvParameters;
        xQueueSend(bmpDataQueue, pressure, 1);
    }
}

void pidAltitudeTask(void *pvParameters){
    
}

void motorTask(void *pvParameters){}

void compfilterTask(void *pvParameters){}

void velocityPIDLoopTask(void *PvParameters){}

void positionPIDControlTask(void *pvParameters){}