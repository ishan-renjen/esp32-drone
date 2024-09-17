#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/message_buffer.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "madgwick.h"
#include "compfilter.h"

#include "uart_to_pi.h"
#include "ICM20948.h"
#include "BMP390.h"
#include "PIDControl.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_mac.h"
#include "IO.h"

QueueHandle_t bmpQueue;
QueueHandle_t icmQueue;
QueueHandle_t uartQueue;
QueueHandle_t outerToInner;
QueueHandle_t throttleQueue;

QueueSetHandle_t altitudeLoopSet; //take in uart, bmp, 
QueueSetHandle_t outerLoopSet; //take in uart and icm
QueueSetHandle_t innerLoopSet; //take in outer loop and icm

#define MAX_STACK_SIZE 100

#define UART_PRIORITY 4
#define ICM_PRIORITY 3
#define BMP_PRIORITY 1

#define OUTER_lOOP_PRIORITY 4
#define INNER_LOOP_PRIORITY 3
#define ALTITUDE_LOOP_PRIORITY 2

#define MOTOR_CONTROL_PRIORITY 2

typedef struct IMUData {
    float accelerometer[3];
    float gyro[3];
    float magnetometer[3];
};

typedef struct UserData {
    float quat_ideal[4];
    float height;
};

typedef struct OuterLoopResult{
    float data[3];
};

typedef struct MotorThrottleData{
    float throttle;
    float motorThrust[4];
}

void app_main(void)
{
    //initialize peripherals
    I2CInit(I2CBUS_0, ICM20948_SLAVE_ADDR, BMP390_SLAVE_ADDR, ICM_MAG_SLAVE_ADDR);
    uart_init();

    bmpQueue = xQueueCreate(10, sizeof(float));
    icmQueue = xQueueCreate(10, sizeof(struct IMUData *));
    uartQueue = xQueueCreate(2, sizeof(struct UserData *));
    outerToInner = xQueueCreate(2, sizeof(struct OuterLoopResult *));
    throttleQueue = xQueueCreate(4, sizeof(struct MotorThrottleData *));

    altitudeLoopSet = xQueueCreateSet(13);
    xQueueAddToSet(bmpQueue, altitudeLoopSet);
    xQueueAddToSet(uartQueue, altitudeLoopSet);

    outerLoopSet = xQueueCreateSet(13);
    xQueueAddToSet(icmQueue, outerLoopSet);
    xQueueAddToSet(uartQueue, outerLoopSet);

    innerLoopSet = xQueueCreateSet(13);
    xQueueAddToSet(icmQueue, innerLoopSet);
    xQueueAddToSet(outerToInner, innerLoopSet);

    TaskHandle_t uartTaskHandle = NULL;
    TaskHandle_t icmTaskHandle = NULL;
    TaskHandle_t bmpTaskHandle = NULL;
    TaskHandle_t pidAltitudeTaskHandle = NULL;
    TaskHandle_t motorTaskHandle = NULL;
    TaskHandle_t velocityPIDLoopTaskHandle = NULL;
    TaskHandle_t positionPIDControlTaskHandle = NULL;

    xTaskCreate(uartTask, "uart data task", MAX_STACK_SIZE, (void*)uartQueue, UART_PRIORITY, uartTaskHandle);
    xTaskCreate(icmTask, "ICM data task", MAX_STACK_SIZE, (void*)bmpQueue, ICM_PRIORITY, icmTaskHandle);
    xTaskCreate(bmpTask, "BMP data task", MAX_STACK_SIZE, (void*)uartQueue, BMP_PRIORITY, bmpTaskHandle);
    xTaskCreate(pidAltitudeTask, "pid altitude loop task", MAX_STACK_SIZE, (void*)uartQueue, ALTITUDE_LOOP_PRIORITY, pidAltitudeTaskHandle);
    xTaskCreate(motorTask, "motor controls task", MAX_STACK_SIZE, (void*)uartQueue, MOTOR_CONTROL_PRIORITY, motorTaskHandle);
    xTaskCreate(velocityPIDLoopTask, "velocity pid loop task", MAX_STACK_SIZE, (void*)uartQueue, INNER_LOOP_PRIORITY, velocityPIDLoopTaskHandle);
    xTaskCreate(positionPIDControlTask, "position pid loop task", MAX_STACK_SIZE, (void*)uartQueue, OUTER_lOOP_PRIORITY, positionPIDControlTaskHandle);

    vTaskStartScheduler();
}

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
    char *buffer;
    QueueSetHandle_t xQueue;
    struct UserData userData;
    float pressureData;
    struct motorThrottleData motorData;
    for(;;){
        xQueue = xQueueSelectFromSet(altitudeLoopSet, 1);

        if(xQueue == NULL){}
        else if(xQueue == (QueueSetMemberHandle_t) uartQueue){
            xQueueReceive(xQueue, buffer, 1);
            userData = (struct UserData *) buffer;
        }
        else if(xQueue == (QueueSetMemberHandle_t) bmpQueue){
            xQueueRecieve(xQueue, buffer, 1);
            pressureData = (float *) buffer;
        }

        heightLoop(&pressureData, &userData.height, &motorData.throttle);
        xQueueSend(throttleQueue, motorData, 1);
    }
}

void velocityPIDLoopTask(void *PvParameters){
    char *buffer;
    QueueSetHandle_t xQueue;
    struct ICMData icmData;
    struct OuterLoopResult outerLoopData;
    struct MotorThrottleData motorData;
    for(;;){
        xQueue = xQueueSelectFromSet(innerLoopSet, 1);

        if(xQueue == NULL){}
        else if(xQueue == (QueueSetMemberHandle_t) outerToInner){
            xQueueReceive(uartQueue, buffer, 0);
            icmData = (struct ICMData *) buffer;
        }
        else if(xQueue == (QueueSetMemberHandle_t) icmQueue){
            xQueueReceive(uartQueue, buffer, 0);
            outerLoopData = (struct OuterLoopResult *) buffer;
        }

        float actual_vel[3];
        complementary_filter(icmData.accelerometer, icmData.gyro, icmData.magnetometer, actual_vel);

        float thrust[4];
        innerLoop(actual_vel, outerLoopData.data, thrust);

        motorData.motorThrust = thrust;
        xQueueSend(throttleQueue, motorData, 1);
    }
}

void motorTask(void *pvParameters){
    char *buffer;
    struct MotorThrottleData motorData;
    for(;;){
        xQueueReceive(throttleQueue, buffer, 1);
        motorData = (struct MotorThrottleData *) buffer;
        motorControl(motorData.motorThrust, &motorData.throttle);
    }
}

void positionPIDControlTask(void *pvParameters){
    char *buffer;
    QueueSetHandle_t xQueue;
    struct IMUdata imuData;
    struct UserData userData;
    struct OuterLoopData data;
    for(;;){
        xQueue = xQueueSelectFromSet(outerLoopSet, 1);

        if(xQueue == NULL){}
        else if(xQueue == (QueueSetMemberHandle_t) uartQueue){
            xQueueReceive(uartQueue, buffer, 0);
            userData = (struct UserData *) buffer;
        }
        else if(xQueue == (QueueSetMemberHandle_t) icmQueue){
            xQueueReceive(icmQueue, buffer, 0);
            imuData = (struct IMUData *) buffer;
        }

        float quat_actual[4];
        float vel_ideal[3];
        Q_est(imuData.accelerometer, imuData.gyro, imuData.magnetometer, quat_actual);  
        outerLoop(userData.quat_ideal, quat_actual, vel_ideal);
        data.data = vel_ideal;
        xQueueSend(outerToInner, data, 1);
    }
}