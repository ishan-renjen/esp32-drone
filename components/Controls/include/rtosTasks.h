#ifndef RTOSTASKS_H
#define RTOSTASKS_H

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "uart_to_pi.h"
#include "ICM20948.h"
#include "BMP390.h"
#include "PIDControl.h"


//highest priority tasks
//collect UART data
void uartTask(void *pvParameters);

//collect IMU data
void imuTask(void *pvParameters);

//collect BMP data
void bmpTask(void *pvParameters);

//run PID altitude loop
void pidAltitudeTask(void *pvParameters);

//medium priority tasks
//determine motor thrusts
void motorTask(void *pvParameters);

//lower priority tasks
//run complementary filter
void compfilterTask(void *pvParameters);

//run inner loop velocity control
void velocityPIDLoopTask(void *PvParameters);

//lowest priority tasks
void positionPIDControlTask(void *pvParameters);

#endif //RTOSTASKS_H