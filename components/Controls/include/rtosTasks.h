#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


//highest priority tasks
//collect UART data
void uartTask(void *pvParameters);
void createUARTtASK(void);

//collect I2C data
void i2cTask(void *pvParameters);
void createI2CTask(void);

//run PID altitude loop
void pidAltitudeTask(void *pvParameters);
void createPidAltitudeTask(void);

//check if I2C and UART data are disrupted - if so suspend all other tasks, set altitude to 1 meter and hover
void safetyTask(void *pvParameters);
void createSafetyTask();

//medium priority tasks
//determine motor thrusts
void motorTask(void *pvParameters);
void createMotorTask(void);

//lower priority tasks
//run complementary filter
void compfilterTask(void *pvParameters);
void createCompFilterTask(void);

//run inner loop velocity control
void velocityPIDLoopTask(void *PvParameters);
void createVelocityPidLoopTask(void);

//lowest priority tasks
void positionPIDControlTask(void *pvParameters);
void create positionPIDControlTask(void *pvParameters);