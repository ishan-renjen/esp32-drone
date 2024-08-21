#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_mac.h"
#include "IO.h"

void app_main(void)
{
    //initialize peripherals
    I2CInit(I2CBUS_0, ICM20948_SLAVE_ADDR, BMP390_SLAVE_ADDR, ICM_MAG_SLAVE_ADDR);
    uart_init();

    //create tasks - high priorities is read user data from uart, read sensor data from i2c - consider setting up interrupts??
    //                  -highest priority is to get altitude data to preserve height in case of sensor disconnection
    //                  -next highest is IMU data for motion and user data to compute next position and current orientation
    //             - medium priority is to run control loop - dependent on high priority tasks

    //safety loop - if user data not present beyond loop frequency, hover at current height

    //resources -- sensors - updated only with sensor tasks, and in meantime use stored data
    //          -- motors - will need mutex around this because of safety loop and control loop setting it
    //          -- user data - updated only from user data task

    //problems - what happens if sensor data refreshes too fast for controls to run? 

    //create startup sequence to read sensor data, run system w/o moving motors to flush the pipeline


}
