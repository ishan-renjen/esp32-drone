#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_mac.h"
#include "IO.h"

void app_main(void)
{
    //initialize peripherals
    I2CInit(I2CBUS_0, ICM20948_SLAVE_ADDR, BMP390_SLAVE_ADDR, ICM_MAG_SLAVE_ADDR);
    uart_init();

    /*RTOS structure
    highest priorities:
        Task - collect UART data
        Task - collect I2C data

    next level:
        Task - compute motor thrusts
        Task - run altitude PID loop - inner

    Safety measure: if I2C read or UART read fails at the specified tick, stop motion - 
        set user quaternion to 0, store current altitude, and hover in place

    next level:
        Task - run complementary filter
        Task - run velocity PID loop - inner

    next level:
        Task - run madgwick filter
        Task - run position PID loop - outer


    Create FIFOs storing I2C data, UART data to be used

    I2C data runs at ~400 khz - store in queue
    UART data runs at 900 Hz - store in queue

    if either data is not ESP_OK stop all motion, freeze altitude

    altitude PID loop runs at 10 KHz - update pid loop
        put motor access in mutex b/c velocity loop and altitude loop will be accessing it
            put buffer inputting data into motor thrust computation to hold most recent value

    position PID loop runs at 5 KHz - inputs data from complementary filter and previous outer PID loop result

    position PID loop runs at 500 Hz - inputs data from madgwick filter and UART FIFO

    Madgwick and complementary filters do not have hard timing requirements but need to be recomputed before dependent PID loops run
    */

}
