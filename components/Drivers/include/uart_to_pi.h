#ifndef UART_TO_PI_H
#define UART_TO_PI_H

#include <stdio.h>
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "IO.h"

extern void uart_init();
extern void uart_write_float(float *data);
extern void uart_read_float(float *data_float);

#endif //UART_TO_PI_H