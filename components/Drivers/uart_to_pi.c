#include "uart_to_pi.h"

void uart_init(){
    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 460800,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_TX, UART_RX, UART_RTS, UART_CTS));

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
}

void uart_write_float(float *data){
    uint8_t bytes[4];
    convert_to_array(data, bytes);
    for(int i=0;i<4;i++){
        const uart_port_t uart_num = UART_NUM_2;
        // Write data to UART.
        uart_write_bytes(uart_num, (const float*)bytes[i], 4);
        // Wait for packet to be sent
        ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 100)); // wait timeout is 100 RTOS ticks (TickType_t)
    }
}

void uart_read_float(float *data_float){
    // Read data from UART.
    const uart_port_t uart_num = UART_NUM_2;
    uint8_t data[4];
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
    length = uart_read_bytes(uart_num, data, length, 100);
    convert_to_float(data, data_float);
}

void convert_to_array(float *data, uint8_t bytes[4]){
    for(int i=0;i<4;i++){
        float data_temp = *data;
        bytes[i] = data_temp;
        data_temp >> 8;
    }
}

void convert_to_float(uint8_t bytes[4], float *data){
    float data_temp;
    for(int i=0;i<4;i++){
        data_temp = bytes[i];
        data_temp << 8;
    }
    *data = data_temp;
}