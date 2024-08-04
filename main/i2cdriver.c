#include "i2cdriver.h"

extern void I2CInit(int i2c_num, int icm_slave_addr, int bmp_slave_addr, int mag_slave_addr){
    i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = i2c_num,
    .scl_io_num = I2C_BUS_SCL_IO,
    .sda_io_num = I2C_BUS_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
    };
 
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t icm_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = icm_slave_addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &icm_dev_cfg, &icm_dev_handle));

    i2c_device_config_t bmp_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = bmp_slave_addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bmp_dev_cfg, &bmp_dev_handle));

    i2c_device_config_t bmp_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = mag_slave_addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bmp_dev_cfg, &mag_dev_handle));
}

extern void Read(uint8_t *data, uint8_t *devaddr, int handle){
    uint8_t recvData[2];
    recvData[0] = devaddr;
    recvData[1] = data;
    switch(handle){
        case ICM:
            ESP_ERROR_CHECK(i2c_master_receive(mag_dev_handle, recvData, 1, 0));
            break;
        case BMP:
            ESP_ERROR_CHECK(i2c_master_receive(mag_dev_handle, recvData, 1, 0));
            break;
        case MAG:
            ESP_ERROR_CHECK(i2c_master_receive(mag_dev_handle, recvData, 1, 0));
            break;
    }
    data = recvData[1];
}

extern void Write(uint8_t *data, uint8_t *devaddr, int handle){
    uint8_t recvData[2];
    recvData[0] = devaddr;
    recvData[1] = data;
    switch(handle){
        case ICM:
            ESP_ERROR_CHECK(i2c_master_receive(mag_dev_handle, recvData, 1, 0));
            break;
        case BMP:
            ESP_ERROR_CHECK(i2c_master_receive(mag_dev_handle, recvData, 1, 0));
            break;
        case MAG:
            ESP_ERROR_CHECK(i2c_master_receive(mag_dev_handle, recvData, 1, 0));
            break;
    }
    data = recvData[1];
}