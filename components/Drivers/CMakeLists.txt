idf_component_register(SRCS "uart_to_pi.c" "i2cdriver.c" "ICM20948.c" "motorconfig.c" "BMP390.c" "uart_to_pi.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_i2c driver esp_driver_ledc)
