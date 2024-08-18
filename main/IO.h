//header file containing global variables

#define MOTOR_FREQ 50
#define PWM_TIMER_RES 13

#define MOTOR_1 4 //GPIO4
#define MOTOR_2 5 //GPIO5
#define MOTOR_3 6 //GPIO6
#define MOTOR_4 7 //GPIO7

#define I2C_BUS_SCL_IO 42 //GPIO37
#define I2C_BUS_SDA_IO 43 //GPIO38

#define I2C_MASTER_FREQ_HZ 400000

#define ICM20948_SLAVE_ADDR 0x68
#define ICM_MAG_SLAVE_ADDR 0x0C
#define BMP390_SLAVE_ADDR 0x70

#define I2CBUS_0 0
#define I2CBUS_1 1

#define ICM_WHOAMI_REG 0x00
#define ICM_WHOAMI 0x3A
#define ICM_REG_BANK_SEL 0x7f //bank 0
#define ENABLE_I2C 0x20
#define I2C_EN_ADDR 0x03 //bank 0

#define ICM_BANK_0 0x00
#define ICM_BANK_1 0x10
#define ICM_BANK_2 0x20
#define ICM_BANK_3 0x30

#define ICM_GYRO_CONFIG 0x01 //bank 2
#define ICM_SET_GYRO_RANGE 0x06

#define ICM_ACCEL_CONFIG 0x14
#define ICM_SET_ACCEL_RANGE 0x06

#define ICM_ACCEL_START_ADDR 0x2D
#define ICM_GYRO_START_ADDR 0x33
#define ICM_MAG_START_ADDR 0x11

#define BMP_STATUS_REG 0x03
#define BMP_DATA_READY_AND_FILTER 0b00100000;
#define BMP_COMMAND_STATUS_READY_AND_FILTER 0b00010000;

#define BMP_PRESSURE_STARTING_ADDR 0x04
#define BMP_TEMP_STARTING_ADDR 0x07

#define BMP_EN_REG 0x1B
#define BMP_PRESS_TEMP_EN_DATA 0x03