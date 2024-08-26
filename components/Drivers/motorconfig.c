#include "motorconfig.h"
#include "IO.h"

ledc_timer_config_t timer1 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = MOTOR_FREQ,
    .duty_resolution = PWM_TIMER_RES
};

ledc_timer_config_t timer2 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_1,
    .freq_hz = MOTOR_FREQ,
    .duty_resolution = PWM_TIMER_RES
};

ledc_timer_config_t timer3 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_2,
    .freq_hz = MOTOR_FREQ,
    .duty_resolution = PWM_TIMER_RES
};

ledc_timer_config_t timer4 = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_3,
    .freq_hz = MOTOR_FREQ,
    .duty_resolution = PWM_TIMER_RES
};

ledc_channel_config_t channel0 = {
    .gpio_num = MOTOR_1,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0
};

ledc_channel_config_t channel0 = {
    .gpio_num = MOTOR_1,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0
};

ledc_channel_config_t channel1 = {
    .gpio_num = MOTOR_2,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .timer_sel = LEDC_TIMER_1,
    .duty = 0
};

ledc_channel_config_t channel2 = {
    .gpio_num = MOTOR_3,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .timer_sel = LEDC_TIMER_2,
    .duty = 0
};

ledc_channel_config_t channel3 = {
    .gpio_num = MOTOR_4,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_3,
    .timer_sel = LEDC_TIMER_3,
    .duty = 0
};

//sets up PWM timers and channels via GPIO pins
void setup(){
    ledc_timer_config(&timer1);
    ledc_timer_config(&timer2);
    ledc_timer_config(&timer3);
    ledc_timer_config(&timer4);

    ledc_channel_config(&channel0);
    ledc_channel_config(&channel1);
    ledc_channel_config(&channel2);
    ledc_channel_config(&channel3);
}

bool setMotorSpeed(int motor, int duty){  
    switch(motor){
        case 1:
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            return true;
            break;
        case 2:
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            return true;
            break;
        case 3:
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
            return true;
            break;
        case 4:
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
            return true;
            break;
        default:
            return false;
            break;
    }
}

double getMotorSpeed(int motor){
    switch(motor){
        case 1:
            uint32_t duty = ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            return duty;
            break;
        case 2:
            uint32_t duty = ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            return duty;
            break;
        case 3:
            uint32_t duty = ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            return duty;
            break;
        case 4:
            uint32_t duty = ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            return duty;
            break;
        default:
            return LEDC_ERR_DUTY;
            break;
    }
}