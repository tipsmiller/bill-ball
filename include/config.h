#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

#define BLINK_PIN GPIO_NUM_2
#define ENABLE_PIN GPIO_NUM_18

#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_SDA_IO GPIO_NUM_23


//motor 1
#define M1_ENC_A GPIO_NUM_13
#define M1_ENC_B GPIO_NUM_14

#define M1_PWM_DIRECTION GPIO_NUM_27
#define M1_PWM_DUTY GPIO_NUM_26

//motor 2
#define M2_ENC_A GPIO_NUM_35
#define M2_ENC_B GPIO_NUM_34

#define M2_PWM_DIRECTION GPIO_NUM_33
#define M2_PWM_DUTY GPIO_NUM_32

//motor 3
#define M3_ENC_A GPIO_NUM_39
#define M3_ENC_B GPIO_NUM_25

#define M3_PWM_DIRECTION GPIO_NUM_19
#define M3_PWM_DUTY GPIO_NUM_21

#endif