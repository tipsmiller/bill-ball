#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "driver/mcpwm.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "pid_controller/pid_controller.h"

struct dc_motor {
    float reverse; // set to 1.0 or -1.0
    gpio_num_t dutyPin;
    gpio_num_t dirPin;
    float duty;
    mcpwm_io_signals_t outputChannel;
    mcpwm_generator_t generator;
    mcpwm_timer_t timer;

    pcnt_unit_t pcntUnit;
    gpio_num_t encAPin;
    gpio_num_t encBPin;
    int64_t encoderAccum;

    int64_t lastEncoderValue;
    int64_t lastTime;
    float lastVelocity;

    //feed-forward control constants
    float kStatic; // fractional duty to overcome static friction
    float kVelocity; // fractional duty for encoder ticks / second
    float kAcceleration; // fractional duty for acceleration

    struct pid_controller_t pidController;
};

void initDcMotor(struct dc_motor *motorConfig);
void setDuty(struct dc_motor *motorConfig, float duty);
void disableMotor(struct dc_motor *motorConfig);

void initEncoder(struct dc_motor *motorConfig);
int64_t getEncoderTotal(struct dc_motor *motorConfig);

float getCurrentVelocity(struct dc_motor *motorConfig);
void initDcMotorController(struct dc_motor *motorConfig);
void motorVelocityControlUpdate(struct dc_motor *motorConfig, int setpoint);
void motorAccelControlUpdate(struct dc_motor *motorConfig, int accel);
float getFeedForwardDutyFromVelocity(struct dc_motor *motorConfig, float velocity, float acceleration);
float getFeedForwardDutyFromDuty(struct dc_motor *motorConfig, float duty, float acceleration);

#endif
