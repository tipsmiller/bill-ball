#include "dc_motor.h"
#include "sys/lock.h"
#include "esp_log.h"
#include "esp_err.h"
#include "math.h"

static const char *TAG = "dc_motor";

// A flag to identify if pcnt isr service has been installed.
static bool is_pcnt_isr_service_installed = false;
// A lock to avoid pcnt isr service being installed twice in multiple threads.
static _lock_t isr_service_install_lock;
#define LOCK_ACQUIRE() _lock_acquire(&isr_service_install_lock)
#define LOCK_RELEASE() _lock_release(&isr_service_install_lock)


void initDcMotor(struct dc_motor *motorConfig) {
    ESP_LOGI(TAG, "Setting up motor");
    gpio_pad_select_gpio(motorConfig->dirPin);
    ESP_ERROR_CHECK(gpio_set_direction(motorConfig->dirPin, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(motorConfig->dirPin, 0));

    motorConfig->duty = 0.0;
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, motorConfig->outputChannel, motorConfig->dutyPin));
    mcpwm_config_t pwmConf = {
        .frequency = 20 * 1000,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, motorConfig->timer, &pwmConf));
    ESP_LOGI(TAG, "Done setting up motor");
};

// Duty can be -1 to 1. Negative values will set the direction pin HIGH
void setDuty(struct dc_motor *motorConfig, float duty) {
    duty *= motorConfig->reverse;
    //ESP_LOGI(TAG, "Setting PWM duty %f", duty);
    int direction = 0;
    if (duty < 0.0) {
        direction = 1;
    }
    ESP_ERROR_CHECK(gpio_set_level(motorConfig->dirPin, direction));
    ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0,  motorConfig->timer, motorConfig->generator, fabs(duty) * 100.0));
    ESP_ERROR_CHECK(mcpwm_set_duty_type(MCPWM_UNIT_0,  motorConfig->timer, motorConfig->generator, MCPWM_DUTY_MODE_0));
}

void disableMotor(struct dc_motor *motorConfig) {
    getCurrentVelocity(motorConfig);
    ESP_ERROR_CHECK(gpio_set_level(motorConfig->dirPin, 0));
    ESP_ERROR_CHECK(mcpwm_set_signal_low(MCPWM_UNIT_0,  motorConfig->timer, motorConfig->generator));
    pid_reset(&motorConfig->pidController);
}

static const int32_t counterLimit = 10000;

static void overflowHandler(void *arg) {
    struct dc_motor *motorConfig = (struct dc_motor *)arg;
    uint32_t status = 0;
    ESP_ERROR_CHECK(pcnt_get_event_status(motorConfig->pcntUnit, &status));

    if (status & PCNT_EVT_H_LIM) {
        motorConfig->encoderAccum += counterLimit;
    } else if (status & PCNT_EVT_L_LIM) {
        motorConfig->encoderAccum += -counterLimit;
    }
};

void initEncoder(struct dc_motor *motorConfig) {
    motorConfig->encoderAccum = 0;

    pcnt_config_t pconfA = {
        .unit = motorConfig->pcntUnit,
        .counter_h_lim = counterLimit,
        .counter_l_lim = -counterLimit,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .channel = PCNT_CHANNEL_0,
        .pulse_gpio_num = motorConfig->encAPin,
        .ctrl_gpio_num = motorConfig->encBPin,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
    };
    ESP_ERROR_CHECK(pcnt_unit_config(&pconfA));

    pcnt_config_t pconfB = {
        .unit = motorConfig->pcntUnit,
        .counter_h_lim = counterLimit,
        .counter_l_lim = -counterLimit,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .channel = PCNT_CHANNEL_1,
        .pulse_gpio_num = motorConfig->encBPin,
        .ctrl_gpio_num = motorConfig->encAPin,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
    };
    ESP_ERROR_CHECK(pcnt_unit_config(&pconfB));
    ESP_ERROR_CHECK(pcnt_counter_pause(motorConfig->pcntUnit));
    ESP_ERROR_CHECK(pcnt_counter_clear(motorConfig->pcntUnit));
    //ESP_ERROR_CHECK(pcnt_set_filter_value(PCNT_UNIT_0, 1 * 80));  // microseconds * clock cycles per microsecond
    //ESP_ERROR_CHECK(pcnt_filter_enable(PCNT_UNIT_0));

    LOCK_ACQUIRE();
    if (!is_pcnt_isr_service_installed) {
        ESP_ERROR_CHECK(pcnt_isr_service_install(0));
        // make sure pcnt isr service won't be installed more than one time
        is_pcnt_isr_service_installed = true;
    }
    LOCK_RELEASE();

    ESP_ERROR_CHECK(pcnt_isr_handler_add(motorConfig->pcntUnit, overflowHandler, motorConfig));
    ESP_ERROR_CHECK(pcnt_event_enable(motorConfig->pcntUnit, PCNT_EVT_H_LIM));
    ESP_ERROR_CHECK(pcnt_event_enable(motorConfig->pcntUnit, PCNT_EVT_L_LIM));
    ESP_ERROR_CHECK(pcnt_counter_resume(motorConfig->pcntUnit));
};

// Ticks since the beginning of time
int64_t getEncoderTotal(struct dc_motor *motorConfig) {
    int16_t counterVal = 0;
    ESP_ERROR_CHECK(pcnt_get_counter_value(motorConfig->pcntUnit, &counterVal));
    int64_t totalCounts = motorConfig->encoderAccum + counterVal;
    //ESP_LOGI(TAG, "Encoder count %lld", totalCounts);
    return totalCounts;
};

float getCurrentVelocity(struct dc_motor *motorConfig) {
    int64_t lastPosition = motorConfig->lastEncoderValue;
    int64_t newPosition = getEncoderTotal(motorConfig);
    motorConfig->lastEncoderValue = newPosition;

    int64_t newTime = esp_timer_get_time();
    int64_t timeDelta = newTime - motorConfig->lastTime;
    motorConfig->lastTime = newTime;
    float currentVelocity = (float)(newPosition - lastPosition)/timeDelta*1000000.0;
    return currentVelocity;
};

void initDcMotorController(struct dc_motor *motorConfig) {
    // velocity constants
    pid_initPIDController(&motorConfig->pidController, 0.00001, 0.00000001, 0.0, 0.0, -1.0, 1.0);
    pid_setPdiSmoothing(&motorConfig->pidController, 0.05);
    pid_reset(&motorConfig->pidController);

    //setup feed-forward controls
    motorConfig->kStatic = 0.07;
    motorConfig->kVelocity = 0.0004;
    motorConfig->kAcceleration = 0.0;
    motorConfig->lastTime = 0;
    motorConfig->lastEncoderValue = 0;
    motorConfig->lastVelocity = 0.0;
};

// call regularly to update PID and set duty
void motorVelocityControlUpdate(struct dc_motor *motorConfig, int targetVelocity) {
    float feedForwardDuty = getFeedForwardDutyFromVelocity(motorConfig, targetVelocity, 0);
    float currentVelocity = getCurrentVelocity(motorConfig);

    float pidDuty = pid_update(&motorConfig->pidController, currentVelocity, targetVelocity);
    float output = feedForwardDuty + pidDuty;
    //ESP_LOGI(TAG, "velocity, duty: %-1.2f, %-1.2f", currentVelocity, output);
    setDuty(motorConfig, output);
};

// "accel" should be in encoder counts/s
void motorAccelControlUpdate(struct dc_motor *motorConfig, int accel) {
    float targetVelocity = motorConfig->pidController.lastInput + accel;
    motorVelocityControlUpdate(motorConfig, targetVelocity);
}

float getFeedForwardDutyFromVelocity(struct dc_motor *motorConfig, float velocity, float acceleration) {
    float result = motorConfig->kVelocity*velocity + motorConfig->kAcceleration*acceleration;
    if (signbit(velocity)) {
        result -= motorConfig->kStatic;
    } else {
        result += motorConfig->kStatic;
    }
    return result;
};

float getFeedForwardDutyFromDuty(struct dc_motor *motorConfig, float duty, float acceleration) {
    float result = duty + motorConfig->kAcceleration*acceleration;
    if (signbit(duty)) {
        result -= motorConfig->kStatic;
    } else {
        result += motorConfig->kStatic;
    }
    return result;
};
