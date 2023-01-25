#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "config.h"
#include "heartbeat.h"
#include "esp_log.h"

static const char *TAG = "heartbeat";

int heartbeatState;
TimerHandle_t tmr_long;
TimerHandle_t tmr_short;
static const int timerId = 1;
static const int intervalLong = 950;
static const int intervalShort = 50;

void heartbeat(TimerHandle_t xTimer) {
    if (heartbeatState == 1) {
        gpio_set_level(BLINK_PIN, 0);
        heartbeatState = 0;
        if (xTimerStart(tmr_long, 10) != pdPASS) {
            ESP_LOGI(TAG, "error starting heartbeat");
        }
    } else {
        gpio_set_level(BLINK_PIN, 1);
        heartbeatState = 1;
        if (xTimerStart(tmr_short, 10) != pdPASS) {
            ESP_LOGI(TAG, "error starting heartbeat");
        }
    }
}

// heartbeat light intialization
void beginHeartbeat() {
    gpio_pad_select_gpio(BLINK_PIN);
    gpio_set_direction(BLINK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_PIN, 1);

    tmr_long = xTimerCreate("heartbeat", pdMS_TO_TICKS(intervalLong), pdFALSE, (void*)timerId, heartbeat);
    tmr_short = xTimerCreate("heartbeat", pdMS_TO_TICKS(intervalShort), pdFALSE, (void*)timerId, heartbeat);
    heartbeat(tmr_long);
}