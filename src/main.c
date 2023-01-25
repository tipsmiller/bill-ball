#include "config.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heartbeat/heartbeat.h"
#include "esp_log.h"
#include "esp_err.h"
#include "control_loop/control_loop.h"

static const char *TAG = "main";

void app_main() { 
    // Setup
    ESP_LOGI(TAG, "Program beginning");

    // start the heartbeat
    beginHeartbeat();

    TaskHandle_t controlTaskHandle;
    xTaskCreate(controlTask, "control loop", 10000, (void*)NULL, configMAX_PRIORITIES-1, &controlTaskHandle);

    // Cleanup
}