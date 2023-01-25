#include "freertos/queue.h"

void setControlQueue(QueueHandle_t q);
void controlTask(void* params);