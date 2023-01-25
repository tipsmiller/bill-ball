#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "esp_timer.h"

struct pid_controller_t {
    float kp;
    float ki;
    float kde;
    float kdi;
    int minimumCycleMs;
    int64_t lastTime; // microseconds
    float lastError;
    float lastInput;
    float lastDInput;
    float lastOutput;
    float outMin;
    float outMax;
    float errorIntegral;
    float pdiSmoothing; // set > 0 to enable pdi smoothing, smaller is more smoothing
};

void pid_initPIDController(struct pid_controller_t *controller, float kp, float ki, float kde, float kdi, float outMin, float outMax);
float pid_update(struct pid_controller_t *controller, float input, float setpoint);
void pid_setCoefficients(struct pid_controller_t *controller, float kp, float ki, float kde, float kdi);
void pid_setLimits(struct pid_controller_t *controller, float outMin, float outMax);
void pid_setMinimumCycleTime(struct pid_controller_t *controller, int minimumCycleMs);
void pid_reset(struct pid_controller_t *controller);
void pid_setPdiSmoothing(struct pid_controller_t *controller, float smoothing);

#endif