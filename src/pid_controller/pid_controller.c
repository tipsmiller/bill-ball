#include "pid_controller.h"

void pid_initPIDController(struct pid_controller_t *controller, float kp, float ki, float kde, float kdi, float outMin, float outMax) {
    pid_setMinimumCycleTime(controller, 1);
    pid_setPdiSmoothing(controller, 0.0);
    pid_setCoefficients(controller, kp, ki, kde, kdi);
    pid_setLimits(controller, outMin, outMax);
    pid_reset(controller);
}

void pid_setCoefficients(struct pid_controller_t *controller, float kp, float ki, float kde, float kdi) {
    controller->kp = kp;
    controller->ki = ki;
    controller->kde = kde;
    controller->kdi = kdi;
}

void pid_setLimits(struct pid_controller_t *controller, float outMin, float outMax) {
    controller->outMin = outMin;
    controller->outMax = outMax;
}

void pid_setMinimumCycleTime(struct pid_controller_t *controller, int minimumCycleMs) {
    controller->minimumCycleMs = minimumCycleMs;
}

float pid_update(struct pid_controller_t *controller, float input, float setpoint) {
    // get the time
    int64_t newTime = esp_timer_get_time();
    int64_t timeDelta = newTime - controller->lastTime;
    float output = 0.0;
    // see if it's been long enough to produce a new value
    if(timeDelta / 1000 > controller->minimumCycleMs) {
        // Run the calc
        float error = setpoint - input;
        // proportional
        output = controller->kp * error;
        // if this is NOT the first run, calculate the derivative and integral
        if(controller->lastTime != 0) {
            // integral
            // Uses a right-side rectangle integration rule, no interpolation
            controller->errorIntegral += controller->ki * timeDelta * error;
            // clamp the integral to prevent wind-up and crazy overshoots
            if(controller->errorIntegral > controller->outMax) {
                controller->errorIntegral = controller->outMax;
            }
            if(controller->errorIntegral < controller->outMin) {
                controller->errorIntegral = controller->outMin;
            }
            output += controller->errorIntegral;

            // error derivative
            output += controller->kde * ((error - controller->lastError) / timeDelta);
            // input (process) derivative
            // weight this by pdiSMoothing to reduce noise
            float dInput = input - controller->lastInput;
            if (controller->pdiSmoothing > 0.0) {
                dInput = (controller->lastDInput + dInput * controller->pdiSmoothing) / (1 + controller->pdiSmoothing);
            }
            controller->lastDInput = dInput;
            output -= controller->kdi * (dInput / timeDelta);
        }
        // clamp the output
        if(output > controller->outMax) {
            output = controller->outMax;
        }
        if(output < controller->outMin) {
            output = controller->outMin;
        }
        // save the results
        controller->lastError = error;
        controller->lastInput = input;
        controller->lastOutput = output;
        controller->lastTime = newTime;
    } else {
        return controller->lastOutput;
    }
    return output;
}

void pid_reset(struct pid_controller_t *controller) {
    controller->lastInput = 0;
    controller->lastOutput = 0;
    controller->lastTime = 0;
    controller->errorIntegral = 0;
    controller->lastDInput = 0;
}

void pid_setPdiSmoothing(struct pid_controller_t *controller, float smoothing) {
    controller->pdiSmoothing = smoothing;
}
