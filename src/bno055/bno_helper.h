#ifndef BNO_HELPER_H
#define BNO_HELPER_H

#include "bno055.h"
#include "driver/i2c.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "config.h"
#include <math.h>


struct bno055_quaternion_double_t
{
    double w;
    double x;
    double y;
    double z;
};

void setupBNO055();
void calibrateBNO();
void writeBNOConfig();
struct bno055_gravity_double_t getGravity();
struct bno055_euler_double_t getYPR();
struct bno055_quaternion_double_t getQuaternionOrientation(struct bno055_quaternion_double_t *p);
struct bno055_quaternion_double_t quaternionNormalize(struct bno055_quaternion_double_t *p);

#endif