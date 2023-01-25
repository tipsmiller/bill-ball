#include "config.h" 
#include "esp_log.h"
#include "esp_err.h"
#include "math.h"
#include "input_filters/median_filter.h"
#include "bno055/bno_helper.h"
#include "dc_motor/dc_motor.h"
#include "pid_controller/pid_controller.h"

static const char *TAG = "control";

/*
Y
|
|       M1
|       |
|     /   \
|    M2    M3
|_______________X
*/
struct dc_motor motor1Conf = {
    .dirPin = M1_PWM_DIRECTION,
    .dutyPin = M1_PWM_DUTY,
    .reverse = 1.0,
    .outputChannel = MCPWM0A,
    .generator = MCPWM_GEN_A,
    .timer = MCPWM_TIMER_0,
    .encAPin = M1_ENC_A,
    .encBPin = M1_ENC_B,
    .pcntUnit = PCNT_UNIT_0,
};
struct dc_motor motor2Conf = {
    .dirPin = M2_PWM_DIRECTION,
    .dutyPin = M2_PWM_DUTY,
    .reverse = 1.0,
    .outputChannel = MCPWM0B,
    .generator = MCPWM_GEN_B,
    .timer = MCPWM_TIMER_0,
    .encAPin = M2_ENC_A,
    .encBPin = M2_ENC_B,
    .pcntUnit = PCNT_UNIT_1,
};
struct dc_motor motor3Conf = {
    .dirPin = M3_PWM_DIRECTION,
    .dutyPin = M3_PWM_DUTY,
    .reverse = 1.0,
    .outputChannel = MCPWM1A,
    .generator = MCPWM_GEN_A,
    .timer = MCPWM_TIMER_1,
    .encAPin = M3_ENC_A,
    .encBPin = M3_ENC_B,
    .pcntUnit = PCNT_UNIT_2,
};

struct pid_controller_t xGravityPID = {};
struct pid_controller_t yGravityPID = {};
struct pid_controller_t xPositionPid = {};
struct pid_controller_t yPositionPid = {};
struct pid_controller_t headingPID = {};
float outDuty;
float positionAdjustmentToSetpoint = 0.0;
struct median_filter_t mFilter = {};
int enabled = 0;
int initialPositionX = 0;
int initialPositionY = 0;
double initialPositionHeading = 0.0;
struct bno055_gravity_double_t dGravityXYZ = {};
struct bno055_euler_double_t dEulerYPR = {};
double sin60 = 0.86602540378;

void getPositionFromEncoders(int *positionX, int *positionY) {
    int tach1 = getEncoderTotal(&motor1Conf);
    int tach2 = getEncoderTotal(&motor2Conf);
    int tach3 = getEncoderTotal(&motor3Conf);
    *positionX = (int)(tach1 - 0.5*tach2 - 0.5*tach3);
    *positionY = (int)(-sin60*tach2 + sin60*tach3);
    //ESP_LOGI(TAG, "Distance from (absolute) 0: %d %d", *positionX, *positionY);
}

void getPositionAdjustment() {
    // use the position when enabled as setpoint
    int positionX = 0;
    int positionY = 0;
    getPositionFromEncoders(&positionX, &positionY);
    pid_update(&xPositionPid, positionX, initialPositionX);
    pid_update(&yPositionPid, positionY, initialPositionY);
    //ESP_LOGI(TAG, "Adjustment for position: X %-1.2f\t Y %-1.2f", xPositionPid.lastOutput, yPositionPid.lastOutput);
}

double getHeadingAdjustment(double heading, double target) {
    // find the angle between heading and target in the shortest direction
    // heading is 0-2*PI
    // shortest direction will always be |delta| <= PI
    // if target = .25*PI and heading = .75*PI, result should be -.5*PI (which is target - heading)
    // if target = .25*PI and heading = 1.75*PI, result should be +.5*PI (which is target - (heading - 2*PI))
    // if target = 1.75*PI and heading = .25*PI, result should be -.5*PI (which is target - (heading + 2*PI))
    double delta = target - heading;
    if (delta >= M_PI) {
        delta = target - (heading + 2*M_PI);
    } else if (delta <= -M_PI) {
        delta = target - (heading - 2*M_PI);
    }
    return delta;
}

void updateMotors(float vx, float vy, float vccw) {
    // M1 is aligned with X axis. D1 = Vx + 0*Vy
    // M2 is 30 deg below X axis. D2 = -sin(30)*Vx - sin(60)*Vy
    // M2 is (also) 30 deg below X axis. D2 = -sin(30)*Vx + sin(60)*Vy
    float D1 = vx + 0.0*vy + vccw;
    float D2 = -0.5*vx - sin60*vy + vccw;
    float D3 = -0.5*vx + sin60*vy + vccw;
    setDuty(&motor1Conf, getFeedForwardDutyFromDuty(&motor1Conf, D1, 0));
    setDuty(&motor2Conf, getFeedForwardDutyFromDuty(&motor2Conf, D2, 0));
    setDuty(&motor3Conf, getFeedForwardDutyFromDuty(&motor3Conf, D3, 0));
}

static void crashDetect() {
    //if (fabs(dGravityXYZ.z) < 9.3) {
    if (fabs(dEulerYPR.p) > .25*M_PI || fabs(dEulerYPR.r) > .25*M_PI) {
        disableMotor(&motor1Conf);
        disableMotor(&motor2Conf);
        disableMotor(&motor3Conf);
        // for safety, wait for the device to be disabled before starting anything
        ESP_LOGI(TAG, "crash detected");
        ESP_LOGI(TAG, "Waiting for disable to clear safety");
        while(gpio_get_level(ENABLE_PIN) != 1) {
            vTaskDelay(20/portTICK_PERIOD_MS);
        };
    }
}

void controlLoop() {
        // Main code goes here
        crashDetect();
        // read orientation
        //dGravityXYZ = getGravity();
        //ESP_LOGI(TAG, "Gravity: X (%f) Y (%f) Z (%f)", dGravityXYZ.x, dGravityXYZ.y, dGravityXYZ.z);
        dEulerYPR = getYPR();
        //ESP_LOGI(TAG, "Pitch: %-1.2f\t Roll: %-1.2f\t Heading: %-1.2f", dEulerYPR.p, dEulerYPR.r, dEulerYPR.h);
        // check if enabled
        enabled = gpio_get_level(ENABLE_PIN);
        if (enabled == 0) {
            // put the position vectors into their PID loops to get gravity adjustments
            getPositionAdjustment();
            // put the gravity vectors into their PID loops to get balancing demands (setpoints = output from position PID)
            float vx = pid_update(&xGravityPID, sin(dEulerYPR.p), sin( 0.06 + xPositionPid.lastOutput));
            float vy = pid_update(&yGravityPID, sin(dEulerYPR.r), sin(-0.04 + yPositionPid.lastOutput));
            float vccw = pid_update(&headingPID, getHeadingAdjustment(dEulerYPR.h, initialPositionHeading), 0.0);
            //ESP_LOGI(TAG, "gravity x, gravity y: %-1.2f, %-1.2f", dGravityXYZ.y, -dGravityXYZ.x);
            // update the motors
            updateMotors(vx, vy, vccw);
        } else {
            pid_reset(&xGravityPID);
            pid_reset(&yGravityPID);
            pid_reset(&xPositionPid);
            pid_reset(&yPositionPid);
            pid_reset(&headingPID);
            getPositionFromEncoders(&initialPositionX, &initialPositionY);
            initialPositionHeading = dEulerYPR.h;
            disableMotor(&motor1Conf);
            disableMotor(&motor2Conf);
            disableMotor(&motor3Conf);
            ESP_LOGI(TAG, "disabled");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
}

void microloop() {
    enabled = gpio_get_level(ENABLE_PIN);
    if (enabled == 0) {
        struct bno055_quaternion_double_t zeroPosition = {
            .w = 0.0,
            .x = 0.03,
            .y = -0.08,
            .z = 1.0,
        };
        struct bno055_quaternion_double_t orientation = getQuaternionOrientation(&zeroPosition);
        ESP_LOGI(TAG, "orientation:\tw %-1.2f\tx %-1.2f\ty %-1.2f\tz %-1.2f", orientation.w, orientation.x, orientation.y, orientation.z);
    } else {
        disableMotor(&motor1Conf);
        disableMotor(&motor2Conf);
        disableMotor(&motor3Conf);
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
}

void controlTask(void* params) {
    // setup BNO055
    setupBNO055();
    //calibrateBNO();

    // setup enable switch
    gpio_pad_select_gpio(ENABLE_PIN);
    gpio_set_direction(ENABLE_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(ENABLE_PIN);

    // for safety, wait for the device to be disabled before starting anything
    ESP_LOGI(TAG, "Waiting for disabled to clear safety");
    while(gpio_get_level(ENABLE_PIN) != 1) {
        vTaskDelay(20/portTICK_PERIOD_MS);
    };

    // setup motors
    initDcMotor(&motor1Conf);
    initDcMotor(&motor2Conf);
    initDcMotor(&motor3Conf);

    // setup encoders
    initEncoder(&motor1Conf);
    initEncoder(&motor2Conf);
    initEncoder(&motor3Conf);

    // setup control loops for motors
    initDcMotorController(&motor1Conf);
    initDcMotorController(&motor2Conf);
    initDcMotorController(&motor3Conf);

    // setup PID

    // angle PID keeps the bot upright
    // input is gravity vector (x or y) in m/s^2
    // output is duty cycle along axis (to be math'd out to motors)
    pid_initPIDController(&xGravityPID, 2.5, 0.00000, 0.0, 48.0, -0.3, 0.3);
    pid_setPdiSmoothing(&xGravityPID, 1.0);
    pid_reset(&xGravityPID);
    pid_initPIDController(&yGravityPID, 2.5, 0.00000, 0.0, 48.0, -0.3, 0.3);
    pid_setPdiSmoothing(&yGravityPID, 1.0);
    pid_reset(&yGravityPID);

    // position PID tries to hold place in space
    // input is the calculated distance from the encoders
    // output is correction vector in m/s^2 (setpoint for gravity PID)
    pid_initPIDController(&xPositionPid, 0.00005, 0.0, 0.0, 200.0, -0.1, 0.1);
    pid_setPdiSmoothing(&xPositionPid, 0.05);
    pid_reset(&xPositionPid);
    pid_initPIDController(&yPositionPid, 0.00005, 0.0, 0.0, 200.0, -0.1, 0.1);
    pid_setPdiSmoothing(&yPositionPid, 0.05);
    pid_reset(&yPositionPid);

    // heading PID tries to point the robot "north"
    // output is motor duty clockwise or counterclockwise
    pid_initPIDController(&headingPID, 0.1, 0.0, 0.0, 200.0, -0.07, 0.07);
    pid_setPdiSmoothing(&headingPID, 0.05);
    pid_reset(&headingPID);

    // begin main loop
    while(true) {
        controlLoop();
        //microloop();
    }
}