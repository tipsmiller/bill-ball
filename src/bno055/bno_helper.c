#include "bno_helper.h"


static const char *TAG = "bno_helper";
static const int i2c_master_port = 0;
struct bno055_t bno055;

static struct bno055_accel_offset_t accelConfigData = {
    .x = -24,
    .y = 38,
    .z = -31,
    .r = 1000,
};
static struct bno055_gyro_offset_t gyroConfigData = {
    .x = -3,
    .y = -2,
    .z = 1,
};
static struct bno055_mag_offset_t magConfigData = {
    .x = 91,
    .y = 15,
    .z = -59,
    .r = 670,
};

s8 bnoRead(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    s32 BNO055_iERROR = 0;
    //ESP_LOGI(TAG, "Reading from register %d with %d bytes", reg_addr, cnt);
    ESP_ERROR_CHECK(i2c_master_write_read_device(i2c_master_port, dev_addr, &reg_addr, 1, reg_data, cnt, 1));
    return (s8)BNO055_iERROR;
}

s8 bnoWrite(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 BNO055_iERROR = 0;
    uint8_t writeBuf[100];
    writeBuf[0] = reg_addr;
    for (int i = 1; i <= cnt; i++) {
        writeBuf[i] = reg_data[i - 1];
    }
    //ESP_LOGI(TAG, "Writing to register %d with %d bytes", reg_addr, cnt);
    ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_master_port, dev_addr, writeBuf, cnt + 1, 1));
    return (s8)BNO055_iERROR;
}

void bnoDelay(u32 ms) {
    //ESP_LOGI(TAG, "BNO is calling for a delay of %d ms", ms);
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void setupBNO055() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,         // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,         // select GPIO specific to your project
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 400000,  // select frequency specific to your project
        .clk_flags = 0,                          // you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
    };
    i2c_param_config(i2c_master_port, &conf);
    i2c_set_timeout(i2c_master_port, 0.0005 * I2C_APB_CLK_FREQ);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    bno055.bus_read = bnoRead;
    bno055.bus_write = bnoWrite;
    bno055.dev_addr = BNO055_I2C_ADDR1;
    bno055.delay_msec = bnoDelay;

    bno055_init(&bno055);
    vTaskDelay(pdMS_TO_TICKS(20));
    writeBNOConfig();
}

void writeBNOConfig() {
    bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
    vTaskDelay(pdMS_TO_TICKS(10));
    bno055_write_accel_offset(&accelConfigData);
    bno055_write_gyro_offset(&gyroConfigData);
    bno055_write_mag_offset(&magConfigData);
    bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void calibrateBNO() {
    bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    vTaskDelay(pdMS_TO_TICKS(10));
    bool calibrating = true;
    int stability = 0;
    while (calibrating) {
            u8 sysCal = 0;
            u8 magCal = 0;
            u8 accCal = 0;
            u8 gyrCal = 0;
            bno055_get_sys_calib_stat(&sysCal);
            bno055_get_mag_calib_stat(&magCal);
            bno055_get_accel_calib_stat(&accCal);
            bno055_get_gyro_calib_stat(&gyrCal);
            ESP_LOGI(TAG, "Current calibration status: sys %d, mag %d, accel %d, gyro %d", sysCal, magCal, accCal, gyrCal);
            if (sysCal == 3 && magCal == 3 && accCal == 3 && gyrCal == 3) {
                stability += 1;
            }
            if (stability > 1000) {
                bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
                vTaskDelay(pdMS_TO_TICKS(10));
                struct bno055_accel_offset_t accelOffset;
                bno055_read_accel_offset(&accelOffset);
                ESP_LOGI(TAG, "accel offsets: x (%d) y (%d) z (%d) r (%d)", accelOffset.x, accelOffset.y, accelOffset.z, accelOffset.r);
                struct bno055_gyro_offset_t gyroOffset;
                bno055_read_gyro_offset(&gyroOffset);
                ESP_LOGI(TAG, "gyro offsets: x (%d) y (%d) z (%d)", gyroOffset.x, gyroOffset.y, gyroOffset.z);
                struct bno055_mag_offset_t magOffset;
                bno055_read_mag_offset(&magOffset);
                ESP_LOGI(TAG, "mag offsets: x (%d) y (%d) z (%d) r (%d)", magOffset.x, magOffset.y, magOffset.z, magOffset.r);
                calibrating = false;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
    }
}

struct bno055_gravity_double_t getGravity() {
    struct bno055_gravity_t gravityXYZ;
    struct bno055_gravity_double_t dGravityXYZ;
    bno055_read_gravity_xyz(&gravityXYZ);
    bno055_convert_double_gravity_xyz_msq(&dGravityXYZ);
    return dGravityXYZ;
}


struct bno055_euler_double_t getYPR() {
    struct bno055_euler_t euler;
    struct bno055_euler_double_t dEuler;
    bno055_read_euler_hrp(&euler);
    bno055_convert_double_euler_hpr_rad(&dEuler);
    return dEuler;
}

 // integer components, divide by 2^14 to get fractional
struct bno055_quaternion_t getQuaternion() {
    struct bno055_quaternion_t q; 
    bno055_read_quaternion_wxyz(&q);
    return q;
}

 // integer components, divide by 2^14 to get fractional
struct bno055_quaternion_double_t quaternionProduct(struct bno055_quaternion_double_t *q, struct bno055_quaternion_double_t *p) {
    // q X p
    struct bno055_quaternion_double_t result;
    result.w = q->w*p->w - q->x*p->x - q->y*p->y - q->z*p->z;
    result.x = q->w*p->x + q->x*p->w + q->y*p->z - q->z*p->y;
    result.y = q->w*p->y + q->y*p->w + q->z*p->x - q->x*p->z;
    result.z = q->w*p->z + q->z*p->w + q->x*p->y - q->y*p->x;
    return result;
}

struct bno055_quaternion_double_t quaternionConjugate(struct bno055_quaternion_double_t *q) {
    // q*
    struct bno055_quaternion_double_t result;
    result.w = q->w;
    result.x = -q->x;
    result.y = -q->y;
    result.z = -q->z;
    return result;
}

// use p to define the relative offsets of your system from the absolute space of the BNO055.
// p should be a unit quaternion to avoid scaling issues (normalized)
struct bno055_quaternion_double_t quaternionNewPose(struct bno055_quaternion_double_t *q, struct bno055_quaternion_double_t *p) {
    // q X p
    struct bno055_quaternion_double_t qp = quaternionProduct(q, p);
    // q*
    struct bno055_quaternion_double_t qstar = quaternionConjugate(q);
    // q X p X q*
    struct bno055_quaternion_double_t result = quaternionProduct(&qp, &qstar);
    return result;
}

struct bno055_quaternion_double_t quaternionConvertDouble(struct bno055_quaternion_t *q) {
    struct bno055_quaternion_double_t result;
    result.w = (double)(q->w) / 16384.0;
    result.x = (double)(q->x) / 16384.0;
    result.y = (double)(q->y) / 16384.0;
    result.z = (double)(q->z) / 16384.0;
    return result;
}

struct bno055_quaternion_double_t getQuaternionOrientation(struct bno055_quaternion_double_t *p) {
    struct bno055_quaternion_t q = getQuaternion();
    struct bno055_quaternion_double_t qDouble = quaternionConvertDouble(&q);
    //ESP_LOGI(TAG, "orientation:\tw %d\tx %d\ty %d\tz %d", q.w, q.x, q.y, q.z);
    struct bno055_quaternion_double_t newPose = quaternionNewPose(&qDouble, p);
    return newPose;
}

struct bno055_quaternion_double_t quaternionNormalize(struct bno055_quaternion_double_t *p) {
    double magnitude = sqrt(p->w*p->w + p->x*p->x + p->y*p->y + p->z*p->z);
    ESP_LOGI(TAG, "normalizing quaternion:\tw %-1.2f\tx %-1.2f\ty %-1.2f\tz %-1.2f", p->w, p->x, p->y, p->z);
    ESP_LOGI(TAG, "quaternion has magnitude: %-1.6f", magnitude);
    struct bno055_quaternion_double_t normalized;
    normalized.w = p->w / magnitude;
    normalized.x = p->x / magnitude;
    normalized.y = p->y / magnitude;
    normalized.z = p->z / magnitude;
    ESP_LOGI(TAG, "quaternion normalized to:\tw %-1.2f\tx %-1.2f\ty %-1.2f\tz %-1.2f", normalized.w, normalized.x, normalized.y, normalized.z);
    return normalized;
}