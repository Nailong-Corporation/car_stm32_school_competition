#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "tilt_guard.h"

typedef struct {
    I2C_HandleTypeDef i2c;
    float gyro_z_bias_lsb;
    float heading_rad; /* Continuous relative yaw: clockwise/right is positive. */
    int16_t gyro_z_raw;
    uint8_t address_7bit;
    uint8_t init_failure_stage;
    TiltGuard pitch_guard;
    bool ready;
} Mpu6050;

/* Must be called once with motors stopped; calibrates the stationary gyro bias. */
bool mpu6050_init(Mpu6050 *imu);
/* Called once per control period; returns false on bus/data/time failures. */
bool mpu6050_update(Mpu6050 *imu, float dt, float *heading_rad);

#endif
