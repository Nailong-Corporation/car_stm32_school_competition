#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "tilt_guard.h"

typedef struct {
    I2C_HandleTypeDef i2c;
    float gyro_z_bias_lsb;
    float gyro_z_calibration_span_lsb; /* Max-min during startup; retained on rejected calibration. */
    float heading_rad; /* Continuous relative yaw: clockwise/right is positive. */
    float accel_x_baseline_g, accel_x_cm_s2;
    bool accel_baseline_valid, accel_valid; /* Longitudinal dynamic acceleration. */
    int16_t gyro_z_raw;
    uint8_t address_7bit;
    uint8_t init_failure_stage;
    TiltGuard pitch_guard;
    bool ready;
    float gyro_z_calibration_central_span_lsb;
    uint32_t calibration_attempts;
    uint32_t init_attempts;
    uint32_t bus_recovery_count, last_i2c_status, last_i2c_error;
    uint8_t last_who_am_i;
    uint8_t bus_line_mask; /* bit0=SCL high, bit1=SDA high after recovery release. */
} Mpu6050;

/* Must be called once with motors stopped; calibrates the stationary gyro bias. */
bool mpu6050_init(Mpu6050 *imu);
/* Called once per control period; returns false on bus/data/time failures. */
bool mpu6050_update(Mpu6050 *imu, float dt, float *heading_rad);

#endif
