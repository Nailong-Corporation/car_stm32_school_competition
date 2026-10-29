#include "mpu6050.h"
#include "board_config.h"
#include "control_config.h"
#include "motion_config.h"
#include <math.h>
#include <string.h>

#if MPU6050_ENABLED
/* MPU6050 gyro: ±2000 dps = 16.4 LSB/(degree/s). DLPF=3 (~42 Hz);
 * sample-rate divider 0 => 1 kHz gyro output. No magnetometer or absolute yaw. */
#define MPU_REG_SMPLRT_DIV    0x19U
#define MPU_REG_CONFIG        0x1AU
#define MPU_REG_ACCEL_CONFIG  0x1CU
#define MPU_REG_GYRO_CONFIG   0x1BU
#define MPU_REG_INT_STATUS    0x3AU
#define MPU_REG_ACCEL_XOUT_H  0x3BU
#define MPU_REG_GYRO_ZOUT_H   0x47U
#define MPU_REG_PWR_MGMT_1    0x6BU
#define MPU_REG_WHO_AM_I      0x75U
#define MPU_GYRO_LSB_PER_DPS  16.4f
#define MPU_ACCEL_LSB_PER_G   16384.0f
#define DEG_TO_RAD            0.017453292519943295f

static bool read_reg(Mpu6050 *imu, uint8_t reg, uint8_t *data, uint16_t count)
{
    return HAL_I2C_Mem_Read(&imu->i2c, imu->address_7bit << 1U, reg,
                            I2C_MEMADD_SIZE_8BIT, data, count,
                            MPU6050_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool write_reg(Mpu6050 *imu, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&imu->i2c, imu->address_7bit << 1U, reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1,
                             MPU6050_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool read_gyro_z(Mpu6050 *imu, int16_t *raw)
{
    uint8_t bytes[2];
    if (!read_reg(imu, MPU_REG_GYRO_ZOUT_H, bytes, sizeof(bytes))) return false;
    *raw = (int16_t)((uint16_t)((uint16_t)bytes[0] << 8U) | bytes[1]);
    return true;
}
#if MPU6050_PITCH_STOP_ENABLED
static int16_t signed_word(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)((uint16_t)bytes[0] << 8U) | bytes[1]);
}
#endif
#endif

bool mpu6050_init(Mpu6050 *imu)
{
    memset(imu, 0, sizeof(*imu));
#if !MPU6050_ENABLED
    return false;
#else
    if ((MPU6050_YAW_SIGN != 1 && MPU6050_YAW_SIGN != -1) ||
        (MPU6050_ADDRESS_7BIT != 0x68U && MPU6050_ADDRESS_7BIT != 0x69U) ||
        MPU6050_GYRO_BIAS_SAMPLES == 0U) return false;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_AFIO_REMAP_I2C1_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = IMU_I2C_SCL_PIN | IMU_I2C_SDA_PIN;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IMU_I2C_PORT, &gpio);

    imu->i2c.Instance = I2C1;
    imu->i2c.Init.ClockSpeed = 400000U;
    imu->i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    imu->i2c.Init.OwnAddress1 = 0;
    imu->i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    imu->i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    imu->i2c.Init.OwnAddress2 = 0;
    imu->i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    imu->i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    imu->init_failure_stage = 1;
    if (HAL_I2C_Init(&imu->i2c) != HAL_OK) return false;

    uint8_t who = 0;
    imu->init_failure_stage = 2;
    imu->address_7bit = MPU6050_ADDRESS_7BIT;
    /* WHO_AM_I stays 0x68 regardless of AD0. Try both possible bus addresses. */
    if (!read_reg(imu, MPU_REG_WHO_AM_I, &who, 1) || who != 0x68U) {
        imu->address_7bit = MPU6050_ADDRESS_7BIT == 0x68U ? 0x69U : 0x68U;
        if (!read_reg(imu, MPU_REG_WHO_AM_I, &who, 1) || who != 0x68U)
            return false;
    }
    imu->init_failure_stage = 3;
    if (!write_reg(imu, MPU_REG_PWR_MGMT_1, 0x01U) ||
        !write_reg(imu, MPU_REG_CONFIG, 0x03U) ||
        !write_reg(imu, MPU_REG_SMPLRT_DIV, 0x00U) ||
#if MPU6050_PITCH_STOP_ENABLED
        !write_reg(imu, MPU_REG_ACCEL_CONFIG, 0x00U) || /* ±2g */
#endif
        !write_reg(imu, MPU_REG_GYRO_CONFIG, 0x18U)) return false;

    /* Only during startup, with motors already disabled. User must keep the car still.
     * HAL_Delay never runs inside the time-critical 5 ms control task. */
    HAL_Delay(100U);
    imu->init_failure_stage = 4;
    int64_t total = 0;
    for (uint32_t i = 0; i < MPU6050_GYRO_BIAS_SAMPLES; ++i) {
        int16_t sample;
        if (!read_gyro_z(imu, &sample)) return false;
        total += sample;
        HAL_Delay(MPU6050_GYRO_CALIBRATION_MS);
    }
    imu->gyro_z_bias_lsb = (float)total / (float)MPU6050_GYRO_BIAS_SAMPLES;
    imu->ready = true;
    imu->init_failure_stage = 0;
    return true;
#endif
}

bool mpu6050_update(Mpu6050 *imu, float dt, float *heading_rad)
{
#if !MPU6050_ENABLED
    (void)imu;
    (void)dt;
    (void)heading_rad;
    return false;
#else
    if (!imu->ready || heading_rad == NULL || !isfinite(dt) || dt <= 0.0f ||
        dt > (float)CONTROL_MAX_INTERVAL_MS / 1000.0f) return false;
    uint8_t status;
    int16_t raw;
    /* 1 kHz sensor samples vs 200 Hz control: missing DATA_RDY means stale yaw. */
    if (!read_reg(imu, MPU_REG_INT_STATUS, &status, 1) || !(status & 0x01U))
        return false;
#if MPU6050_PITCH_STOP_ENABLED
    /* One consistent 14-byte sensor burst (acc XYZ, temp, gyro XYZ) per cycle;
     * avoid another I²C transaction in the 5ms control task. */
    uint8_t sample[14];
    if (!read_reg(imu, MPU_REG_ACCEL_XOUT_H, sample, sizeof(sample))) return false;
    raw = signed_word(&sample[12]);
    if (!tilt_guard_update(&imu->pitch_guard,
            signed_word(&sample[0]) / MPU_ACCEL_LSB_PER_G,
            signed_word(&sample[2]) / MPU_ACCEL_LSB_PER_G,
            signed_word(&sample[4]) / MPU_ACCEL_LSB_PER_G, dt)) return false;
#else
    if (!read_gyro_z(imu, &raw)) return false;
#endif
    float rate = (float)MPU6050_YAW_SIGN *
                 ((float)raw - imu->gyro_z_bias_lsb) /
                 MPU_GYRO_LSB_PER_DPS * DEG_TO_RAD;
    float next = imu->heading_rad + rate * dt;
    if (!isfinite(next)) return false;
    imu->gyro_z_raw = raw;
    imu->heading_rad = next;
    *heading_rad = next;
    return true;
#endif
}
