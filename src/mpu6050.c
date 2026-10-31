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
    HAL_StatusTypeDef status=HAL_I2C_Mem_Read(&imu->i2c, imu->address_7bit << 1U, reg,
                            I2C_MEMADD_SIZE_8BIT, data, count,
                            imu->ready ? MPU6050_I2C_TIMEOUT_MS : MPU6050_INIT_I2C_TIMEOUT_MS);
    imu->last_i2c_status=(uint32_t)status;
    imu->last_i2c_error=HAL_I2C_GetError(&imu->i2c);
    return status==HAL_OK;
}

static bool write_reg(Mpu6050 *imu, uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status=HAL_I2C_Mem_Write(&imu->i2c, imu->address_7bit << 1U, reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1,
                             imu->ready ? MPU6050_I2C_TIMEOUT_MS : MPU6050_INIT_I2C_TIMEOUT_MS);
    imu->last_i2c_status=(uint32_t)status;
    imu->last_i2c_error=HAL_I2C_GetError(&imu->i2c);
    return status==HAL_OK;
}

static bool bus_level(uint16_t pin, GPIO_PinState level)
{
    for(uint32_t wait=0;wait<MPU6050_BUS_WAIT_MS;++wait) {
        if(HAL_GPIO_ReadPin(IMU_I2C_PORT,pin)==level) return true;
        HAL_Delay(1);
    }
    return HAL_GPIO_ReadPin(IMU_I2C_PORT,pin)==level;
}
static bool bus_drive(uint16_t pin, GPIO_PinState level)
{
    HAL_GPIO_WritePin(IMU_I2C_PORT,pin,level);
    HAL_Delay(MPU6050_BUS_EDGE_MS);
    return bus_level(pin,level);
}
/* Startup only. ST ES096 2.8.7: APB reset alone cannot unlock the F1
 * analog filter. Generate SDA/SCL transitions before resetting I2C.
 * Also clock out an interrupted slave transaction, at most nine pulses. */
static bool recover_bus(Mpu6050 *imu)
{
    ++imu->bus_recovery_count;
    __HAL_I2C_DISABLE(&imu->i2c);
    HAL_GPIO_WritePin(IMU_I2C_PORT,IMU_I2C_SCL_PIN|IMU_I2C_SDA_PIN,GPIO_PIN_SET);
    GPIO_InitTypeDef gpio={0};
    gpio.Pin=IMU_I2C_SCL_PIN|IMU_I2C_SDA_PIN;
    gpio.Mode=GPIO_MODE_OUTPUT_OD;gpio.Pull=GPIO_NOPULL;gpio.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IMU_I2C_PORT,&gpio);
    HAL_Delay(MPU6050_BUS_EDGE_MS);
    bool ok=bus_level(IMU_I2C_SCL_PIN,GPIO_PIN_SET);
    for(unsigned pulse=0;ok && pulse<9 &&
        HAL_GPIO_ReadPin(IMU_I2C_PORT,IMU_I2C_SDA_PIN)==GPIO_PIN_RESET;++pulse)
        ok=bus_drive(IMU_I2C_SCL_PIN,GPIO_PIN_RESET) &&
           bus_drive(IMU_I2C_SCL_PIN,GPIO_PIN_SET);
    ok=ok && bus_level(IMU_I2C_SDA_PIN,GPIO_PIN_SET);
    if(ok) ok=bus_drive(IMU_I2C_SDA_PIN,GPIO_PIN_RESET) &&
              bus_drive(IMU_I2C_SCL_PIN,GPIO_PIN_RESET) &&
              bus_drive(IMU_I2C_SCL_PIN,GPIO_PIN_SET) &&
              bus_drive(IMU_I2C_SDA_PIN,GPIO_PIN_SET);
    /* Release even on failure; never drive the shared bus push-pull. */
    HAL_GPIO_WritePin(IMU_I2C_PORT,gpio.Pin,GPIO_PIN_SET);
    imu->bus_line_mask=(HAL_GPIO_ReadPin(IMU_I2C_PORT,IMU_I2C_SCL_PIN)==GPIO_PIN_SET?1U:0U) |
                       (HAL_GPIO_ReadPin(IMU_I2C_PORT,IMU_I2C_SDA_PIN)==GPIO_PIN_SET?2U:0U);
    gpio.Mode=GPIO_MODE_AF_OD;HAL_GPIO_Init(IMU_I2C_PORT,&gpio);
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    return ok;
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

static bool mpu6050_init_once(Mpu6050 *imu)
{
    memset(imu, 0, sizeof(*imu));
#if !MPU6050_ENABLED
    return false;
#else
    if ((MPU6050_YAW_SIGN != 1 && MPU6050_YAW_SIGN != -1) ||
        (MPU6050_ADDRESS_7BIT != 0x68U && MPU6050_ADDRESS_7BIT != 0x69U) ||
        MPU6050_PROBE_ATTEMPTS == 0U || MPU6050_CALIBRATION_ATTEMPTS == 0U ||
        MPU6050_GYRO_BIAS_SAMPLES < 2U || MPU6050_GYRO_CALIBRATION_MS == 0U) return false;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_AFIO_REMAP_I2C1_ENABLE();
    /* Reset the old peripheral state before bounded GPIO bus recovery below. */
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
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
    HAL_Delay(MPU6050_POWER_SETTLE_MS);
    imu->init_failure_stage = 9;
    if(!recover_bus(imu)) return false;
    imu->init_failure_stage = 1;
    if (HAL_I2C_Init(&imu->i2c) != HAL_OK) return false;

    imu->init_failure_stage = 2;
    /* WHO_AM_I stays 0x68 regardless of AD0. Try both possible bus addresses. */
    bool found = false;
    for (uint32_t attempt = 0; attempt < MPU6050_PROBE_ATTEMPTS && !found; ++attempt) {
        for (unsigned alternate = 0; alternate < 2; ++alternate) {
            imu->address_7bit = alternate ? (MPU6050_ADDRESS_7BIT ^ 1U) : MPU6050_ADDRESS_7BIT;
            uint8_t who = 0;
            bool read_ok=read_reg(imu, MPU_REG_WHO_AM_I, &who, 1);
            imu->last_who_am_i=who;
            if (read_ok && who == 0x68U) {
                found = true;
                break;
            }
        }
        if (!found && attempt + 1U < MPU6050_PROBE_ATTEMPTS) {
            imu->init_failure_stage = 8;
            if (HAL_I2C_DeInit(&imu->i2c) != HAL_OK) return false;
            imu->init_failure_stage = 9;
            if(!recover_bus(imu)) return false;
            imu->init_failure_stage = 1;
            if (HAL_I2C_Init(&imu->i2c) != HAL_OK) return false;
            imu->init_failure_stage = 2;
            HAL_Delay(MPU6050_PROBE_RETRY_MS);
        }
    }
    if (!found) return false;
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
    HAL_Delay(MPU6050_GYRO_SETTLE_MS);
    /* Never integrate with an assumed scale after a configuration mismatch. */
    imu->init_failure_stage = 6;
    uint8_t config;
    if (!read_reg(imu, MPU_REG_GYRO_CONFIG, &config, 1) || config != 0x18U)
        return false;
    /* Discard the already-latched sample; each calibration read below must be fresh. */
    imu->init_failure_stage = 4;
    uint8_t status;
    if (!read_reg(imu, MPU_REG_INT_STATUS, &status, 1)) return false;
    /* Independent bounded windows while motors remain disabled. A rejected
     * window never supplies a bias; bus failures remain immediate failures. */
    int16_t samples[MPU6050_GYRO_BIAS_SAMPLES];
    for (uint32_t attempt = 0; attempt < MPU6050_CALIBRATION_ATTEMPTS; ++attempt) {
        imu->calibration_attempts = attempt + 1U;
        imu->init_failure_stage = 4;
        for (uint32_t i = 0; i < MPU6050_GYRO_BIAS_SAMPLES; ++i) {
            HAL_Delay(MPU6050_GYRO_CALIBRATION_MS);
            int16_t sample;
            if (!read_reg(imu, MPU_REG_INT_STATUS, &status, 1) || !(status & 0x01U) ||
                !read_gyro_z(imu, &sample)) return false;
            imu->gyro_z_raw = sample;
            uint32_t j = i;
            while (j > 0 && samples[j-1U] > sample) {
                samples[j] = samples[j-1U];
                --j;
            }
            samples[j] = sample;
        }
        uint32_t middle = MPU6050_GYRO_BIAS_SAMPLES / 2U;
        imu->gyro_z_bias_lsb = MPU6050_GYRO_BIAS_SAMPLES % 2U ? (float)samples[middle] :
            0.5f * ((float)samples[middle-1U] + (float)samples[middle]);
        imu->gyro_z_calibration_span_lsb = (float)((int32_t)samples[MPU6050_GYRO_BIAS_SAMPLES-1U] - samples[0]);
        imu->init_failure_stage = 7;
        uint32_t trim = MPU6050_GYRO_BIAS_SAMPLES / 20U;
        imu->gyro_z_calibration_central_span_lsb =
            (float)samples[MPU6050_GYRO_BIAS_SAMPLES-1U-trim]-samples[trim];
        if (fabsf(imu->gyro_z_bias_lsb)>MPU6050_CALIBRATION_MAX_BIAS_LSB ||
            imu->gyro_z_calibration_central_span_lsb>MPU6050_CALIBRATION_MAX_CENTRAL_SPAN_LSB) continue;
#if MPU6050_PITCH_STOP_ENABLED
        /* Static gravity baseline; a failed read only disables accel estimation. */
        uint8_t gravity[14];
        if (read_reg(imu, MPU_REG_ACCEL_XOUT_H, gravity, sizeof(gravity))) {
            float ax=signed_word(&gravity[0])/MPU_ACCEL_LSB_PER_G;
            float ay=signed_word(&gravity[2])/MPU_ACCEL_LSB_PER_G;
            float az=signed_word(&gravity[4])/MPU_ACCEL_LSB_PER_G;
            float norm=sqrtf(ax*ax+ay*ay+az*az);
            if(norm>=MPU6050_PITCH_MIN_G && norm<=MPU6050_PITCH_MAX_G) {
                imu->accel_x_baseline_g=ax;
                imu->accel_baseline_valid=true;
            }
        }
#endif
        imu->ready = true;
        imu->init_failure_stage = 0;
        return true;
    }
    return false;
#endif
}

bool mpu6050_init(Mpu6050 *imu)
{
#if !MPU6050_ENABLED
    return mpu6050_init_once(imu);
#else
    uint32_t recoveries=0;
    for (uint32_t attempt = 1; attempt <= MPU6050_INIT_ATTEMPTS; ++attempt) {
        bool ok = mpu6050_init_once(imu);
        recoveries+=imu->bus_recovery_count;
        imu->bus_recovery_count=recoveries;
        imu->init_attempts = attempt;
        if (ok) return true;
        /* Invalid configuration or exhausted stationary windows are not bus faults. */
        if (imu->init_failure_stage == 0 || imu->init_failure_stage == 7 || imu->init_failure_stage == 8 ||
            attempt == MPU6050_INIT_ATTEMPTS) return false;
        if (HAL_I2C_DeInit(&imu->i2c) != HAL_OK) {
            imu->init_failure_stage = 8;
            return false;
        }
        __HAL_RCC_I2C1_FORCE_RESET();
        __HAL_RCC_I2C1_RELEASE_RESET();
        HAL_Delay(MPU6050_INIT_RETRY_MS);
    }
    return false;
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
    imu->accel_valid=false;
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
    float ax=signed_word(&sample[0])/MPU_ACCEL_LSB_PER_G;
    float ay=signed_word(&sample[2])/MPU_ACCEL_LSB_PER_G;
    float az=signed_word(&sample[4])/MPU_ACCEL_LSB_PER_G;
    if (!tilt_guard_update(&imu->pitch_guard,ax,ay,az,dt)) return false;
    float norm=sqrtf(ax*ax+ay*ay+az*az);
    float pitch_shift=atan2f(ax,hypotf(ay,az))-
                      atan2f(imu->accel_x_baseline_g,hypotf(ay,az));
    if(imu->accel_baseline_valid && norm>=MPU6050_PITCH_MIN_G &&
       norm<=MPU6050_PITCH_MAX_G && fabsf(pitch_shift)<=BRAKING_IMU_TILT_LIMIT_DEG*DEG_TO_RAD) {
        imu->accel_x_cm_s2=BRAKING_IMU_ACCEL_SIGN*(ax-imu->accel_x_baseline_g)*981.0f;
        imu->accel_valid=true;
    }
#else
    if (!read_gyro_z(imu, &raw)) return false;
#endif
    float rate = (float)MPU6050_YAW_SIGN *
                 ((float)raw - imu->gyro_z_bias_lsb) /
                 MPU_GYRO_LSB_PER_DPS * DEG_TO_RAD;
    if(!isfinite(rate) || fabsf(rate)>MPU6050_MAX_YAW_RATE_RAD_S || raw==INT16_MIN || raw==INT16_MAX) return false;
    float next = imu->heading_rad + rate * dt;
    if (!isfinite(next)) return false;
    imu->gyro_z_raw = raw;
    imu->heading_rad = next;
    *heading_rad = next;
    return true;
#endif
}
