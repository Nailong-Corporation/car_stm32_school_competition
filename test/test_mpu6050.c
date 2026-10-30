#include "mpu6050.h"
#include "motion_config.h"
#include "line_follow.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef fake_gpio_a, fake_gpio_b;
static uint8_t registers[128];
static unsigned samples, scenario;
static unsigned probe_reads, delayed_ms;
static int16_t stationary_raw = -5;
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio) { (void)port; (void)gpio; }
void HAL_Delay(uint32_t ms) { assert(ms > 0); delayed_ms += ms; }
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *i2c) { (void)i2c; return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
        uint16_t size, uint8_t *data, uint16_t count, uint32_t timeout)
{
    (void)i2c; (void)size; (void)timeout;
    assert(address == ((scenario == 8 ? 0x69U : 0x68U) << 1U) && count == 1);
    registers[reg] = *data;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
        uint16_t size, uint8_t *data, uint16_t count, uint32_t timeout)
{
    (void)i2c; (void)address; (void)size; (void)timeout;
    memset(data, 0, count);
    if (reg == 0x75U) {
        assert(delayed_ms >= MPU6050_POWER_SETTLE_MS);
        ++probe_reads;
        if (scenario == 9 || (scenario == 7 && probe_reads <= 2) ||
            (scenario == 8 && address != (0x69U << 1U))) return HAL_ERROR;
        *data = scenario == 10 ? 0x00U : 0x68U;
    }
    else if (reg == 0x1BU) *data = scenario == 4 ? 0 : registers[reg];
    else if (reg == 0x3AU) *data = scenario == 3 ? 0 : 1;
    else if (reg == 0x47U || reg == 0x3BU) {
        if (scenario == 5) return HAL_ERROR;
        int16_t raw = stationary_raw;
        if (reg == 0x47U) {
            if (scenario == 1) raw = 970; /* This board's bad boot calibration. */
            if (scenario == 2) raw = samples % 2 ? 100 : -100; /* Mean zero but moving. */
            if (scenario == 6) raw = -970;
            if (scenario == 11 && samples < MPU6050_GYRO_BIAS_SAMPLES)
                raw = samples == 0 ? -983 : -3; /* 980LSB startup span then quiet. */
            if (scenario == 12 && samples < 4U * MPU6050_GYRO_BIAS_SAMPLES)
                raw = samples % 2 ? 100 : -100;
            ++samples;
        }
        unsigned offset = reg == 0x47U ? 0 : 12;
        data[offset] = (uint8_t)((uint16_t)raw >> 8);
        data[offset + 1] = (uint8_t)raw;
        if (reg == 0x3BU) data[4] = 0x40; /* 1g on Z. */
    }
    return HAL_OK;
}

int main(void)
{
    Mpu6050 imu;
    for (scenario = 3; scenario <= 5; ++scenario) {
        samples = 0;
        assert(!mpu6050_init(&imu) && !imu.ready);
        assert(imu.init_failure_stage == (scenario == 4 ? 6 : 4));
        float heading = 123;
        assert(!mpu6050_update(&imu, .005f, &heading) && heading == 123);
    }
    for (scenario = 11; scenario <= 12; ++scenario) {
        samples = 0;
        assert(mpu6050_init(&imu) && imu.ready);
        assert(samples == MPU6050_GYRO_BIAS_SAMPLES);
        assert(imu.gyro_z_bias_lsb == (scenario == 11 ? -3 : 0));
        assert(imu.gyro_z_calibration_span_lsb == (scenario == 11 ? 980 : 200));
    }
    for (scenario = 7; scenario <= 10; ++scenario) {
        probe_reads = delayed_ms = samples = 0;
        bool ok = mpu6050_init(&imu);
        if (scenario <= 8) {
            assert(ok && imu.ready);
            assert(probe_reads == (scenario == 7 ? 3U : 2U));
            assert(imu.address_7bit == (scenario == 8 ? 0x69U : 0x68U));
        } else {
            assert(!ok && !imu.ready && imu.init_failure_stage == 2);
            assert(probe_reads == 2U * MPU6050_PROBE_ATTEMPTS);
            assert(delayed_ms == MPU6050_POWER_SETTLE_MS +
                   (MPU6050_PROBE_ATTEMPTS - 1U) * MPU6050_PROBE_RETRY_MS);
        }
    }
    const unsigned nonblocking_scenarios[] = {1, 2, 6};
    const float expected_bias[] = {970, 0, -970};
    for (unsigned i = 0; i < 3; ++i) {
        scenario = nonblocking_scenarios[i];
        samples = 0;
        assert(mpu6050_init(&imu) && imu.ready && imu.init_failure_stage == 0);
        assert(imu.gyro_z_bias_lsb == expected_bias[i]);
        assert(samples == MPU6050_GYRO_BIAS_SAMPLES);
    }
    scenario = samples = 0;
    assert(mpu6050_init(&imu) && imu.ready && imu.init_failure_stage == 0);
    assert(samples == MPU6050_GYRO_BIAS_SAMPLES && fabsf(imu.gyro_z_bias_lsb + 5) < .001f);
    float heading = 0;
    /* Full driver-to-start gate: a wide calibration span must not strand main
     * before gray sampling; normal centered-line + button starts the controller. */
    LineFollow follow;
    const RouteStep steps[] = {{200,10,STOP,40,0,false}};
    OdometryCalibration geometry = {4.8f,4.8f,1560,1560,14.5f,-1,1,true};
    line_follow_init(&follow,geometry,steps,1,NULL,NULL);
    for (unsigned i = 0; i < LINE_CONFIRM_SAMPLES; ++i) {
        assert(mpu6050_update(&imu,.005f,&heading));
        line_follow_update_heading(&follow,gray_classify(0x18,true,0),
            (EncoderCounts){30000,30000},(i+1U)*5U,.005f,
            i == LINE_CONFIRM_SAMPLES-1U,true,heading);
    }
    assert(follow.state == LINE_TRACKING && follow.start_result == LINE_START_ACCEPTED);
    assert(follow.command.enabled);
    for (unsigned i = 0; i < 2000; ++i)
        assert(mpu6050_update(&imu, .005f, &heading));
    assert(fabsf(heading) < .00001f); /* No phantom 59.5 deg/s while stationary. */
    stationary_raw = 1635; /* Bias -5 + 100 deg/s for 0.9 seconds. */
    for (unsigned i = 0; i < 180; ++i)
        assert(mpu6050_update(&imu, .005f, &heading));
    assert(fabsf(heading - MPU6050_YAW_SIGN * 1.5707963f) < .0001f);
    scenario = 3;
    assert(!mpu6050_update(&imu, .005f, &heading));
    puts("PASS: MPU fixed median calibration; centered-line button START accepted; spike rejection; probe/bus faults; stationary/90deg integration");
    return 0;
}
