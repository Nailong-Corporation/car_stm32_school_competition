#include "mpu6050.h"
#include "motion_config.h"
#include "line_follow.h"
#include "board_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef fake_gpio_a, fake_gpio_b;
static uint8_t registers[128];
static unsigned samples, scenario;
static unsigned probe_reads, delayed_ms;
static unsigned deinit_count, write_failures;
static bool runtime_phase;
static bool bus_unlocked;
static unsigned bus_sequence, recovery_clocks;
static int16_t stationary_raw = -5;
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio) { (void)port; (void)gpio; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState state)
{
 if(state==GPIO_PIN_SET) port->output|=pins; else port->output&=(uint16_t)~pins;
 if(pins==IMU_I2C_SDA_PIN && state==GPIO_PIN_RESET) bus_sequence=1;
 if(pins==IMU_I2C_SCL_PIN && state==GPIO_PIN_RESET && bus_sequence==1) bus_sequence=2;
 if(pins==IMU_I2C_SCL_PIN && state==GPIO_PIN_SET) {
  ++recovery_clocks;
  if(bus_sequence==2) bus_sequence=3;
 }
 if(pins==IMU_I2C_SDA_PIN && state==GPIO_PIN_SET && bus_sequence==3) bus_unlocked=true;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
 if(scenario==19 && pin==IMU_I2C_SCL_PIN) return GPIO_PIN_RESET;
 if((scenario==21 || (scenario==20 && recovery_clocks<3)) && pin==IMU_I2C_SDA_PIN)
  return GPIO_PIN_RESET;
 return (port->output & pin)?GPIO_PIN_SET:GPIO_PIN_RESET;
}
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *i2c)
{ (void)i2c;return scenario==9 || scenario==17 || (scenario==18 && !bus_unlocked)?4U:0U; }
void HAL_Delay(uint32_t ms) { assert(ms > 0); delayed_ms += ms; }
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *i2c) { (void)i2c; return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *i2c)
{ (void)i2c; ++deinit_count; return scenario == 17 ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
        uint16_t size, uint8_t *data, uint16_t count, uint32_t timeout)
{
    (void)i2c; (void)size;
    assert(timeout == (runtime_phase ? MPU6050_I2C_TIMEOUT_MS : MPU6050_INIT_I2C_TIMEOUT_MS));
    assert(address == ((scenario == 8 ? 0x69U : 0x68U) << 1U) && count == 1);
    if (scenario == 16 && write_failures++ == 0) return HAL_ERROR;
    registers[reg] = *data;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *i2c, uint16_t address, uint16_t reg,
        uint16_t size, uint8_t *data, uint16_t count, uint32_t timeout)
{
    (void)i2c; (void)address; (void)size;
    assert(timeout == (runtime_phase ? MPU6050_I2C_TIMEOUT_MS : MPU6050_INIT_I2C_TIMEOUT_MS));
    memset(data, 0, count);
    if (reg == 0x75U) {
        assert(delayed_ms >= MPU6050_POWER_SETTLE_MS);
        ++probe_reads;
        if(scenario==18 && !bus_unlocked) return HAL_ERROR;
        if (scenario == 9 || scenario == 17 || (scenario == 15 && probe_reads <= 2U * MPU6050_PROBE_ATTEMPTS) ||
            (scenario == 7 && probe_reads <= 2) ||
            (scenario == 8 && address != (0x69U << 1U))) return HAL_ERROR;
        *data = scenario == 10 ? 0x00U : 0x68U;
    }
    else if (reg == 0x1BU) *data = scenario == 4 ? 0 : registers[reg];
    else if (reg == 0x3AU) *data = scenario == 3 ? 0 : 1;
    else if (reg == 0x47U || reg == 0x3BU) {
        if (scenario == 5 || (scenario == 14 && samples >= MPU6050_GYRO_BIAS_SAMPLES)) return HAL_ERROR;
        int16_t raw = stationary_raw;
        if (reg == 0x47U) {
            if (scenario == 1) raw = 970; /* This board's bad boot calibration. */
            if (scenario == 2) raw = samples % 2 ? 100 : -100; /* Mean zero but moving. */
            if (scenario == 6) raw = -970;
            if (scenario == 11 && samples < MPU6050_GYRO_BIAS_SAMPLES)
                raw = samples == 0 ? -983 : -3; /* 980LSB startup span then quiet. */
            if (scenario == 12 && samples < 4U * MPU6050_GYRO_BIAS_SAMPLES)
                raw = samples % 2 ? 100 : -100;
            if (scenario == 13 || scenario == 14) raw = samples % 2 ? 100 : -100;
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
    scenario=18;bus_unlocked=false;bus_sequence=recovery_clocks=0;
    assert(mpu6050_init(&imu) && imu.ready && bus_unlocked);
    assert(imu.bus_recovery_count==1 && imu.last_i2c_status==HAL_OK && imu.last_who_am_i==0x68);
    assert(imu.bus_line_mask==3);
    for(scenario=19;scenario<=21;++scenario) {
        recovery_clocks=0;bus_sequence=0;bus_unlocked=false;
        bool ok=mpu6050_init(&imu);
        if(scenario==20) assert(ok && imu.ready && recovery_clocks>=3);
        else {
            assert(!ok && !imu.ready && imu.init_failure_stage==9);
            assert(imu.bus_recovery_count==MPU6050_INIT_ATTEMPTS);
            assert(!(imu.bus_line_mask & (scenario==19?1U:2U)));
        }
    }
    for (scenario = 3; scenario <= 5; ++scenario) {
        samples = 0;
        assert(!mpu6050_init(&imu) && !imu.ready);
        assert(imu.init_failure_stage == (scenario == 4 ? 6 : 4));
        float heading = 123;
        assert(!mpu6050_update(&imu, .005f, &heading) && heading == 123);
    }
    for (scenario = 11; scenario <= 12; ++scenario) {
        samples = 0;
        bool calibrated=mpu6050_init(&imu);
        assert(calibrated && imu.ready);
        assert(samples == (scenario == 11 ? 1U : 5U) * MPU6050_GYRO_BIAS_SAMPLES);
        assert(imu.calibration_attempts == (scenario == 11 ? 1U : 5U));
        assert(imu.gyro_z_bias_lsb == (scenario == 11 ? -3 : -5));
        assert(imu.gyro_z_calibration_span_lsb == (scenario == 11 ? 980 : 0));
        assert(imu.gyro_z_calibration_central_span_lsb == 0);
    }
    scenario = 13; samples = 0;
    assert(!mpu6050_init(&imu) && !imu.ready && imu.init_failure_stage == 7);
    assert(imu.calibration_attempts == MPU6050_CALIBRATION_ATTEMPTS);
    assert(samples == MPU6050_CALIBRATION_ATTEMPTS * MPU6050_GYRO_BIAS_SAMPLES);
    assert(imu.gyro_z_calibration_central_span_lsb == 200);
    /* Failed real driver initialization is not a retry-only START trap. */
    LineFollow degraded;
    const RouteStep fallback_steps[]={{1,6,STOP,40,0,false,0}};
    OdometryCalibration fallback_geometry={4.8f,4.8f,1560,1560,14.5f,-1,1,true};
    line_follow_init(&degraded,fallback_geometry,fallback_steps,1,NULL,NULL);
    for(unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) {
        float ignored=0;
        bool valid=mpu6050_update(&imu,.005f,&ignored);
        assert(!valid);
        line_follow_update_heading(&degraded,gray_classify(0x18,true,0),
            (EncoderCounts){30000,30000},(i+1U)*5U,.005f,i==LINE_CONFIRM_SAMPLES-1U,valid,ignored);
    }
    assert(degraded.command.enabled && degraded.start_result==LINE_START_ACCEPTED &&
           degraded.heading_source==HEADING_ENCODER);
    /* A new stopped initialization can recover after noisy readings become quiet. */
    scenario = 0; samples = delayed_ms = 0;
    assert(mpu6050_init(&imu) && imu.ready && imu.calibration_attempts == 1);
    assert(delayed_ms == MPU6050_POWER_SETTLE_MS + MPU6050_GYRO_SETTLE_MS +
           MPU6050_GYRO_BIAS_SAMPLES * MPU6050_GYRO_CALIBRATION_MS + 5U*MPU6050_BUS_EDGE_MS);
    scenario = 14; samples = 0;
    assert(!mpu6050_init(&imu) && !imu.ready && imu.init_failure_stage == 4);
    assert(imu.init_attempts == MPU6050_INIT_ATTEMPTS);
    for (scenario = 7; scenario <= 10; ++scenario) {
        probe_reads = delayed_ms = samples = 0;
        bool ok = mpu6050_init(&imu);
        if (scenario <= 8) {
            assert(ok && imu.ready);
            assert(probe_reads == (scenario == 7 ? 3U : 2U));
            assert(imu.address_7bit == (scenario == 8 ? 0x69U : 0x68U));
        } else {
            assert(!ok && !imu.ready && imu.init_failure_stage == 2);
            assert(imu.bus_recovery_count==MPU6050_PROBE_ATTEMPTS*MPU6050_INIT_ATTEMPTS);
            if(scenario==9) assert(imu.last_i2c_status==HAL_ERROR && imu.last_i2c_error==4U);
            assert(probe_reads == 2U * MPU6050_PROBE_ATTEMPTS * MPU6050_INIT_ATTEMPTS);
            assert(delayed_ms == MPU6050_INIT_ATTEMPTS * (MPU6050_POWER_SETTLE_MS +
                   (MPU6050_PROBE_ATTEMPTS - 1U) * MPU6050_PROBE_RETRY_MS +
                   5U*MPU6050_BUS_EDGE_MS*MPU6050_PROBE_ATTEMPTS) +
                   (MPU6050_INIT_ATTEMPTS - 1U) * MPU6050_INIT_RETRY_MS);
        }
    }
    scenario = 15; probe_reads = delayed_ms = samples = deinit_count = 0;
    assert(mpu6050_init(&imu) && imu.ready && imu.init_attempts == 2);
    assert(deinit_count == MPU6050_PROBE_ATTEMPTS && probe_reads == 2U * MPU6050_PROBE_ATTEMPTS + 1U);
    scenario = 16; probe_reads = delayed_ms = samples = deinit_count = write_failures = 0;
    assert(mpu6050_init(&imu) && imu.ready && imu.init_attempts == 2 && deinit_count == 1);
    scenario = 17; probe_reads = delayed_ms = samples = deinit_count = 0;
    assert(!mpu6050_init(&imu) && !imu.ready && imu.init_failure_stage == 8 && deinit_count == 1);
    const unsigned nonblocking_scenarios[] = {1, 2, 6};
    const float expected_bias[] = {970, 0, -970};
    for (unsigned i = 0; i < 3; ++i) {
        scenario = nonblocking_scenarios[i];
        samples = 0;
        assert(!mpu6050_init(&imu) && !imu.ready && imu.init_failure_stage == 7);
        assert(imu.gyro_z_bias_lsb == expected_bias[i]);
        assert(samples == MPU6050_CALIBRATION_ATTEMPTS * MPU6050_GYRO_BIAS_SAMPLES);
    }
    scenario = samples = 0;
    assert(mpu6050_init(&imu) && imu.ready && imu.init_failure_stage == 0);
    assert(samples == MPU6050_GYRO_BIAS_SAMPLES && fabsf(imu.gyro_z_bias_lsb + 5) < .001f);
    runtime_phase = true;
    float heading = 0;
    /* Full stationary-calibration driver-to-start path. */
    LineFollow follow;
    const RouteStep steps[] = {{1, 6.0f, STOP, 40, 0, false, 0.0f}};
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
    float saved_heading=heading;
    stationary_raw=INT16_MAX;
    assert(!mpu6050_update(&imu,.005f,&heading) && heading==saved_heading);
    stationary_raw=20000;
    assert(!mpu6050_update(&imu,.005f,&heading) && heading==saved_heading);
    scenario = 3;
    assert(!mpu6050_update(&imu, .005f, &heading));
    line_follow_update_heading(&follow,gray_classify(0x18,true,0),
        (EncoderCounts){30000,30000},20,.005f,false,false,heading);
    assert(follow.command.enabled && follow.heading_source==HEADING_ENCODER &&
           follow.heading_fallback_reason==HEADING_FALLBACK_DATA);
    assert(fabsf(follow.odometry.heading_rad)<.00001f);
    puts("PASS: MPU stationary median calibration and movement rejection; centered-line button START accepted; spike rejection; probe/bus faults; stationary/90deg integration");
    return 0;
}
