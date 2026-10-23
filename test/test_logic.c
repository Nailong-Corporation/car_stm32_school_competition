#include "gray_test_input.h"
#include "control_config.h"
#include "gray_sensor.h"
#include "motor_command.h"
#include "pid.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static unsigned checks;
#define CHECK(condition) do { ++checks; assert(condition); } while (0)
#define NEAR(a, b) CHECK(fabsf((a) - (b)) < 0.001f)

static void test_sensor(void)
{
    /* 固定物理电平验证，避免仅靠测试输入转换函数掩盖极性选错。 */
    GraySensor physical;
    gray_sensor_init(&physical);
    GrayObservation center = gray_sensor_update(&physical,
#if GRAY_INVERT_ENABLED
        0xe7, /* 白线：S3/S4 低，其余高。 */
#else
        0x18, /* 黑线：S3/S4 高，其余低。 */
#endif
        false, 0.0f);
    CHECK(center.kind == GRAY_VALID && center.line_mask == 0x18);
    NEAR(center.position, 0.0f);
    /* Independent enumeration: count transitions and sum physical positions. */
    for (unsigned mask = 0; mask < 256; ++mask) {
        GraySensor sensor;
        gray_sensor_init(&sensor);
        GrayObservation obs = gray_sensor_update(&sensor, raw_for_line((uint8_t)mask), false, 0.0f);
        unsigned count = 0, regions = 0;
        float sum = 0.0f;
        for (unsigned i = 0; i < 8; ++i) {
            if (mask & (1U << i)) {
                ++count;
                sum += 2.0f * (float)i - 7.0f;
                if (i == 0 || !(mask & (1U << (i - 1U)))) ++regions;
            }
        }
        CHECK(obs.line_mask == mask);
        CHECK(obs.raw_mask == raw_for_line((uint8_t)mask));
        CHECK(obs.white_count == count && obs.region_count == regions);
        if (count == 0) CHECK(obs.kind == GRAY_NONE);
        else if (regions == 1 && count <= SENSOR_MAX_REGION_WIDTH) {
            CHECK(obs.kind == GRAY_VALID);
            NEAR(obs.position, sum / (float)count);
        } else CHECK(obs.kind == GRAY_AMBIGUOUS);
    }
    NEAR(gray_classify(0x18, false, 0.0f).position, 0.0f);
    NEAR(gray_classify(0x01, false, 0.0f).position, -7.0f);
    NEAR(gray_classify(0x80, false, 0.0f).position, 7.0f);
    NEAR(gray_classify(0x07, false, 0.0f).position, -5.0f);
    for (unsigned mask = 0; mask < 256; ++mask) {
        unsigned mirror = 0;
        for (unsigned i = 0; i < 8; ++i)
            if (mask & (1U << i)) mirror |= 1U << (7U - i);
        GrayObservation a = gray_classify((uint8_t)mask, false, 0.0f);
        GrayObservation b = gray_classify((uint8_t)mirror, false, 0.0f);
        CHECK(a.kind == b.kind);
        if (a.kind == GRAY_VALID) NEAR(a.position, -b.position);
    }
    GrayObservation selected = gray_classify(0xc3, true, -6.0f);
    CHECK(selected.kind == GRAY_VALID && selected.selected_mask == 0x03);
    NEAR(selected.position, -6.0f);
    selected = gray_classify(0xc3, true, 6.0f);
    CHECK(selected.kind == GRAY_VALID && selected.selected_mask == 0xc0);
    CHECK(gray_classify(0xc3, true, 0.0f).kind == GRAY_AMBIGUOUS);
    CHECK(gray_classify(0xc3, true, NAN).kind == GRAY_AMBIGUOUS);
    CHECK(gray_classify(0xff, true, 0.0f).kind == GRAY_AMBIGUOUS);
    GraySensor sensor;
    gray_sensor_init(&sensor);
    gray_sensor_update(&sensor, raw_for_line(0x18), false, 0.0f);
    gray_sensor_update(&sensor, raw_for_line(0x18), false, 0.0f);
    selected = gray_sensor_update(&sensor, raw_for_line(0x00), false, 0.0f);
    CHECK(selected.kind == GRAY_VALID && selected.line_mask == 0x18);
    selected = gray_sensor_update(&sensor, raw_for_line(0x00), false, 0.0f);
    CHECK(selected.kind == GRAY_NONE);
}

static void test_pid_and_mixing(void)
{
    Pid pid;
    float correction;
    pid_init(&pid, (PidConfig){8.0f, 1.0f, 0.08f, 2.0f, 0.02f});
    CHECK(pid_update(&pid, 7.0f, 0.005f, 20.0f, &correction));
    NEAR(correction, 20.0f);
    NEAR(pid.derivative, 0.0f); /* initial sample has no derivative kick */
    for (unsigned i = 0; i < 5000; ++i)
        CHECK(pid_update(&pid, 7.0f, 0.005f, 20.0f, &correction));
    NEAR(pid.integral, 0.0f); /* no windup at saturated positive output */
    pid_reset(&pid);
    CHECK(pid_update(&pid, -7.0f, 0.005f, 20.0f, &correction));
    NEAR(correction, -20.0f);
    pid_init(&pid, (PidConfig){0.0f, 1.0f, 0.0f, 2.0f, 0.02f});
    for (unsigned i = 0; i < 2000; ++i)
        CHECK(pid_update(&pid, 1.0f, 0.005f, 20.0f, &correction));
    NEAR(pid.integral, 2.0f);
    CHECK(pid_update(&pid, -1.0f, 0.005f, 20.0f, &correction));
    CHECK(pid.integral < 2.0f);
    pid_init(&pid, (PidConfig){0.0f, 0.0f, 1.0f, 2.0f, 0.02f});
    CHECK(pid_update(&pid, -1.0f, 0.005f, 1000.0f, &correction));
    CHECK(pid_update(&pid, 1.0f, 0.005f, 1000.0f, &correction));
    CHECK(pid.derivative > 0.0f && pid.derivative < 400.0f);
    pid_reset(&pid);
    CHECK(!pid.primed && pid.integral == 0.0f && pid.derivative == 0.0f);
    CHECK(!pid_update(&pid, 1.0f, 0.0f, 20.0f, &correction));
    CHECK(!pid_update(&pid, NAN, 0.005f, 20.0f, &correction));
    CHECK(!pid_update(&pid, 1.0f, 0.005f, INFINITY, &correction));
    for (int base = -10; base <= 300; ++base) {
        for (int turn = -300; turn <= 300; turn += 5) {
            MotorCommand command = motor_mix((float)base, (float)turn);
            CHECK(command.left >= 0.0f && command.left <= MOTOR_PWM_LIMIT);
            CHECK(command.right >= 0.0f && command.right <= MOTOR_PWM_LIMIT);
            if (base <= 0) CHECK(!command.enabled && command.left == 0.0f && command.right == 0.0f);
            if (base > 0 && turn > 0) CHECK(command.left >= command.right);
            if (base > 0 && turn < 0) CHECK(command.left <= command.right);
        }
    }
    CHECK(!motor_mix(NAN, 1.0f).enabled);
    CHECK(!motor_mix(10.0f, INFINITY).enabled);
    CHECK(motor_command_to_ccr(0.0f, 1.0f, 20.0f, 6399) == 0);
    CHECK(motor_command_to_ccr(-1.0f, 1.0f, 20.0f, 6399) == 0);
    CHECK(motor_command_to_ccr(255.0f, 1.0f, 0.0f, 6399) == 6400);
    CHECK(motor_command_to_ccr(300.0f, 1.0f, 0.0f, 6399) == 6400);
    CHECK(motor_command_to_ccr(50.0f, 1.0f, 0.0f, 6399) == 1255);
    CHECK(motor_command_to_ccr(NAN, 1.0f, 0.0f, 6399) == 0);
}

int main(void)
{
    test_sensor();
    test_pid_and_mixing();
    printf("PASS: 256 masks, filtering/continuity, PID and PWM; %u checks\n", checks);
    return 0;
}
