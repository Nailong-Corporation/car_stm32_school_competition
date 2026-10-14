#include "control_config.h"
#include "gray_sensor.h"
#include "line_follow.h"
#include "motor_command.h"
#include "pid.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static unsigned checks;
#define CHECK(condition) do { ++checks; assert(condition); } while (0)
#define NEAR(a, b) CHECK(fabsf((a) - (b)) < 0.001f)

static GrayObservation valid(float position)
{
    GrayObservation obs = {0};
    obs.kind = GRAY_VALID;
    obs.position = position;
    return obs;
}

static GrayObservation missing(void)
{
    GrayObservation obs = {0};
    obs.kind = GRAY_NONE;
    return obs;
}

static GrayObservation ambiguous(void)
{
    GrayObservation obs = {0};
    obs.kind = GRAY_AMBIGUOUS;
    return obs;
}

static void test_sensor(void)
{
    /* Independent enumeration: count transitions and sum physical positions. */
    for (unsigned mask = 0; mask < 256; ++mask) {
        GraySensor sensor;
        gray_sensor_init(&sensor);
        GrayObservation obs = gray_sensor_update(&sensor, (uint8_t)~mask, false, 0.0f);
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
        CHECK(obs.raw_mask == (uint8_t)~mask);
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
    gray_sensor_update(&sensor, 0xe7, false, 0.0f);
    gray_sensor_update(&sensor, 0xe7, false, 0.0f);
    selected = gray_sensor_update(&sensor, 0xff, false, 0.0f);
    CHECK(selected.kind == GRAY_VALID && selected.line_mask == 0x18);
    selected = gray_sensor_update(&sensor, 0xff, false, 0.0f);
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
    CHECK(motor_command_to_ccr(255.0f, 1.0f, 0.0f, 6399) == 2510);
    CHECK(motor_command_to_ccr(50.0f, 1.0f, 0.0f, 6399) == 1255);
    CHECK(motor_command_to_ccr(NAN, 1.0f, 0.0f, 6399) == 0);
}

static void step(LineFollow *follow, GrayObservation obs, uint32_t *time, bool start)
{
    *time += CONTROL_PERIOD_MS;
    line_follow_update(follow, obs, *time, 0.005f, start);
    CHECK(follow->command.left >= 0.0f && follow->command.left <= MOTOR_PWM_LIMIT);
    CHECK(follow->command.right >= 0.0f && follow->command.right <= MOTOR_PWM_LIMIT);
    if (follow->state == LINE_IDLE || follow->state == LINE_STOPPED)
        CHECK(!follow->command.enabled && follow->command.left == 0.0f && follow->command.right == 0.0f);
}

static void start(LineFollow *follow, uint32_t *time, float position)
{
    line_follow_init(follow);
    for (unsigned i = 0; i < LINE_CONFIRM_SAMPLES; ++i)
        step(follow, valid(position), time, i == LINE_CONFIRM_SAMPLES - 1U);
    CHECK(follow->state == LINE_TRACKING);
}

static void test_states(void)
{
    LineFollow f;
    uint32_t time = 0;
    line_follow_init(&f);
    for (unsigned i = 0; i < 100; ++i) step(&f, missing(), &time, true);
    CHECK(f.state == LINE_IDLE); /* Never start blind. */
    step(&f, ambiguous(), &time, true);
    CHECK(f.state == LINE_IDLE);
    for (unsigned i = 0; i < 10; ++i) step(&f, valid(0.0f), &time, false);
    CHECK(f.state == LINE_IDLE); /* Rejected start did not queue. */
    start(&f, &time, 3.0f);
    for (unsigned i = 0; i < 80; ++i) step(&f, valid(3.0f), &time, false);
    CHECK(f.command.left > f.command.right);
    float integral = f.pid.integral;
    uint32_t last_reliable = f.last_reliable_ms;
    for (unsigned i = 0; i < 10; ++i) step(&f, missing(), &time, false);
    CHECK(f.state == LINE_GAP_HOLD && f.abnormal);
    CHECK(f.last_reliable_ms == last_reliable);
    NEAR(f.pid.integral, integral);
    step(&f, valid(1.0f), &time, false);
    CHECK(f.state == LINE_GAP_HOLD && f.last_reliable_ms == last_reliable);
    step(&f, valid(1.0f), &time, false);
    step(&f, valid(1.0f), &time, false);
    CHECK(f.state == LINE_TRACKING && !f.abnormal);
    NEAR(f.pid.derivative, 0.0f);
    start(&f, &time, -3.0f);
    for (unsigned i = 0; i < 20; ++i) step(&f, valid(-3.0f), &time, false);
    CHECK(f.command.left < f.command.right);
    last_reliable = f.last_reliable_ms;
    while ((uint32_t)(time - last_reliable) < LINE_GAP_MS)
        step(&f, missing(), &time, false);
    CHECK(f.state == LINE_SEARCH && f.command.left < f.command.right);
    for (unsigned i = 0; i < LINE_CONFIRM_SAMPLES; ++i)
        step(&f, valid(-1.0f), &time, false);
    CHECK(f.state == LINE_TRACKING);
    last_reliable = f.last_reliable_ms;
    /* Isolated valid frames amidst missing observations must not renew the budget. */
    for (unsigned i = 0; i < 100; ++i) {
        step(&f, i % 4U == 0 ? valid(-1.0f) : missing(), &time, false);
        if (f.state == LINE_STOPPED) break;
    }
    CHECK(f.state == LINE_STOPPED && f.stop_reason == LINE_STOP_LOST);
    CHECK((uint32_t)(time - last_reliable) <= LINE_GAP_MS + LINE_SEARCH_MS + CONTROL_PERIOD_MS);
    for (unsigned i = 0; i < 20; ++i) step(&f, valid(-1.0f), &time, false);
    CHECK(f.state == LINE_STOPPED);
    step(&f, valid(-1.0f), &time, true);
    CHECK(f.state == LINE_TRACKING);
    line_follow_stop(&f, LINE_STOP_REQUEST);
    CHECK(f.state == LINE_STOPPED && !f.command.enabled && !f.pid.primed);
    start(&f, &time, 0.0f);
    for (unsigned i = 0; i < LINE_GAP_MS / CONTROL_PERIOD_MS; ++i)
        step(&f, missing(), &time, false);
    CHECK(f.state == LINE_STOPPED && f.stop_reason == LINE_STOP_NO_DIRECTION);
    start(&f, &time, 3.0f);
    last_reliable = f.last_reliable_ms;
    for (unsigned i = 0; i < 40; ++i) {
        GrayObservation obs = i % 3U == 0 ? ambiguous() : (i % 3U == 1 ? missing() : valid(3.0f));
        step(&f, obs, &time, false);
        if (f.state == LINE_STOPPED) break;
    }
    CHECK(f.state == LINE_STOPPED && f.stop_reason == LINE_STOP_AMBIGUOUS);
    CHECK((uint32_t)(time - last_reliable) == LINE_AMBIGUITY_MS);
    start(&f, &time, 3.0f);
    last_reliable = f.last_reliable_ms;
    while ((uint32_t)(time - last_reliable) < LINE_GAP_MS + 20U)
        step(&f, missing(), &time, false);
    CHECK(f.state == LINE_SEARCH);
    step(&f, ambiguous(), &time, false);
    CHECK(f.state == LINE_STOPPED && f.stop_reason == LINE_STOP_AMBIGUOUS);
    start(&f, &time, 1.0f);
    line_follow_update(&f, valid(1.0f), time + 20U, 0.020f, false);
    CHECK(f.state == LINE_STOPPED && f.stop_reason == LINE_STOP_FAULT);
    line_follow_stop(&f, LINE_STOP_REQUEST);
    CHECK(f.stop_reason == LINE_STOP_FAULT); /* stop button cannot unlock a fault */
    for (unsigned i = 0; i < 5; ++i) step(&f, valid(1.0f), &time, true);
    CHECK(f.state == LINE_STOPPED); /* Timing fault requires reset. */
    start(&f, &time, 1.0f);
    line_follow_update(&f, valid(NAN), time + 5U, 0.005f, false);
    CHECK(f.state == LINE_STOPPED && f.stop_reason == LINE_STOP_FAULT);
    time = UINT32_MAX - 30U;
    start(&f, &time, 3.0f);
    for (unsigned i = 0; i < 10; ++i) step(&f, missing(), &time, false);
    CHECK(f.state == LINE_GAP_HOLD); /* uint32_t millisecond wrap */
    for (unsigned i = 0; i < LINE_CONFIRM_SAMPLES; ++i)
        step(&f, valid(1.0f), &time, false);
    CHECK(f.state == LINE_TRACKING);
    /* Recovery confirmations arriving at the hard deadline must still stop. */
    start(&f, &time, 3.0f);
    last_reliable = f.last_reliable_ms;
    while ((uint32_t)(time - last_reliable) < LINE_GAP_MS + LINE_SEARCH_MS - 15U)
        step(&f, missing(), &time, false);
    for (unsigned i = 0; i < LINE_CONFIRM_SAMPLES; ++i)
        step(&f, valid(3.0f), &time, false);
    CHECK(f.state == LINE_STOPPED && f.stop_reason == LINE_STOP_LOST);
    start(&f, &time, 3.0f);
    step(&f, ambiguous(), &time, false);
    last_reliable = f.last_reliable_ms;
    while ((uint32_t)(time - last_reliable) < LINE_AMBIGUITY_MS - 15U)
        step(&f, ambiguous(), &time, false);
    for (unsigned i = 0; i < LINE_CONFIRM_SAMPLES; ++i)
        step(&f, valid(3.0f), &time, false);
    CHECK(f.state == LINE_STOPPED && f.stop_reason == LINE_STOP_AMBIGUOUS);
}

static void test_pipeline(void)
{
    GraySensor sensor;
    LineFollow follow;
    uint32_t time = 0;
    gray_sensor_init(&sensor);
    line_follow_init(&follow);
    for (unsigned i = 0; i < 20; ++i) {
        time += 5U;
        GrayObservation obs = gray_sensor_update(&sensor, 0xdf, false, 0.0f); /* S5 white */
        line_follow_update(&follow, obs, time, 0.005f, i == 19U);
    }
    CHECK(follow.state == LINE_TRACKING && follow.command.left > follow.command.right);
    for (unsigned i = 0; i < 8; ++i) {
        time += 5U;
        GrayObservation obs = gray_sensor_update(&sensor, 0xff,
            line_follow_have_recent_position(&follow, time), follow.reliable_position);
        line_follow_update(&follow, obs, time, 0.005f, false);
    }
    CHECK(follow.state == LINE_GAP_HOLD);
    for (unsigned i = 0; i < 5; ++i) {
        time += 5U;
        GrayObservation obs = gray_sensor_update(&sensor, 0xdf,
            line_follow_have_recent_position(&follow, time), follow.reliable_position);
        line_follow_update(&follow, obs, time, 0.005f, false);
    }
    CHECK(follow.state == LINE_TRACKING && !follow.abnormal);
    uint32_t last_stable = follow.last_reliable_ms;
    for (unsigned i = 0; i < 100; ++i) {
        time += 5U;
        uint8_t raw = i % 4U == 3U ? 0xdf : 0xff; /* isolated raw noise */
        GrayObservation obs = gray_sensor_update(&sensor, raw,
            line_follow_have_recent_position(&follow, time), follow.reliable_position);
        line_follow_update(&follow, obs, time, 0.005f, false);
        if (follow.state == LINE_STOPPED) break;
    }
    CHECK(follow.state == LINE_STOPPED && follow.stop_reason == LINE_STOP_LOST);
    CHECK((uint32_t)(time - last_stable) <= LINE_GAP_MS + LINE_SEARCH_MS + 5U);
}

int main(void)
{
    test_sensor();
    test_pid_and_mixing();
    test_states();
    test_pipeline();
    printf("PASS: 256 masks, sensor filtering/continuity, PID, PWM, recovery/timeouts/wrap; %u checks\n", checks);
    return 0;
}
