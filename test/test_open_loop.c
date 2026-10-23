#include "gray_test_input.h"
#include "line_follow.h"
#include "motion_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#if ENCODER_CONTROL_ENABLED
#error "This suite must run with encoder control disabled"
#endif
static unsigned checks, callbacks;
#define CHECK(c) do { ++checks; assert(c); } while (0)
#define NEAR(a,b) CHECK(fabsf((a)-(b)) < 0.001f)
static void callback(size_t index, void *context)
{
    (void)index; (void)context;
    ++callbacks;
}
static GrayObservation mask(uint8_t line) { return gray_classify(line, true, 0.0f); }
static void tick(LineFollow *f, uint32_t *now, uint8_t line, bool start)
{
    *now += CONTROL_PERIOD_MS;
    /* Deliberately invalid as sampled odometry: must be completely ignored. */
    EncoderCounts garbage = {*now & 0x8000U ? 32768U : 65535U, (uint16_t)*now};
    line_follow_update(f, mask(line), garbage, *now, .005f, start);
    CHECK(f->command.left >= 0 && f->command.left <= MOTOR_PWM_LIMIT);
    CHECK(f->command.right >= 0 && f->command.right <= MOTOR_PWM_LIMIT);
    CHECK(f->route.state == ROUTE_DISABLED && !f->route.stop_pending);
    CHECK(!f->odometry.ready && !f->odometry.primed && f->odometry.distance_cm == 0);
    CHECK(f->left_target_cm_s == 0 && f->right_target_cm_s == 0);
    if (f->state == LINE_IDLE || f->state == LINE_STOPPED)
        CHECK(!f->command.enabled && f->command.left == 0 && f->command.right == 0);
}
static void start(LineFollow *f, uint32_t *now)
{
    /* Neither calibration nor a valid route is needed for PWM mode. */
    OdometryCalibration unknown = {0};
    line_follow_init(f, unknown, NULL, 0, callback, NULL);
    for (unsigned i=0; i<LINE_CONFIRM_SAMPLES; ++i)
        tick(f, now, 0x18, i == LINE_CONFIRM_SAMPLES-1U);
    CHECK(f->state == LINE_TRACKING && f->command.enabled);
    CHECK(f->stop_reason == LINE_STOP_NONE);
    CHECK(f->start_result == LINE_START_ACCEPTED);
}

/* 经过实际极性转换/滤波/启动门禁/PWM 换算，复现“按键请求有了但不转”。 */
static void raw_start_cases(void)
{
    const uint8_t line[] = {0x00, 0xff, 0x19, 0x18, 0x1f, 0x3e, 0x7c, 0xf8, 0x3f, 0x7e, 0xfc, 0x7f, 0xfe};
    for (unsigned n = 0; n < sizeof(line); ++n) {
        GraySensor sensor;
        LineFollow f;
        gray_sensor_init(&sensor);
        line_follow_init(&f, (OdometryCalibration){0}, NULL, 0, NULL, NULL);
        CHECK(f.start_result == LINE_START_NONE);
        for (unsigned i = 1; i <= 300; ++i) {
            GrayObservation obs = gray_sensor_update(&sensor, raw_for_line(line[n]), false, 0);
            line_follow_update(&f, obs, (EncoderCounts){0}, i * CONTROL_PERIOD_MS,
                               .005f, i == 10);
        }
        if (line[n] == 0x18 || n >= 4) { /* S3/S4 目标线居中，两种原始极性均可启动。 */
            CHECK(f.state == LINE_TRACKING && f.command.enabled);
            CHECK(f.start_result == LINE_START_ACCEPTED);
            CHECK(fmaxf(f.command.left, f.command.right) == OPEN_LOOP_BASE_PWM);
            if (mask(line[n]).position == 0.0f) NEAR(f.command.left, f.command.right);
            else if (mask(line[n]).position > 0) CHECK(f.command.left > f.command.right);
            else CHECK(f.command.left < f.command.right);
            /* 居中并完成加速后 255/255，CCR=ARR+1，达到满占空比。 */
            CHECK(motor_command_to_ccr(fmaxf(f.command.left, f.command.right), 1, 0, 6399) == 6400);
        } else {
            CHECK(f.state == LINE_IDLE && !f.command.enabled);
            CHECK(f.start_result == LINE_START_REJECTED_LINE);
            CHECK(f.command.left == 0 && f.command.right == 0);
            /* 拒绝结果保留，随后有白线也不能自行启动。 */
            for (unsigned i = 301; i <= 310; ++i) {
                GrayObservation obs = gray_sensor_update(&sensor, raw_for_line(0x18), false, 0);
                line_follow_update(&f, obs, (EncoderCounts){0}, i * CONTROL_PERIOD_MS,
                                   .005f, false);
            }
            CHECK(f.state == LINE_IDLE && f.start_result == LINE_START_REJECTED_LINE);
        }
    }
}
int main(void)
{
    raw_start_cases();
    LineFollow f; uint32_t now=0;
    line_follow_init(&f, (OdometryCalibration){0}, NULL, 0, callback, NULL);
    for (unsigned i=0; i<10; ++i) tick(&f,&now,0,true);
    CHECK(f.state==LINE_IDLE);
    CHECK(f.start_result==LINE_START_REJECTED_LINE);
    for (unsigned i=0; i<5; ++i) tick(&f,&now,0x18,false);
    CHECK(f.state==LINE_IDLE); /* rejected start never queues */
    start(&f,&now);
    for (unsigned i=1; i<=200; ++i) {
        NEAR(f.base,OPEN_LOOP_BASE_PWM*(float)i/200.0f);
        if (i<200) tick(&f,&now,0x18,false);
    }
    for (unsigned i=0; i<500; ++i) tick(&f,&now,0x18,false);
    CHECK(f.state==LINE_TRACKING); /* no encoder stall or route deadline */
    NEAR(f.command.left,OPEN_LOOP_BASE_PWM); NEAR(f.command.right,OPEN_LOOP_BASE_PWM);
    for (unsigned i=0; i<20; ++i) tick(&f,&now,0x20,false);
    CHECK(f.command.left>f.command.right); NEAR(f.command.left, OPEN_LOOP_BASE_PWM);
    for (unsigned i=0; i<20; ++i) tick(&f,&now,0x04,false);
    CHECK(f.command.left<f.command.right); NEAR(f.command.right, OPEN_LOOP_BASE_PWM);
    float before=f.base, integral=f.pid.integral, derivative=f.pid.derivative;
    tick(&f,&now,0x19,false); /* selected multi-region target is still interference */
    CHECK(f.state==LINE_GAP_HOLD && f.base<=before && f.base<=OPEN_LOOP_HOLD_PWM);
    NEAR(f.pid.integral,integral); NEAR(f.pid.derivative,derivative);
    CHECK(f.command.left<f.command.right);
    float difference=f.command.right-f.command.left;
    for (unsigned i=0; i<10; ++i) tick(&f,&now,0xff,false);
    CHECK(f.command.right-f.command.left<difference);
    for (unsigned i=0; i<LINE_CONFIRM_SAMPLES; ++i) tick(&f,&now,0x7f,false);
    CHECK(f.state==LINE_TRACKING && !f.abnormal); NEAR(f.pid.derivative,0);
    for (unsigned i=0; i<LINE_CONFIRM_SAMPLES; ++i) tick(&f,&now,0x7e,false);
    CHECK(f.state==LINE_TRACKING && !f.abnormal);
    uint32_t reliable=f.last_reliable_ms;
    for (unsigned i=0; i<100 && f.state!=LINE_STOPPED; ++i)
        tick(&f,&now,i%4U==3U ? 0x18 : 0,false);
    CHECK(f.stop_reason==LINE_STOP_LOST && now-reliable==INTERFERENCE_TIMEOUT_MS);
    CHECK(callbacks==0);
    for (unsigned i=0; i<5; ++i) tick(&f,&now,0x18,false);
    CHECK(f.state==LINE_STOPPED);
    tick(&f,&now,0x18,true); CHECK(f.state==LINE_TRACKING);
    line_follow_stop(&f,LINE_STOP_REQUEST);
    CHECK(f.command.left==0 && f.command.right==0 && !f.pid.primed && f.base==0);
    start(&f,&now);
    line_follow_update(&f,mask(0x18),(EncoderCounts){0},now+20,.020f,false);
    CHECK(f.stop_reason==LINE_STOP_FAULT);
    line_follow_stop(&f,LINE_STOP_REQUEST); CHECK(f.stop_reason==LINE_STOP_FAULT);
    for (unsigned i=0; i<5; ++i) tick(&f,&now,0x18,true);
    CHECK(f.state==LINE_STOPPED);
    CHECK(f.start_result==LINE_START_REJECTED_FAULT);
    now=UINT32_MAX-20U; start(&f,&now); reliable=f.last_reliable_ms;
    while (now-reliable<INTERFERENCE_TIMEOUT_MS-15U) tick(&f,&now,0,false);
    for (unsigned i=0; i<LINE_CONFIRM_SAMPLES; ++i) tick(&f,&now,0x18,false);
    CHECK(f.stop_reason==LINE_STOP_LOST); /* deadline precedes recovery, also across wrap */
    start(&f,&now);
    GrayObservation bad=mask(0x18); bad.position=NAN;
    line_follow_update(&f,bad,(EncoderCounts){0},now+5,.005f,false);
    CHECK(f.stop_reason==LINE_STOP_FAULT && !f.command.enabled && callbacks==0);
    printf("PASS: open-loop PWM, encoder bypass, interference, stop and timing; %u checks\n",checks);
    return 0;
}
