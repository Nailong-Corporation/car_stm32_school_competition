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
}
int main(void)
{
    LineFollow f; uint32_t now=0;
    line_follow_init(&f, (OdometryCalibration){0}, NULL, 0, callback, NULL);
    for (unsigned i=0; i<10; ++i) tick(&f,&now,0,true);
    CHECK(f.state==LINE_IDLE);
    for (unsigned i=0; i<5; ++i) tick(&f,&now,0x18,false);
    CHECK(f.state==LINE_IDLE); /* rejected start never queues */
    start(&f,&now);
    for (unsigned i=0; i<500; ++i) tick(&f,&now,0x18,false);
    CHECK(f.state==LINE_TRACKING); /* no encoder stall or route deadline */
    NEAR(f.command.left,OPEN_LOOP_BASE_PWM); NEAR(f.command.right,OPEN_LOOP_BASE_PWM);
    for (unsigned i=0; i<20; ++i) tick(&f,&now,0x20,false);
    CHECK(f.command.left>f.command.right);
    for (unsigned i=0; i<20; ++i) tick(&f,&now,0x04,false);
    CHECK(f.command.left<f.command.right);
    float before=f.base, integral=f.pid.integral, derivative=f.pid.derivative;
    tick(&f,&now,0x19,false); /* selected multi-region target is still interference */
    CHECK(f.state==LINE_GAP_HOLD && f.base<=before && f.base<=OPEN_LOOP_HOLD_PWM);
    NEAR(f.pid.integral,integral); NEAR(f.pid.derivative,derivative);
    CHECK(f.command.left<f.command.right);
    float difference=f.command.right-f.command.left;
    for (unsigned i=0; i<10; ++i) tick(&f,&now,0xff,false);
    CHECK(f.command.right-f.command.left<difference);
    for (unsigned i=0; i<LINE_CONFIRM_SAMPLES; ++i) tick(&f,&now,0x18,false);
    CHECK(f.state==LINE_TRACKING && !f.abnormal); NEAR(f.pid.derivative,0);
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
