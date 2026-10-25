#include "line_follow.h"
#include "motion_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#if !ENCODER_CONTROL_ENABLED || PRESET_ROUTE_ENABLED
#error "This suite requires encoder control with preset route disabled"
#endif
static unsigned checks, callbacks;
#define CHECK(c) do { ++checks; assert(c); } while (0)
#define NEAR(a,b) CHECK(fabsf((a)-(b)) < 0.001f)
static OdometryCalibration calibration(void)
{
    /* 合成几何在生产距离系数生效后保持 0.01 cm/count，不依赖实车标定值改变输入尺度。 */
    return (OdometryCalibration){1.0f/(3.14159265359f*ODOMETRY_DISTANCE_SCALE),
                                 1.0f/(3.14159265359f*ODOMETRY_DISTANCE_SCALE),
                                 100.0f, 100.0f, 10.0f, 1, 1, true};
}
static void callback(size_t index, void *context)
{
    (void)index; (void)context;
    ++callbacks;
}
static GrayObservation mask(uint8_t line) { return gray_classify(line, true, 0.0f); }
static void tick(LineFollow *f, EncoderCounts *c, uint32_t *now,
                 int left, int right, uint8_t line, bool request)
{
    c->left = (uint16_t)(c->left + left);
    c->right = (uint16_t)(c->right + right);
    *now += CONTROL_PERIOD_MS;
    line_follow_update(f, mask(line), *c, *now, .005f, request);
    CHECK(f->route.state == ROUTE_DISABLED && !f->route.stop_pending);
    CHECK(f->route.steps == NULL && f->route.count == 0 && f->route.stop_callback == NULL);
    CHECK(f->state != LINE_JUNCTION && callbacks == 0);
    CHECK(f->command.left >= 0 && f->command.left <= MOTOR_PWM_LIMIT);
    CHECK(f->command.right >= 0 && f->command.right <= MOTOR_PWM_LIMIT);
    CHECK(f->left_target_cm_s >= 0 && f->left_target_cm_s <= WHEEL_TARGET_MAX_CM_S);
    CHECK(f->right_target_cm_s >= 0 && f->right_target_cm_s <= WHEEL_TARGET_MAX_CM_S);
    if (f->state == LINE_IDLE || f->state == LINE_STOPPED)
        CHECK(!f->command.enabled && f->command.left == 0 && f->command.right == 0);
}
static void start(LineFollow *f, EncoderCounts *c, uint32_t *now,
                  const RouteStep *steps, size_t count)
{
    *c = (EncoderCounts){0};
    line_follow_init(f, calibration(), steps, count, callback, NULL);
    for (unsigned i = 0; i < LINE_CONFIRM_SAMPLES; ++i)
        tick(f,c,now,0,0,0x18,i == LINE_CONFIRM_SAMPLES-1U);
    CHECK(f->state == LINE_TRACKING && f->command.enabled && f->odometry.ready);
    CHECK(f->start_result == LINE_START_ACCEPTED);
}
int main(void)
{
    LineFollow f; EncoderCounts c = {0}; uint32_t now = 0;
    OdometryCalibration unknown = calibration(); unknown.confirmed = false;
    line_follow_init(&f,unknown,NULL,0,callback,NULL);
    tick(&f,&c,&now,0,0,0x18,true);
    CHECK(f.stop_reason == LINE_STOP_CALIBRATION);
    CHECK(f.start_result == LINE_START_REJECTED_CALIBRATION);

    line_follow_init(&f,calibration(),NULL,0,callback,NULL);
    for (unsigned i=0; i<10; ++i) tick(&f,&c,&now,0,0,0,true);
    CHECK(f.state == LINE_IDLE);
    CHECK(f.start_result == LINE_START_REJECTED_LINE);
    for (unsigned i=0; i<5; ++i) tick(&f,&c,&now,0,0,0x18,false);
    CHECK(f.state == LINE_IDLE); /* Rejected start is not queued. */
    start(&f,&c,&now,NULL,0); /* A missing route must not block closed-loop tracking. */
    for (unsigned i=0; i<200; ++i) tick(&f,&c,&now,3,3,0x18,false);
    NEAR(f.base,LINE_FOLLOW_SPEED_CM_S);
    CHECK(f.speed.left.primed && f.speed.right.primed && f.odometry.distance_cm > 0);
    for (unsigned i=0; i<20; ++i) tick(&f,&c,&now,3,3,0x20,false);
    CHECK(f.left_target_cm_s > f.right_target_cm_s && f.command.left > f.command.right);
    for (unsigned i=0; i<20; ++i) tick(&f,&c,&now,3,3,0x04,false);
    CHECK(f.left_target_cm_s < f.right_target_cm_s && f.command.left < f.command.right);

    /* A supplied LEFT/STOP table must neither steer, set speed nor stop this mode. */
    const RouteStep table[] = {{1,.3f,LEFT,4, 0.0f, true},{1,.3f,STOP,4, 0.0f, true}};
    start(&f,&c,&now,table,2);
    uint32_t began = now;
    while ((uint32_t)(now-began) <= 30000U)
        tick(&f,&c,&now,3,3,0x18,false);
    CHECK(f.state == LINE_TRACKING && f.odometry.distance_cm > 55.0f);
    NEAR(f.base,LINE_FOLLOW_SPEED_CM_S); /* Ignores speed=4 in the table. */
    float integral=f.pid.integral, derivative=f.pid.derivative;
    float distance=f.odometry.distance_cm;
    const uint8_t wide = (uint8_t)((1U << (SENSOR_MAX_REGION_WIDTH+1U))-1U);
    const uint8_t graphics[] = {wide,(uint8_t)(wide << (7U-SENSOR_MAX_REGION_WIDTH)),0xff,0x19,0};
    for (size_t i=0; i<sizeof(graphics); ++i) {
        tick(&f,&c,&now,3,3,graphics[i],false);
        CHECK(f.state == LINE_GAP_HOLD && f.base <= HOLD_SPEED_CM_S);
        NEAR(f.pid.integral,integral); NEAR(f.pid.derivative,derivative);
    }
    CHECK(f.odometry.distance_cm > distance);
    tick(&f,&c,&now,8,2,0xff,false);
    CHECK(f.left_target_cm_s < f.right_target_cm_s); /* Correct measured heading drift. */
    for (unsigned i=0; i<LINE_CONFIRM_SAMPLES; ++i) {
        tick(&f,&c,&now,3,3,0x18,false);
        if (i+1U < LINE_CONFIRM_SAMPLES) CHECK(f.state == LINE_GAP_HOLD);
    }
    CHECK(f.state == LINE_TRACKING && !f.abnormal); NEAR(f.pid.derivative,0);

    uint32_t reliable=f.last_reliable_ms;
    tick(&f,&c,&now,1,1,0,false);
    while (f.state != LINE_STOPPED)
        tick(&f,&c,&now,1,1,(now/5U)%4U == 0 ? 0x18 : 0,false);
    CHECK(f.stop_reason == LINE_STOP_LOST && now-reliable == INTERFERENCE_TIMEOUT_MS);
    for (unsigned i=0; i<5; ++i) tick(&f,&c,&now,0,0,0x18,false);
    CHECK(f.state == LINE_STOPPED);
    tick(&f,&c,&now,0,0,0x18,true); CHECK(f.state == LINE_TRACKING);
    line_follow_stop(&f,LINE_STOP_REQUEST);
    CHECK(!f.command.enabled && !f.pid.primed && !f.speed.left.primed);

    start(&f,&c,&now,NULL,0); reliable=f.last_reliable_ms;
    for (unsigned i=0; i<30 && f.state != LINE_STOPPED; ++i)
        tick(&f,&c,&now,50,50,0,false);
    CHECK(f.stop_reason == LINE_STOP_LOST && now-reliable < INTERFERENCE_TIMEOUT_MS);
    now=UINT32_MAX-20U; start(&f,&c,&now,NULL,0); reliable=f.last_reliable_ms;
    while ((uint32_t)(now-reliable) < INTERFERENCE_TIMEOUT_MS-15U)
        tick(&f,&c,&now,1,1,0,false);
    for (unsigned i=0; i<LINE_CONFIRM_SAMPLES; ++i) tick(&f,&c,&now,1,1,0x18,false);
    CHECK(f.stop_reason == LINE_STOP_LOST); /* Deadline precedes recovery across wrap. */

    start(&f,&c,&now,NULL,0);
    for (unsigned i=0; i<150 && f.state != LINE_STOPPED; ++i)
        tick(&f,&c,&now,0,0,0x18,false);
    CHECK(f.stop_reason == LINE_STOP_ENCODER); /* Speed-loop stall guard remains active. */
    tick(&f,&c,&now,3,3,0x18,true); CHECK(f.state == LINE_STOPPED);
    start(&f,&c,&now,NULL,0);
    tick(&f,&c,&now,1000,1000,0x18,false);
    CHECK(f.stop_reason == LINE_STOP_ENCODER);
    start(&f,&c,&now,NULL,0);
    line_follow_update(&f,mask(0x18),c,now+20,.020f,false);
    CHECK(f.stop_reason == LINE_STOP_FAULT && !f.command.enabled);
    line_follow_stop(&f,LINE_STOP_REQUEST); CHECK(f.stop_reason == LINE_STOP_FAULT);
    printf("PASS: encoder line-only, route bypass, interference and safety; %u checks\n",checks);
    return 0;
}
