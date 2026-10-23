#include "gray_test_input.h"
#include "line_follow.h"
#include "motion_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static unsigned checks;
#define CHECK(c) do { ++checks; assert(c); } while (0)
#define NEAR(a,b) CHECK(fabsf((a)-(b)) < 0.001f)
static OdometryCalibration calibration(void)
{
    /* Synthetic test geometry only: circumference 1 cm, 100 x4 counts/rev. */
    return (OdometryCalibration){1.0f/3.14159265359f, 1.0f/3.14159265359f,
                                 100.0f, 100.0f, 10.0f, 1, 1, true};
}
static GrayObservation mask(uint8_t value) { return gray_classify(value, true, 0.0f); }
static const RouteStep long_route[] = {{1000.0f, 2.0f, STOP, 12}};
static unsigned callbacks;
static void stopped_callback(size_t index, void *context)
{
    LineFollow *f = context;
    CHECK(index == f->route.index && f->state == LINE_STOPPED);
    CHECK(!f->command.enabled && f->command.left == 0 && f->command.right == 0);
    CHECK(!f->pid.primed && !f->speed.left.primed && !f->speed.right.primed);
    ++callbacks;
    route_dispatch_stop_callback(&f->route); /* reentry cannot invoke it twice */
}
static void tick(LineFollow *f, EncoderCounts *counts, uint32_t *now,
                  int l, int r, uint8_t line, bool start)
{
    counts->left = (uint16_t)(counts->left + l);
    counts->right = (uint16_t)(counts->right + r);
    *now += 5U;
    line_follow_update(f, mask(line), *counts, *now, 0.005f, start);
    CHECK(f->command.left >= 0 && f->command.left <= MOTOR_PWM_LIMIT);
    CHECK(f->command.right >= 0 && f->command.right <= MOTOR_PWM_LIMIT);
    CHECK(f->left_target_cm_s >= 0 && f->left_target_cm_s <= WHEEL_TARGET_MAX_CM_S);
    CHECK(f->right_target_cm_s >= 0 && f->right_target_cm_s <= WHEEL_TARGET_MAX_CM_S);
    if (f->state == LINE_STOPPED || f->state == LINE_IDLE) CHECK(!f->command.enabled);
}
static void start(LineFollow *f, EncoderCounts *counts, uint32_t *now,
                   const RouteStep *steps, size_t count)
{
    *counts = (EncoderCounts){0};
    line_follow_init(f, calibration(), steps, count, stopped_callback, f);
    for (unsigned i = 0; i < LINE_CONFIRM_SAMPLES; ++i)
        tick(f, counts, now, 0, 0, 0x18, i == LINE_CONFIRM_SAMPLES-1U);
    CHECK(f->state == LINE_TRACKING);
}
static void test_odometry(void)
{
    Odometry o;
    OdometryCalibration c = calibration();
    odometry_init(&o, c);
    CHECK(o.ready);
    CHECK(odometry_update(&o, (EncoderCounts){65530, 65530}, .005f));
    CHECK(odometry_update(&o, (EncoderCounts){4, 4}, .005f));
    NEAR(o.distance_cm, .1f); NEAR(o.heading_rad, 0);
    CHECK(odometry_update(&o, (EncoderCounts){65534, 65534}, .005f));
    NEAR(o.distance_cm, .04f);
    CHECK(odometry_update(&o, (EncoderCounts){8, 65534}, .005f));
    NEAR(o.heading_rad, .01f);
    CHECK(!odometry_update(&o, (EncoderCounts){1000, 65534}, .005f));
    CHECK(!odometry_update(&o, (EncoderCounts){32776, 65534}, .005f));
    c.right_sign = -1; odometry_init(&o, c);
    CHECK(odometry_update(&o, (EncoderCounts){0, 0}, .005f));
    CHECK(odometry_update(&o, (EncoderCounts){10, 65526}, .005f));
    NEAR(o.distance_cm, .1f); NEAR(o.heading_rad, 0);
    c.confirmed = false; odometry_init(&o,c); CHECK(!o.ready);
    c = calibration(); c.track_cm = NAN; odometry_init(&o,c); CHECK(!o.ready);
    c = calibration(); c.left_counts_per_rev = 0; odometry_init(&o,c); CHECK(!o.ready);
    c = calibration(); c.left_sign = 0; odometry_init(&o,c); CHECK(!o.ready);
    c = calibration(); c.left_counts_per_rev = 1e9f; odometry_init(&o,c); CHECK(!o.ready);
}
static void test_speed(void)
{
    SpeedControl s; MotorCommand out;
    speed_control_init(&s);
    float l=0, r=0;
    for (unsigned i=0; i<800; ++i) {
        CHECK(speed_control_update(&s,10,8,l,r,.005f,&out));
        CHECK(out.left>=0 && out.left<=MOTOR_PWM_LIMIT);
        l += (out.left/3.0f-l)*.005f/.15f;
        r += (out.right/3.0f-r)*.005f/.20f;
    }
    CHECK(fabsf(l-10)<.05f && fabsf(r-8)<.05f);
    CHECK(speed_control_update(&s,0,0,l,r,.005f,&out));
    CHECK(!out.enabled && out.left==0 && out.right==0 && !s.left.primed);
    Pid p; pid_init(&p, (PidConfig){2,8,0,10,.02f});
    float v;
    for (unsigned i=0;i<100;++i) CHECK(pid_update_bounded(&p,-10,.005f,0,100,0,&v));
    NEAR(v,0); NEAR(p.integral,0);
    for (unsigned i=0;i<100;++i) CHECK(pid_update_bounded(&p,100,.005f,0,100,0,&v));
    NEAR(v,100); NEAR(p.integral,0);
    speed_control_reset(&s);
    bool ok=true;
    for(unsigned i=0;i<100 && ok;++i) ok=speed_control_update(&s,10,10,0,0,.005f,&out);
    CHECK(!ok && !out.enabled);
    CHECK(!speed_control_update(&s,10,10,-2,10,.005f,&out));
    CHECK(!speed_control_update(&s,NAN,10,0,0,.005f,&out));
}
static void test_left_encoder_direction(void)
{
    /* Replay the reported startup snapshot: left 0 -> 65527 (-9), right 0 -> 2.
     * Use the reported geometry; a sign correction must preserve reverse-feedback protection. */
    for (int sign = 1; sign >= -1; sign -= 2) {
        LineFollow f; EncoderCounts counts = {0}; uint32_t now = 0;
        OdometryCalibration c = {4.8f, 4.8f, 1560.0f, 1560.0f, 14.5f, sign, 1, true};
        line_follow_init(&f,c,long_route,1,NULL,NULL);
        for (unsigned i=0; i<LINE_CONFIRM_SAMPLES; ++i)
            tick(&f,&counts,&now,0,0,0x18,i==LINE_CONFIRM_SAMPLES-1U);
        CHECK(f.start_result==LINE_START_ACCEPTED && f.state==LINE_TRACKING);
        tick(&f,&counts,&now,-9,2,0x18,false);
        CHECK(counts.left==65527U && counts.right==2U);
        if (sign==1) {
            CHECK(f.stop_reason==LINE_STOP_ENCODER && !f.command.enabled);
            tick(&f,&counts,&now,0,0,0x18,true);
            CHECK(f.start_result==LINE_START_REJECTED_FAULT);
        } else {
            CHECK(f.odometry.left_delta==9 && f.odometry.right_delta==2);
            CHECK(f.odometry.left_speed_cm_s>0 && f.odometry.right_speed_cm_s>0);
            CHECK(f.state==LINE_TRACKING && f.command.enabled);
            for (unsigned i=0; i<120; ++i)
                tick(&f,&counts,&now,-6,6,0x18,false);
            CHECK(f.state==LINE_TRACKING && f.stop_reason==LINE_STOP_NONE);
        }
    }
}
static void confirm(RoutePlanner *p, uint8_t shape, float start_cm, uint32_t t)
{
    for(unsigned i=0;i<3;++i) route_update(p,mask(shape),start_cm+.06f*(float)i,0,t+5U*i);
}
static void test_routes(void)
{
    RoutePlanner p;
    RouteStep table[]={{10,2,LEFT,8},{10,2,STOP,8}};
    route_init(&p,table,2,NULL,NULL); CHECK(p.valid);
    for(unsigned a=0;a<3;++a) {
        table[0].action=(Action)a;
        route_init(&p,table,2,NULL,NULL); route_start(&p,0,0);
        confirm(&p,0xff,5,10); CHECK(p.state==ROUTE_FOLLOW);
        uint8_t shape = a==LEFT ? 0x1f : a==RIGHT ? 0xf8 : 0xff;
        confirm(&p,shape,9,30); CHECK(p.state==ROUTE_EXECUTING);
        /* Same marker cannot consume the next step while executing. */
        route_update(&p,mask(0xff),9.2f,0,50); CHECK(p.index==0);
        float heading=a==LEFT ? -1.45f : a==RIGHT ? 1.45f : 0;
        float distance=a==STRAIGHT ? 12.3f : 10.0f;
        if (a!=STRAIGHT) {
            float early=a==LEFT ? -1.2f : 1.2f;
            for (unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i)
                route_update(&p,mask(0x18),distance,early,51+2*i);
            CHECK(p.state==ROUTE_EXECUTING); /* A centered graphic at 69° cannot exit. */
        }
        for(unsigned i=0;i<3;++i) route_update(&p,mask(0x18),distance,heading,60+5*i);
        CHECK(p.state==ROUTE_CLEARING && p.index==0);
        route_update(&p,mask(0xff),distance+3.1f,heading,80); CHECK(p.index==0);
        for(unsigned i=0;i<3;++i) route_update(&p,mask(0x18),distance+3.2f,heading,85+5*i);
        CHECK(p.index==1 && p.state==ROUTE_FOLLOW);
        confirm(&p,0xff,distance+3.3f,100); CHECK(p.state==ROUTE_FOLLOW);
        float origin=p.origin_cm;
        confirm(&p,0xff,origin+9,120); CHECK(p.state==ROUTE_FINISHED && p.stop_pending);
    }
    route_init(&p,table,2,NULL,NULL); route_start(&p,0,0);
    route_update(&p,mask(0x18),12.01f,0,50); CHECK(p.state==ROUTE_FAILED);
    route_init(&p,table,2,NULL,NULL); route_start(&p,0,0);
    for(unsigned i=0;i<3;++i) route_update(&p,mask(0xff),9,0,10+5*i);
    CHECK(p.state==ROUTE_CANDIDATE); /* stationary pattern cannot confirm */
    route_update(&p,mask(0xff),9,0,110); CHECK(p.state==ROUTE_FAILED);
    table[0].action=LEFT;
    route_init(&p,table,2,NULL,NULL); route_start(&p,0,0); confirm(&p,0xff,9,10);
    route_update(&p,mask(0x18),9.2f,+.5f,30); CHECK(p.state==ROUTE_FAILED); /* wrong way */
    route_init(&p,table,2,NULL,NULL); route_start(&p,0,0); confirm(&p,0xff,9,10);
    route_update(&p,mask(0xff),9.2f,0,20+JUNCTION_ACTION_TIMEOUT_MS);
    CHECK(p.state==ROUTE_FAILED);
    route_init(&p,table,2,NULL,NULL); route_start(&p,0,UINT32_MAX-10U);
    confirm(&p,0xff,9,UINT32_MAX-5U); CHECK(p.state==ROUTE_EXECUTING);
    table[0].speed=0; route_init(&p,table,2,NULL,NULL); CHECK(!p.valid);
    table[0].speed=8; table[1].action=LEFT; route_init(&p,table,2,NULL,NULL); CHECK(!p.valid);
    table[1].action=STOP; table[0].action=STOP; route_init(&p,table,2,NULL,NULL); CHECK(!p.valid);
    route_init(&p,NULL,0,NULL,NULL); CHECK(!p.valid);
}
static void test_controller(void)
{
    LineFollow f; EncoderCounts c={0}; uint32_t now=0;
    OdometryCalibration invalid=calibration(); invalid.confirmed=false;
    line_follow_init(&f,invalid,long_route,1,NULL,NULL);
    tick(&f,&c,&now,0,0,0x18,true);
    CHECK(f.stop_reason==LINE_STOP_CALIBRATION && !f.command.enabled);
    CHECK(f.start_result==LINE_START_REJECTED_CALIBRATION);
    line_follow_init(&f,calibration(),NULL,0,NULL,NULL);
    for(unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i)
        tick(&f,&c,&now,0,0,0x18,i==LINE_CONFIRM_SAMPLES-1U);
    CHECK(f.stop_reason==LINE_STOP_ROUTE && !f.command.enabled);
    CHECK(f.start_result==LINE_START_REJECTED_ROUTE);
    line_follow_init(&f,calibration(),long_route,1,NULL,NULL);
    for(unsigned i=0;i<10;++i) tick(&f,&c,&now,0,0,0,true);
    CHECK(f.state==LINE_IDLE);
    CHECK(f.start_result==LINE_START_REJECTED_LINE);
    for(unsigned i=0;i<5;++i) tick(&f,&c,&now,0,0,0x18,false);
    CHECK(f.state==LINE_IDLE); /* no queued start */
    start(&f,&c,&now,long_route,1);
    CHECK(f.start_result==LINE_START_ACCEPTED);
    tick(&f,&c,&now,0,0,0x18,true);
    CHECK(f.state==LINE_TRACKING && f.start_result==LINE_START_ALREADY_RUNNING);
    for(unsigned i=0;i<20;++i) tick(&f,&c,&now,5,5,0x20,false);
    CHECK(f.left_target_cm_s>f.right_target_cm_s);
    float integral=f.pid.integral, derivative=f.pid.derivative;
    tick(&f,&c,&now,5,5,0x19,false); /* multiple regions even if classifier selects center */
    CHECK(f.state==LINE_GAP_HOLD && f.route.index==0);
    NEAR(f.pid.integral,integral); NEAR(f.pid.derivative,derivative);
    tick(&f,&c,&now,8,2,0xff,false); /* clockwise disturbance -> left correction */
    CHECK(f.left_target_cm_s<f.right_target_cm_s);
    for(unsigned i=0;i<3;++i) tick(&f,&c,&now,3,3,0x18,false);
    CHECK(f.state==LINE_TRACKING && !f.abnormal); NEAR(f.pid.derivative,0);
    uint32_t reliable=f.last_reliable_ms;
    for(unsigned i=0;i<100 && f.state!=LINE_STOPPED;++i)
        tick(&f,&c,&now,1,1,i%4==3 ? 0x18 : 0,false);
    CHECK(f.stop_reason==LINE_STOP_LOST && now-reliable==INTERFERENCE_TIMEOUT_MS);
    for(unsigned i=0;i<5;++i) tick(&f,&c,&now,0,0,0x18,false);
    CHECK(f.state==LINE_STOPPED);
    tick(&f,&c,&now,0,0,0x18,true); CHECK(f.state==LINE_TRACKING);
    line_follow_update(&f,mask(0x18),c,now+20,.020f,false);
    CHECK(f.stop_reason==LINE_STOP_FAULT);
    line_follow_stop(&f,LINE_STOP_REQUEST); CHECK(f.stop_reason==LINE_STOP_FAULT);
    for(unsigned i=0;i<5;++i) tick(&f,&c,&now,0,0,0x18,true);
    CHECK(f.state==LINE_STOPPED);
    CHECK(f.start_result==LINE_START_REJECTED_FAULT);
    start(&f,&c,&now,long_route,1);
    for(unsigned i=0;i<150 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&now,0,0,0x18,false);
    CHECK(f.stop_reason==LINE_STOP_ENCODER); /* stalled or disconnected encoder */
    now=UINT32_MAX-20U; start(&f,&c,&now,long_route,1);
    reliable=f.last_reliable_ms;
    for(unsigned i=0;i<100 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&now,1,1,0,false);
    CHECK(f.stop_reason==LINE_STOP_LOST && now-reliable==INTERFERENCE_TIMEOUT_MS);
    /* Recovery arriving at the deadline is rejected. */
    start(&f,&c,&now,long_route,1); reliable=f.last_reliable_ms;
    while(now-reliable<INTERFERENCE_TIMEOUT_MS-15U) tick(&f,&c,&now,1,1,0,false);
    for(unsigned i=0;i<3;++i) tick(&f,&c,&now,1,1,0x18,false);
    CHECK(f.stop_reason==LINE_STOP_LOST);
    /* A rapid forward displacement exhausts the distance cap before the time cap. */
    start(&f,&c,&now,long_route,1); reliable=f.last_reliable_ms;
    for(unsigned i=0;i<20 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&now,50,50,0,false);
    CHECK(f.stop_reason==LINE_STOP_LOST && now-reliable<INTERFERENCE_TIMEOUT_MS);
    /* STOP requires both odometry window and intersection, then callback once. */
    RouteStep stop[]={{1,.3f,STOP,8}};
    start(&f,&c,&now,stop,1);
    for(unsigned i=0;i<16;++i) tick(&f,&c,&now,5,5,0x18,false);
    CHECK(f.state==LINE_TRACKING && !f.route.stop_pending);
    for(unsigned i=0;i<4 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&now,5,5,0xff,false);
    CHECK(f.stop_reason==LINE_STOP_ROUTE_COMPLETE && callbacks==0);
    route_dispatch_stop_callback(&f.route); route_dispatch_stop_callback(&f.route);
    CHECK(callbacks==1);
    for(unsigned i=0;i<5;++i) tick(&f,&c,&now,0,0,0x18,false);
    CHECK(f.state==LINE_STOPPED && callbacks==1);
    /* Reaching/passing the STOP distance without a marker is a route fault, not success. */
    start(&f,&c,&now,stop,1);
    for(unsigned i=0;i<30 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&now,5,5,0x18,false);
    CHECK(f.stop_reason==LINE_STOP_ROUTE && !f.route.stop_pending);
}
static void test_full_route_pipeline(void)
{
    RouteStep steps[]={{1,.3f,LEFT,8},{1,.3f,STOP,8}};
    LineFollow f; EncoderCounts c; uint32_t now=0;
    start(&f,&c,&now,steps,2);
    for(unsigned i=0;i<16;++i) tick(&f,&c,&now,5,5,0x18,false);
    for(unsigned i=0;i<4 && f.route.state!=ROUTE_EXECUTING;++i)
        tick(&f,&c,&now,5,5,0xff,false);
    CHECK(f.state==LINE_JUNCTION && f.route.index==0);
    float angle=f.route.entry_heading;
    while(f.odometry.heading_rad-angle> -1.45f && f.state!=LINE_STOPPED)
        tick(&f,&c,&now,1,7,0,false);
    for(unsigned i=0;i<3;++i) tick(&f,&c,&now,1,7,0x18,false);
    CHECK(f.route.state==ROUTE_CLEARING);
    for(unsigned i=0;i<110 && f.route.index==0 && f.state!=LINE_STOPPED;++i)
        tick(&f,&c,&now,3,3,0x18,false);
    CHECK(f.route.index==1 && f.state==LINE_TRACKING);
    for(unsigned i=0;i<16;++i) tick(&f,&c,&now,5,5,0x18,false);
    for(unsigned i=0;i<4 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&now,5,5,0xff,false);
    CHECK(f.stop_reason==LINE_STOP_ROUTE_COMPLETE);
    route_dispatch_stop_callback(&f.route); CHECK(callbacks==2);
}
static void test_raw_pipeline(void)
{
    GraySensor sensor; LineFollow f; EncoderCounts c={0}; uint32_t now=0;
    gray_sensor_init(&sensor);
    line_follow_init(&f,calibration(),long_route,1,NULL,NULL);
    for(unsigned i=0;i<12;++i) {
        now+=5; c.left+=3; c.right+=3;
        GrayObservation obs=gray_sensor_update(&sensor,raw_for_line(0x18),
            line_follow_have_recent_position(&f,now),f.reliable_position);
        line_follow_update(&f,obs,c,now,.005f,i==11);
    }
    CHECK(f.state==LINE_TRACKING);
    for(unsigned i=0;i<5;++i) {
        now+=5; c.left+=3; c.right+=3;
        GrayObservation obs=gray_sensor_update(&sensor,raw_for_line(0x19),
            line_follow_have_recent_position(&f,now),f.reliable_position);
        line_follow_update(&f,obs,c,now,.005f,false);
    }
    CHECK(f.state==LINE_GAP_HOLD && f.route.index==0);
    for(unsigned i=0;i<5;++i) {
        now+=5; c.left+=3; c.right+=3;
        GrayObservation obs=gray_sensor_update(&sensor,raw_for_line(0x18),
            line_follow_have_recent_position(&f,now),f.reliable_position);
        line_follow_update(&f,obs,c,now,.005f,false);
    }
    CHECK(f.state==LINE_TRACKING && !f.abnormal);
}

static void test_startup_ramp(void)
{
    const int speeds[] = {12, 25};
    for (size_t s = 0; s < sizeof(speeds)/sizeof(speeds[0]); ++s) {
        RouteStep steps[] = {{1000, 2, STOP, speeds[s]}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,1);
        /* start() has already applied the first 5 ms control interval. */
        for (unsigned i=1; i<=200; ++i) {
            float expected=(float)speeds[s]*(float)i/200.0f;
            NEAR(f.base,expected);
            NEAR(f.left_target_cm_s,expected); NEAR(f.right_target_cm_s,expected);
            if (i<200) tick(&f,&c,&now,6,6,0x18,false);
        }
        tick(&f,&c,&now,6,6,0x18,false); NEAR(f.base,(float)speeds[s]);
        line_follow_stop(&f,LINE_STOP_REQUEST);
        for (unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i)
            tick(&f,&c,&now,0,0,0x18,i==LINE_CONFIRM_SAMPLES-1U);
        NEAR(f.base,(float)speeds[s]/200.0f); /* Explicit restart ramps again. */

        /* 控制间隔可合法地为 5/15/10 ms；斜坡按实际 dt，且不受毫秒回绕影响。 */
        now=UINT32_MAX-20U;
        start(&f,&c,&now,steps,1);
        const unsigned intervals[]={5,15,10};
        unsigned elapsed=5, n=0;
        while (elapsed<1000U) {
            unsigned ms=intervals[n++%3U];
            if (ms>1000U-elapsed) ms=1000U-elapsed;
            now+=ms; c.left=(uint16_t)(c.left+ms); c.right=(uint16_t)(c.right+ms);
            elapsed+=ms;
            line_follow_update(&f,mask(0x18),c,now,(float)ms/1000.0f,false);
            CHECK(f.state==LINE_TRACKING && f.command.enabled);
            NEAR(f.base,(float)speeds[s]*(float)elapsed/1000.0f);
        }
        NEAR(f.base,(float)speeds[s]);
    }
}

static void test_long_straight_without_timeout(void)
{
    RouteStep steps[]={{131,8,LEFT,12},{70,5,STOP,12}};
    LineFollow f; EncoderCounts c; uint32_t now=0;
    start(&f,&c,&now,steps,2);
    uint32_t began=now;
    /* Synthetic encoder feedback: 2 cm/s, >60 s to junction; no segment time limit. */
    while (f.odometry.distance_cm-f.route.origin_cm<130.0f && f.state!=LINE_STOPPED)
        tick(&f,&c,&now,1,1,0x18,false);
    CHECK(now-began>60000U);
    CHECK(f.state==LINE_TRACKING && f.route.failure==ROUTE_FAILURE_NONE);
    for (unsigned i=0;i<5 && f.route.state!=ROUTE_EXECUTING;++i)
        tick(&f,&c,&now,3,3,0x1f,false);
    CHECK(f.route.state==ROUTE_EXECUTING);

    RoutePlanner p;
    route_init(&p,steps,2,NULL,NULL); route_start(&p,0,UINT32_MAX-100U);
    uint32_t began_wrap=p.segment_ms;
    route_update(&p,mask(0x18),120,0,began_wrap+3600000U);
    CHECK(p.state==ROUTE_FOLLOW);
    route_update(&p,mask(0x18),120,0,began_wrap+7200000U);
    CHECK(p.state==ROUTE_FOLLOW && p.failure==ROUTE_FAILURE_NONE);
    confirm(&p,0xff,130,began_wrap+7200005U);
    CHECK(p.state==ROUTE_EXECUTING);
    route_start(&p,0,0); route_update(&p,mask(0x18),139.01f,0,5);
    CHECK(p.failure==ROUTE_FAILURE_WINDOW && !p.stop_pending);
    route_start(&p,0,0); confirm(&p,0xff,130,5);
    route_update(&p,mask(0x18),131,.5f,30);
    CHECK(p.failure==ROUTE_FAILURE_WRONG_WAY);
    route_start(&p,0,0); confirm(&p,0xff,130,5);
    route_update(&p,mask(0x18),131,-2.2f,30);
    CHECK(p.failure==ROUTE_FAILURE_TURN_LIMIT);
    route_start(&p,0,0); route_update(&p,mask(0xff),130,0,5);
    route_update(&p,mask(0xff),130,0,5+JUNCTION_CONFIRM_TIMEOUT_MS);
    CHECK(p.failure==ROUTE_FAILURE_CONFIRM_TIMEOUT);
    route_start(&p,0,0); confirm(&p,0xff,130,5);
    route_update(&p,mask(0x18),131,-1.5f,p.phase_ms+JUNCTION_ACTION_TIMEOUT_MS);
    CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
}

/* 真实配置轮距和计数比例下的理想运动学回归；灰度与轮速为合成输入，不是实车证据。 */
static void test_configured_path(void)
{
    CHECK(route_count==5U);
    const float distances[]={104,131,60,39,70};
    for (size_t i=0;i<route_count;++i) {
        NEAR(route[i].distance,distances[i]); CHECK(route[i].speed==25);
        CHECK(route[i].action==(i+1U==route_count ? STOP : LEFT));
    }
    for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[5];
        for (size_t i=0;i<route_count;++i) {
            steps[i]=route[i];
            if (mirror && steps[i].action==LEFT) steps[i].action=RIGHT;
        }
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        OdometryCalibration geometry={4.8f,4.8f,1560,1560,14.5f,-1,1,true};
        line_follow_init(&f,geometry,steps,route_count,NULL,NULL);
        for (unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i)
            tick(&f,&c,&now,0,0,0x18,i==LINE_CONFIRM_SAMPLES-1U);
        float residual_l=0,residual_r=0;
        bool saw_fast=false, saw_slow=false, saw_approach=false;
        unsigned completed_turns=0;
        uint32_t action_started=0;
        RouteState previous=f.route.state;
        for (unsigned i=0;i<16000 && f.state!=LINE_STOPPED;++i) {
            float cm_per_count=3.14159265359f*4.8f/1560.0f;
            residual_l+=f.left_target_cm_s*.005f/cm_per_count;
            residual_r+=f.right_target_cm_s*.005f/cm_per_count;
            int l=(int)residual_l, r=(int)residual_r;
            residual_l-=(float)l; residual_r-=(float)r;
            uint8_t line=0x18;
            RouteStep step=steps[f.route.index];
            if (f.route.state==ROUTE_FOLLOW || f.route.state==ROUTE_CANDIDATE) {
                float progress=f.odometry.distance_cm-f.route.origin_cm;
                if (progress>=step.distance) line=mirror ? 0xf8 : 0x1f;
                if (progress>=step.distance-step.tolerance) {
                    CHECK(f.base<=JUNCTION_TURN_SPEED_CM_S+.001f);
                    saw_approach=true;
                }
            } else if (f.route.state==ROUTE_EXECUTING) {
                float turn=fabsf(f.odometry.heading_rad-f.route.entry_heading);
                line=turn>=1.48f ? 0x18 : 0;
                if (mirror) NEAR(f.right_target_cm_s,0);
                else NEAR(f.left_target_cm_s,0);
                if (turn<JUNCTION_TURN_SLOW_RAD) {
                    NEAR(f.base,JUNCTION_TURN_SPEED_CM_S); saw_fast=true;
                } else {
                    CHECK(f.base<=JUNCTION_TURN_SLOW_CM_S+.001f); saw_slow=true;
                }
            }
            tick(&f,&c,&now,-l,r,line,false);
            if (previous!=ROUTE_EXECUTING && f.route.state==ROUTE_EXECUTING) action_started=now;
            if (previous==ROUTE_CLEARING && f.route.state==ROUTE_FOLLOW) {
                CHECK(now-action_started<JUNCTION_ACTION_TIMEOUT_MS);
                ++completed_turns;
            }
            previous=f.route.state;
        }
        CHECK(saw_fast && saw_slow && saw_approach && completed_turns==4);
        CHECK(f.stop_reason==LINE_STOP_ROUTE_COMPLETE && f.route.index==4);
        CHECK(f.route.failure==ROUTE_FAILURE_NONE && !f.command.enabled);
    }
}
int main(void)
{
    test_odometry(); test_speed(); test_left_encoder_direction(); test_routes(); test_controller(); test_full_route_pipeline(); test_raw_pipeline();
    test_startup_ramp(); test_long_straight_without_timeout(); test_configured_path();
    printf("PASS: odometry, nested loops, junction FSM, interference and route STOP; %u checks\n",checks);
    return 0;
}
