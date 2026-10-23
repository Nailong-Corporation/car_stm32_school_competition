#include "gray_test_input.h"
#include "line_follow.h"
#include "motion_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static unsigned checks;
#define CHECK(c) do { ++checks; assert(c); } while (0)
#define NEAR(a,b) CHECK(fabsf((a)-(b)) < 0.001f)
/* 旧回归显式保持单轮；循环用例使用生产初始化和实际配置。 */
static void init_route_once(RoutePlanner *p, const RouteStep *steps, size_t count,
                            RouteStopCallback callback, void *context)
{
    route_init(p, steps, count, callback, context);
    p->repeat_count = 1;
}
static void init_follow_once(LineFollow *f, OdometryCalibration c, const RouteStep *steps,
                             size_t count, RouteStopCallback callback, void *context)
{
    line_follow_init(f, c, steps, count, callback, context);
    f->route.repeat_count = 1;
}
static OdometryCalibration calibration(void)
{
    /* 合成几何在生产距离系数生效后仍为 0.01 cm/count，保持状态机测试的固定输入尺度。
     * 真实轮径/计数及实测比例另由 test_measured_distance_scale() 验证，不覆盖生产宏。 */
    return (OdometryCalibration){1.0f/(3.14159265359f*ODOMETRY_DISTANCE_SCALE),
                                 1.0f/(3.14159265359f*ODOMETRY_DISTANCE_SCALE),
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
    init_follow_once(f, calibration(), steps, count, stopped_callback, f);
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
/* 真实配置回放：旧版每米约 10345 个计数实际对应 153 cm；修正后约 6762 个计数为 1 m。
 * 原始计数按实际方向注入，既核对生产比例，也核对轮速/航向和左轮负向回绕。 */
static void test_measured_distance_scale(void)
{
    Odometry o;
    OdometryCalibration c = {LEFT_WHEEL_DIAMETER_CM, RIGHT_WHEEL_DIAMETER_CM,
        LEFT_COUNTS_PER_WHEEL_REV, RIGHT_COUNTS_PER_WHEEL_REV, WHEEL_TRACK_CM,
        ENCODER_LEFT_SIGN, ENCODER_RIGHT_SIGN, ODOMETRY_CALIBRATION_CONFIRMED != 0};
    EncoderCounts counts = {0};
    odometry_init(&o, c);
    CHECK(o.ready);
    CHECK(odometry_update(&o, counts, .005f));
    for (unsigned i=0; i<1127; ++i) {
        counts.left = (uint16_t)(counts.left + 6*ENCODER_LEFT_SIGN);
        counts.right = (uint16_t)(counts.right + 6*ENCODER_RIGHT_SIGN);
        CHECK(odometry_update(&o, counts, .005f));
    }
    CHECK(o.distance_cm >= 100.0f && o.distance_cm < 100.02f);
    NEAR(o.heading_rad, 0);
    CHECK(fabsf(o.left_speed_cm_s - 17.747582f) < .002f);
    CHECK(fabsf(o.right_speed_cm_s - 17.747582f) < .002f);
    /* 等速直行后左轮额外前进 10 个计数：校验右转航向也使用修正后的厘米尺度。 */
    counts.left = (uint16_t)(counts.left + 10*ENCODER_LEFT_SIGN);
    CHECK(odometry_update(&o, counts, .005f));
    CHECK(fabsf(o.heading_rad - .01019976f) < .00001f);
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
    CHECK(s.fault==SPEED_FAULT_STALL && s.fault_wheel==1);
    CHECK(!speed_control_update(&s,10,10,-2,10,.005f,&out));
    CHECK(s.fault==SPEED_FAULT_REVERSE && s.fault_wheel==1);
    CHECK(!speed_control_update(&s,NAN,10,0,0,.005f,&out));
    CHECK(s.fault==SPEED_FAULT_INPUT && s.fault_wheel==1);
    speed_control_reset(&s);
    ok=true;
    for(unsigned i=0;i<100 && ok;++i) ok=speed_control_update(&s,10,10,10,0,.005f,&out);
    CHECK(!ok && s.fault==SPEED_FAULT_STALL && s.fault_wheel==2 && !out.enabled);
}
static void test_left_encoder_direction(void)
{
    /* Replay the reported startup snapshot: left 0 -> 65527 (-9), right 0 -> 2.
     * Use the reported geometry; a sign correction must preserve reverse-feedback protection. */
    for (int sign = 1; sign >= -1; sign -= 2) {
        LineFollow f; EncoderCounts counts = {0}; uint32_t now = 0;
        OdometryCalibration c = {4.8f, 4.8f, 1560.0f, 1560.0f, 14.5f, sign, 1, true};
        init_follow_once(&f,c,long_route,1,NULL,NULL);
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
    init_route_once(&p,table,2,NULL,NULL); CHECK(p.valid);
    for(unsigned a=0;a<3;++a) {
        table[0].action=(Action)a;
        init_route_once(&p,table,2,NULL,NULL); route_start(&p,0,0);
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
    init_route_once(&p,table,2,NULL,NULL); route_start(&p,0,0);
    route_update(&p,mask(0x18),12.01f,0,50); CHECK(p.state==ROUTE_FAILED);
    init_route_once(&p,table,2,NULL,NULL); route_start(&p,0,0);
    for(unsigned i=0;i<3;++i) route_update(&p,mask(0xff),9,0,10+5*i);
    CHECK(p.state==ROUTE_CANDIDATE); /* stationary pattern cannot confirm */
    route_update(&p,mask(0xff),9,0,110); CHECK(p.state==ROUTE_FAILED);
    table[0].action=LEFT;
    init_route_once(&p,table,2,NULL,NULL); route_start(&p,0,0); confirm(&p,0xff,9,10);
    route_update(&p,mask(0x18),9.2f,+.5f,30); CHECK(p.state==ROUTE_FAILED); /* wrong way */
    init_route_once(&p,table,2,NULL,NULL); route_start(&p,0,0); confirm(&p,0xff,9,10);
    route_update(&p,mask(0xff),9.2f,0,20+JUNCTION_ACTION_TIMEOUT_MS);
    CHECK(p.state==ROUTE_FAILED);
    init_route_once(&p,table,2,NULL,NULL); route_start(&p,0,UINT32_MAX-10U);
    confirm(&p,0xff,9,UINT32_MAX-5U); CHECK(p.state==ROUTE_EXECUTING);
    table[0].speed=0; init_route_once(&p,table,2,NULL,NULL); CHECK(!p.valid);
    table[0].speed=8; table[1].action=LEFT; init_route_once(&p,table,2,NULL,NULL); CHECK(!p.valid);
    table[1].action=STOP; table[0].action=STOP; init_route_once(&p,table,2,NULL,NULL); CHECK(!p.valid);
    init_route_once(&p,NULL,0,NULL,NULL); CHECK(!p.valid);
}
static void test_controller(void)
{
    LineFollow f; EncoderCounts c={0}; uint32_t now=0;
    OdometryCalibration invalid=calibration(); invalid.confirmed=false;
    init_follow_once(&f,invalid,long_route,1,NULL,NULL);
    tick(&f,&c,&now,0,0,0x18,true);
    CHECK(f.stop_reason==LINE_STOP_CALIBRATION && !f.command.enabled);
    CHECK(f.start_result==LINE_START_REJECTED_CALIBRATION);
    init_follow_once(&f,calibration(),NULL,0,NULL,NULL);
    for(unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i)
        tick(&f,&c,&now,0,0,0x18,i==LINE_CONFIRM_SAMPLES-1U);
    CHECK(f.stop_reason==LINE_STOP_ROUTE && !f.command.enabled);
    CHECK(f.start_result==LINE_START_REJECTED_ROUTE);
    init_follow_once(&f,calibration(),long_route,1,NULL,NULL);
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
    CHECK(f.stop_snapshot.valid && f.stop_snapshot.reason==LINE_STOP_LOST);
    CHECK(f.stop_snapshot.reliable_age_ms==INTERFERENCE_TIMEOUT_MS);
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
    RouteStep steps[]={{1,.3f,LEFT,8},{30,.3f,STOP,8}};
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
    while(f.odometry.distance_cm-f.route.origin_cm<29.85f && f.state!=LINE_STOPPED)
        tick(&f,&c,&now,5,5,0x18,false);
    for(unsigned i=0;i<4 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&now,5,5,0xff,false);
    CHECK(f.stop_reason==LINE_STOP_ROUTE_COMPLETE);
    route_dispatch_stop_callback(&f.route); CHECK(callbacks==2);
}
/* 没有实车停车快照时，仅复现可能路径，不能把该合成场景认定为实车根因。 */
static void test_post_turn_stop_snapshot(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{1,.3f,mirror?RIGHT:LEFT,8},{100,5,STOP,40}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        while(f.odometry.distance_cm-f.route.origin_cm<.85f)
            tick(&f,&c,&now,5,5,0x18,false);
        for(unsigned i=0;i<5 && f.route.state!=ROUTE_EXECUTING;++i)
            tick(&f,&c,&now,5,5,0xff,false);
        CHECK(f.route.state==ROUTE_EXECUTING);
        float direction=mirror?1.0f:-1.0f;
        while(direction*(f.odometry.heading_rad-f.route.entry_heading)<1.0f)
            tick(&f,&c,&now,mirror?7:1,mirror?1:7,0,false);
        unsigned straight_frames=0;
        while(f.state!=LINE_STOPPED && straight_frames++<1000) {
            /* 已看到出口线、两轮开始等计数前进，但编码器转角仍不足 1.35 rad。
             * 外观可表现为沿线低速走一段；FSM 正确保留动作确认并在超时后停车。 */
            tick(&f,&c,&now,3,3,0x18,false);
            if(f.state!=LINE_STOPPED) {
                CHECK(f.route.state==ROUTE_EXECUTING && f.route.turn_capturing);
                CHECK(f.base<=JUNCTION_CAPTURE_SPEED_CM_S && f.command.enabled);
            }
        }
        CHECK(straight_frames>100 && f.stop_reason==LINE_STOP_ROUTE);
        LineStopSnapshot saved=f.stop_snapshot;
        CHECK(saved.valid && saved.reason==LINE_STOP_ROUTE);
        CHECK(saved.state_before_stop==LINE_JUNCTION);
        CHECK(saved.route_state_before_update==ROUTE_EXECUTING);
        CHECK(saved.route_state==ROUTE_FAILED && saved.route_failure==ROUTE_FAILURE_ACTION_TIMEOUT);
        CHECK(saved.route_phase_age_ms>=JUNCTION_ACTION_TIMEOUT_MS);
        CHECK(saved.observation.line_mask==0x18 && saved.observation.kind==GRAY_VALID);
        CHECK(fabsf(saved.turn_rad)<JUNCTION_TURN_MIN_RAD);
        CHECK(saved.left_target_cm_s>0 && saved.right_target_cm_s>0 && f.left_target_cm_s==0);
        for(unsigned i=0;i<20;++i) tick(&f,&c,&now,0,0,0,false);
        CHECK(f.stop_snapshot.sample_ms==saved.sample_ms);
        CHECK(f.stop_snapshot.observation.line_mask==0x18);
        CHECK(f.stop_snapshot.route_phase_age_ms==saved.route_phase_age_ms);
        line_follow_stop(&f,LINE_STOP_REQUEST);
        CHECK(f.stop_snapshot.reason==LINE_STOP_ROUTE);
        for(unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i)
            tick(&f,&c,&now,0,0,0x18,i==LINE_CONFIRM_SAMPLES-1U);
        CHECK(f.start_result==LINE_START_ACCEPTED && !f.stop_snapshot.valid);

        /* 编码器反馈停车快照必须在速度 PID 复位前保存具体轮侧和原因。 */
        start(&f,&c,&now,long_route,1);
        tick(&f,&c,&now,-6,6,0x18,false);
        CHECK(f.stop_reason==LINE_STOP_ENCODER && f.stop_snapshot.valid);
        CHECK(f.stop_snapshot.speed_fault==SPEED_FAULT_REVERSE && f.stop_snapshot.speed_fault_wheel==1);
        CHECK(f.stop_snapshot.left_speed_cm_s<0 && f.stop_snapshot.left_target_cm_s>0);
        saved=f.stop_snapshot;
        tick(&f,&c,&now,0,0,0x18,true);
        CHECK(f.start_result==LINE_START_REJECTED_FAULT && f.stop_snapshot.sample_ms==saved.sample_ms);
    }
}

static void test_raw_pipeline(void)
{
    GraySensor sensor; LineFollow f; EncoderCounts c={0}; uint32_t now=0;
    gray_sensor_init(&sensor);
    init_follow_once(&f,calibration(),long_route,1,NULL,NULL);
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
    const int speeds[] = {12, 25, 40};
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

/* 提前减速只影响基础目标；窗口外即使出现路口图形也不能触发转弯。 */
static void test_junction_approach_speed(void)
{
    const int speeds[]={4,12,40};
    for (unsigned mirror=0;mirror<2;++mirror) for (unsigned n=0;n<3;++n) {
        RouteStep steps[]={{100,8,mirror?RIGHT:LEFT,speeds[n]},{100,5,STOP,40}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        float previous=0;
        bool saw_deceleration=false, saw_window=false;
        while (f.odometry.distance_cm-f.route.origin_cm<99 && f.state!=LINE_STOPPED) {
            tick(&f,&c,&now,speeds[n]/2,speeds[n]/2,0x18,false);
            float progress=f.odometry.distance_cm-f.route.origin_cm;
            float remaining=steps[0].distance-steps[0].tolerance-progress;
            float slow=fminf((float)speeds[n],JUNCTION_APPROACH_SPEED_CM_S);
            float fraction=fmaxf(0,fminf(1,remaining/JUNCTION_APPROACH_DISTANCE_CM));
            float expected=sqrtf(slow*slow+((float)(speeds[n]*speeds[n])-slow*slow)*fraction);
            if (progress>50) {
                NEAR(f.base,expected);
                NEAR(f.left_target_cm_s,expected); NEAR(f.right_target_cm_s,expected);
                if (remaining>0 && remaining<JUNCTION_APPROACH_DISTANCE_CM && speeds[n]>JUNCTION_APPROACH_SPEED_CM_S) {
                    CHECK(f.base<previous && f.base>slow);
                    saw_deceleration=true;
                }
                if (remaining<=0) { NEAR(f.base,slow); saw_window=true; }
            }
            CHECK(f.state==LINE_TRACKING && f.route.state==ROUTE_FOLLOW);
            previous=f.base;
        }
        CHECK(saw_window && (speeds[n]<=JUNCTION_APPROACH_SPEED_CM_S || saw_deceleration));
        /* 最终 STOP 仍需路口确认；短路段从启动就受减速包络约束。 */
        RouteStep short_step[]={{10,2,STOP,40}};
        start(&f,&c,&now,short_step,1);
        for (unsigned i=0;i<25;++i) tick(&f,&c,&now,20,20,0x18,false);
        CHECK(f.route.state==ROUTE_FOLLOW && f.command.enabled);
        float before=f.base;
        tick(&f,&c,&now,20,20,0xff,false); /* 仍在窗口外，不能提前确认 STOP。 */
        CHECK(f.route.state==ROUTE_FOLLOW && !f.route.stop_pending);
        CHECK(f.base<=before && f.base<=HOLD_SPEED_CM_S);
    }
}

/* 真实走完左右转确认，再用中心线验证离口中即提速、下一段低速/短段包络及里程去重。 */
static void test_early_exit_acceleration(void)
{
    const int speeds[]={4,12,40};
    for (unsigned mirror=0;mirror<2;++mirror) for (unsigned n=0;n<3;++n)
    for (unsigned short_next=0;short_next<2;++short_next) {
        RouteStep steps[]={{100,8,mirror?RIGHT:LEFT,40},
                           {short_next?20:100,2,STOP,speeds[n]}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        for (unsigned i=0;i<200;++i) tick(&f,&c,&now,5,5,0x18,false);
        f.route.origin_cm=f.odometry.distance_cm-99.0f;
        for (unsigned i=0;i<4 && f.route.state!=ROUTE_EXECUTING;++i)
            tick(&f,&c,&now,5,5,0xff,false);
        CHECK(f.route.state==ROUTE_EXECUTING && f.route.index==0);
        float direction=mirror?1.0f:-1.0f;
        unsigned turn_frames=0;
        while (direction*(f.odometry.heading_rad-f.route.entry_heading)<1.45f &&
               f.state!=LINE_STOPPED && turn_frames++<400)
            tick(&f,&c,&now,mirror?7:1,mirror?1:7,0,false);
        CHECK(f.route.state==ROUTE_EXECUTING);
        float before_exit=f.base;
        for (unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) {
            before_exit=f.base;
            tick(&f,&c,&now,mirror?7:1,mirror?1:7,0x18,false);
        }
        CHECK(f.route.state==ROUTE_CLEARING && f.route.index==0);
        CHECK(f.startup_elapsed_s>=STARTUP_RAMP_TIME_S);
        float old_origin=f.route.origin_cm, clear_cm=f.route.clear_cm;
        float expected=(float)speeds[n];
        float remaining=18.0f-f.route.outgoing_cm;
        if (short_next && remaining<JUNCTION_APPROACH_DISTANCE_CM && expected>JUNCTION_APPROACH_SPEED_CM_S) {
            float slow=JUNCTION_APPROACH_SPEED_CM_S;
            expected=sqrtf(slow*slow+(expected*expected-slow*slow)*fmaxf(0.0f,remaining)/JUNCTION_APPROACH_DISTANCE_CM);
        }
        if (speeds[n]>6) CHECK(f.base>before_exit); /* 首个离口周期就上升，不等 3 cm。 */
        else NEAR(f.base,4.0f); /* 下一段低速不能沿用旧段 40 cm/s。 */
        uint32_t began=now;
        while (now-began<220U && f.route.state==ROUTE_CLEARING && f.state!=LINE_STOPPED) {
            float previous=f.base;
            tick(&f,&c,&now,5,5,0x18,false); /* 220 ms 只走 2.2 cm，仍处于去重区。 */
            CHECK(f.route.state==ROUTE_CLEARING && f.route.index==0);
            NEAR(f.route.origin_cm,old_origin);
            CHECK(f.base<=expected+.001f);
            CHECK(f.base>=previous-.001f);
            CHECK(f.base<=previous+TARGET_RISE_CM_S2*.005f+.001f);
        }
        CHECK(f.route.state==ROUTE_CLEARING && f.odometry.distance_cm-clear_cm<3.0f);
        NEAR(f.base,expected); /* 6→40 在 220 ms 内完成目标规划。 */
        for (unsigned i=0;i<30 && f.route.index==0 && f.state!=LINE_STOPPED;++i)
            tick(&f,&c,&now,5,5,0x18,false);
        CHECK(f.route.state==ROUTE_FOLLOW && f.route.index==1);
        CHECK(f.route.origin_cm<clear_cm);
        NEAR(f.odometry.distance_cm-f.route.origin_cm,f.route.outgoing_cm);
        NEAR(f.base,expected); /* 切段不再重置到 6 cm/s。 */
        tick(&f,&c,&now,5,5,0,false);
        CHECK(f.state==LINE_GAP_HOLD && f.base<=HOLD_SPEED_CM_S);
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
    init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,UINT32_MAX-100U);
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
        NEAR(route[i].distance,distances[i]); CHECK(route[i].speed==40);
        CHECK(i+1U==route_count ? route[i].action==STOP :
              (route[i].action==LEFT || route[i].action==RIGHT));
    }
    for (unsigned mirror=0;mirror<2;++mirror) for (uint32_t limit=0;limit<=2;++limit) {
        RouteStep steps[5];
        for (size_t i=0;i<route_count;++i) {
            steps[i]=route[i];
            if (mirror) {
                if (steps[i].action==LEFT) steps[i].action=RIGHT;
                else if (steps[i].action==RIGHT) steps[i].action=LEFT;
            }
        }
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        OdometryCalibration geometry={4.8f,4.8f,1560,1560,14.5f,-1,1,true};
        init_follow_once(&f,geometry,steps,route_count,NULL,NULL);
        for (unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i)
            tick(&f,&c,&now,0,0,0x18,i==LINE_CONFIRM_SAMPLES-1U);
        f.route.repeat_count=limit;
        unsigned expected_laps=limit ? limit : 2;
        float residual_l=0,residual_r=0;
        bool saw_fast=false, saw_slow=false, saw_approach=false, saw_reacceleration=false;
        unsigned completed_turns=0;
        uint32_t action_started=0;
        RouteState previous=f.route.state;
        for (unsigned i=0;i<16000 && f.state!=LINE_STOPPED &&
             !(f.route.completed_laps>=expected_laps && f.route.state==ROUTE_FOLLOW);++i) {
            float cm_per_count=3.14159265359f*4.8f/1560.0f*ODOMETRY_DISTANCE_SCALE;
            residual_l+=f.left_target_cm_s*.005f/cm_per_count;
            residual_r+=f.right_target_cm_s*.005f/cm_per_count;
            int l=(int)residual_l, r=(int)residual_r;
            residual_l-=(float)l; residual_r-=(float)r;
            uint8_t line=0x18;
            RouteStep step=steps[f.route.index];
            if (f.route.state==ROUTE_FOLLOW || f.route.state==ROUTE_CANDIDATE) {
                float progress=f.odometry.distance_cm-f.route.origin_cm;
                if (progress>=step.distance) line=step.action==RIGHT ? 0xf8 : 0x1f;
                if (progress>=step.distance-step.tolerance) {
                    CHECK(f.base<=JUNCTION_APPROACH_SPEED_CM_S+.001f);
                    saw_approach=true;
                }
            } else if (f.route.state==ROUTE_EXECUTING && route_current_action(&f.route)==STRAIGHT) {
                line=0x18;
                CHECK(fabsf(f.left_target_cm_s-f.right_target_cm_s)<.1f);
            } else if (f.route.state==ROUTE_EXECUTING) {
                float turn=fabsf(f.odometry.heading_rad-f.route.entry_heading);
                line=turn>=1.48f ? 0x18 : 0;
                if (!f.route.turn_capturing) {
                    if (step.action==RIGHT) NEAR(f.right_target_cm_s,0);
                    else NEAR(f.left_target_cm_s,0);
                }
                if (f.route.turn_capturing) {
                    CHECK(f.base<=JUNCTION_CAPTURE_SPEED_CM_S+.001f);
                } else if (turn<JUNCTION_TURN_SLOW_RAD) {
                    NEAR(f.base,JUNCTION_TURN_SPEED_CM_S); saw_fast=true;
                } else {
                    CHECK(f.base<=JUNCTION_TURN_SLOW_CM_S+.001f); saw_slow=true;
                }
            }
            float previous_base=f.base;
            size_t previous_index=f.route.index;
            tick(&f,&c,&now,-l,r,line,false);
            if (f.route.state==ROUTE_CLEARING && f.base>JUNCTION_TURN_SPEED_CM_S+.001f)
                saw_reacceleration=true; /* 确认转弯后，去重完成前就开始恢复下一段速度。 */
            if (previous!=ROUTE_EXECUTING && f.route.state==ROUTE_EXECUTING) action_started=now;
            if (previous==ROUTE_CLEARING && f.route.state==ROUTE_FOLLOW) {
                CHECK(now-action_started<JUNCTION_ACTION_TIMEOUT_MS);
                CHECK(f.base>=previous_base-.001f);
                CHECK(f.base<=previous_base+TARGET_RISE_CM_S2*.005f+.001f);
                if (previous_index+1U<route_count) ++completed_turns;
            }
            previous=f.route.state;
        }
        CHECK(saw_fast && saw_slow && saw_approach && saw_reacceleration && completed_turns==4U*expected_laps);
        CHECK(f.route.completed_laps==expected_laps && f.route.failure==ROUTE_FAILURE_NONE);
        if (limit) {
            CHECK(f.stop_reason==LINE_STOP_ROUTE_COMPLETE && f.route.index==4);
            CHECK(!f.command.enabled);
        } else {
            CHECK(f.route.index==0 && f.command.enabled && !f.route.stop_pending);
            line_follow_stop(&f,LINE_STOP_REQUEST);
            CHECK(!f.command.enabled);
        }
    }
}
/* 一轮在末项标记确认时计数；中间轮完整直行/离口后才重建首段。 */
/* 两个方向的恒曲率 90° 圆弧：半径 10 cm 的出口位移为 10，非弧长 15.708。 */
/* 实车快照：1.3465 rad、出口 position=+1，修正抵消后一直直走至超时。
 * 两方向均须继续完成最小转角，随后连续中心线才退出，不放宽几何门禁。 */
/* 实测目标轮差存在而实际转角停在 1.3474 rad；出口证据可有界结束动作。 */
/* 实车第四次快照：EXECUTING 已成功转 CLEARING，只有 0.288 cm 时共享预算耗尽。
 * 晚完成动作仍有独立离口时间，但两个阶段各自的时限不能被反复刷新。 */
static void test_late_turn_clear_budget(void)
{
    const RouteStep steps[]={{100,16,LEFT,40},{131,16,STOP,40}};
    RoutePlanner p;
    init_route_once(&p,steps,2,NULL,NULL);
    route_start(&p,0,0);
    p.state=ROUTE_EXECUTING;
    p.entry_cm=p.projection_previous_cm=100;
    for(unsigned i=0;i<3;++i)
        route_update(&p,mask(0x18),110,-1.5f,4390+i*5);
    CHECK(p.state==ROUTE_CLEARING && p.phase_ms==4400);
    for(unsigned i=0;i<3;++i)
        route_update(&p,mask(0x18),110.288f,-1.5f,4500+i*5);
    CHECK(p.state==ROUTE_CLEARING && p.phase_ms==4400);
    route_update(&p,mask(0x18),113.1f,-1.5f,4600);
    CHECK(p.state==ROUTE_FOLLOW && p.index==1);
    /* 同样晚入离口但未离开足够距离，独立时限仍停车。 */
    route_start(&p,0,0);
    p.state=ROUTE_EXECUTING;
    p.entry_cm=p.projection_previous_cm=100;
    for(unsigned i=0;i<3;++i)
        route_update(&p,mask(0x18),110,-1.5f,4390+i*5);
    route_update(&p,mask(0x18),110.288f,-1.5f,4400+JUNCTION_CLEAR_TIMEOUT_MS);
    CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
}

static void test_near_angle_exit(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{100,16,mirror?RIGHT:LEFT,40},{131,16,STOP,40}};
        for(unsigned scenario=0;scenario<6;++scenario) {
            RoutePlanner p;
            init_route_once(&p,steps,2,NULL,NULL);
            route_start(&p,0,0);
            p.state=ROUTE_EXECUTING;
            p.entry_cm=p.projection_previous_cm=100;
            float heading=(mirror?1.0f:-1.0f)*(scenario==1?1.29f:1.3474f);
            p.projection_previous_heading=heading;
            /* 连续中心线、两路中心偏差=2，重现板上 0x30。 */
            for(unsigned i=0;i<28;++i) {
                float distance=100+(scenario==2?0:.03f*(float)i);
                uint8_t line=scenario==3?0xff:(mirror?0x0c:0x30);
                if(scenario==5) {
                    /* 100 ms 内位置 2/3 往返，最后三帧居中。旧实现持续清空证据。 */
                    line=i<24 && i%2 ? (mirror?0x04:0x20) : (mirror?0x0c:0x30);
                }
                if(scenario==4 && i%10==9) line=0; /* 孤立/中断证据不得累积。 */
                route_update(&p,mask(line),distance,heading,200+i*5);
                if (i<20) CHECK(p.state==ROUTE_EXECUTING);
            }
            if(scenario==0 || scenario==5) CHECK(p.state==ROUTE_CLEARING);
            else CHECK(p.state==ROUTE_EXECUTING);
        }
    }
}

static void test_capture_cancellation(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{100,16,mirror?RIGHT:LEFT,40},{131,16,STOP,40}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        float direction=mirror?1.0f:-1.0f;
        f.route.state=ROUTE_EXECUTING;
        f.route.phase_ms=now;
        f.route.entry_heading=0;
        f.route.entry_cm=f.route.projection_previous_cm=f.odometry.distance_cm;
        f.odometry.heading_rad=direction*1.3465f;
        f.route.projection_previous_heading=f.odometry.heading_rad;
        f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
        f.base=6;
        /* 强制稳态位置 PID：与航向项相反的线位置，覆盖抵消/反向修正。 */
        for(unsigned i=0;i<20;++i) {
            tick(&f,&c,&now,3,3,mirror?0x08:0x10,false);
            CHECK(f.route.state==ROUTE_EXECUTING);
            CHECK(direction*(f.left_target_cm_s-f.right_target_cm_s)>=
                  2.0f*JUNCTION_CAPTURE_MIN_TURN_CM_S-.001f);
        }
        for(unsigned i=0;i<40 && f.route.state==ROUTE_EXECUTING;++i) {
            int l=(int)lroundf(f.left_target_cm_s*.5f);
            int r=(int)lroundf(f.right_target_cm_s*.5f);
            tick(&f,&c,&now,l,r,0x18,false);
        }
        CHECK(f.route.state==ROUTE_CLEARING && f.state!=LINE_STOPPED);
        CHECK(direction*(f.odometry.heading_rad-f.route.entry_heading)>=JUNCTION_TURN_MIN_RAD);
    }
}

static void test_corner_distance_projection(void)
{
    for (unsigned mirror=0; mirror<2; ++mirror) {
        RouteStep steps[]={{100,16,mirror?RIGHT:LEFT,40},{131,16,STOP,40}};
        RoutePlanner p;
        init_route_once(&p,steps,2,NULL,NULL);
        route_start(&p,0,0);
        p.state=ROUTE_EXECUTING;
        p.entry_cm=p.projection_previous_cm=100.0f;
        float sign=mirror?1.0f:-1.0f;
        float arc=10.0f*JUNCTION_TURN_TARGET_RAD;
        for (unsigned i=1;i<=20;++i)
            route_update(&p,mask(0),100.0f+arc*(float)i/20.0f,
                         sign*JUNCTION_TURN_TARGET_RAD*(float)i/20.0f,i*5);
        NEAR(p.outgoing_cm,10.0f);
        for (unsigned i=0;i<3;++i)
            route_update(&p,mask(0x18),100.0f+arc,sign*JUNCTION_TURN_TARGET_RAD,110+i*5);
        CHECK(p.state==ROUTE_CLEARING);
        for (unsigned i=0;i<3;++i)
            route_update(&p,mask(0x18),103.1f+arc,sign*JUNCTION_TURN_TARGET_RAD,130+i*5);
        CHECK(p.state==ROUTE_FOLLOW && p.index==1);
        NEAR(103.1f+arc-p.origin_cm,13.1f);
    }
}

static void test_route_repeats(void)
{
    const RouteStep stop[]={{10,2,STOP,8}};
    RoutePlanner p;
    route_init(&p,stop,1,NULL,NULL);
    CHECK(p.repeat_count == ROUTE_REPEAT_COUNT);
    for (uint32_t limit=0;limit<=3;++limit) {
        p.repeat_count=limit;
        route_start(&p,0,0);
        uint32_t now=10;
        unsigned laps=limit ? limit : 8;
        for (unsigned lap=1;lap<=laps;++lap) {
            float origin=p.origin_cm;
            confirm(&p,0xff,origin+9,now);
            CHECK(p.completed_laps==lap);
            if (limit && lap==limit) {
                CHECK(p.state==ROUTE_FINISHED && p.stop_pending);
                break;
            }
            CHECK(p.state==ROUTE_EXECUTING && p.lap_boundary && !p.stop_pending);
            CHECK(route_current_action(&p)==STRAIGHT);
            route_update(&p,mask(0xff),origin+12.3f,0,now+20);
            CHECK(p.state==ROUTE_EXECUTING && p.completed_laps==lap);
            for (unsigned i=0;i<3;++i)
                route_update(&p,mask(0x18),origin+12.4f,0,now+25+5*i);
            CHECK(p.state==ROUTE_CLEARING);
            for (unsigned i=0;i<3;++i)
                route_update(&p,mask(0x18),origin+15.5f,0,now+40+5*i);
            CHECK(p.state==ROUTE_FOLLOW && p.index==0 && !p.lap_boundary);
            NEAR(p.origin_cm,p.entry_cm);
            confirm(&p,0xff,p.origin_cm+.1f,now+55);
            CHECK(p.state==ROUTE_FOLLOW && p.completed_laps==lap);
            now+=100;
        }
        route_start(&p,100,now);
        CHECK(p.index==0 && p.completed_laps==0 && !p.stop_pending && !p.lap_boundary);
        NEAR(p.origin_cm,100);
    }
    /* 无限模式计数饱和，不能回绕成 0 或被误判为最终停车。 */
    p.repeat_count=0; route_start(&p,0,0); p.completed_laps=UINT32_MAX;
    confirm(&p,0xff,9,10);
    CHECK(p.completed_laps==UINT32_MAX && p.state==ROUTE_EXECUTING);
    route_update(&p,mask(0x18),13,0,p.phase_ms+JUNCTION_ACTION_TIMEOUT_MS);
    CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT && !p.stop_pending);

    /* 生产控制链中，前几轮不能停车或发成功回调，末轮才清空电机并分派一次。 */
    RouteStep short_stop[]={{10,.3f,STOP,8}};
    for (uint32_t limit=0;limit<=2;++limit) {
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        unsigned before=callbacks;
        start(&f,&c,&now,short_stop,1); f.route.repeat_count=limit;
        for (unsigned lap=1;lap<=2;++lap) {
            while (f.odometry.distance_cm-f.route.origin_cm<9.85f && f.state!=LINE_STOPPED)
                tick(&f,&c,&now,5,5,0x18,false);
            for (unsigned i=0;i<5 && f.route.state!=ROUTE_EXECUTING && f.state!=LINE_STOPPED;++i)
                tick(&f,&c,&now,5,5,0xff,false);
            if (limit && lap==limit) {
                CHECK(f.stop_reason==LINE_STOP_ROUTE_COMPLETE && f.route.completed_laps==lap);
                route_dispatch_stop_callback(&f.route); route_dispatch_stop_callback(&f.route);
                CHECK(callbacks==before+1U);
                break;
            }
            CHECK(f.state==LINE_JUNCTION && f.route.lap_boundary && f.command.enabled);
            route_dispatch_stop_callback(&f.route); CHECK(callbacks==before);
            for (unsigned i=0;i<160 && f.route.state!=ROUTE_FOLLOW && f.state!=LINE_STOPPED;++i)
                tick(&f,&c,&now,5,5,0x18,false);
            CHECK(f.state==LINE_TRACKING && f.route.index==0 && f.route.completed_laps==lap);
        }
        if (!limit) {
            line_follow_stop(&f,LINE_STOP_REQUEST);
            CHECK(!f.command.enabled && !f.route.stop_pending && callbacks==before);
        }
    }
}

static void test_turn_capture(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{1,.3f,mirror?RIGHT:LEFT,8},{10,2,STOP,8}};
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        while(f.odometry.distance_cm-f.route.origin_cm<.85f)
            tick(&f,&c,&now,5,5,0x18,false);
        for(unsigned i=0;i<5 && f.route.state!=ROUTE_EXECUTING;++i)
            tick(&f,&c,&now,5,5,mirror?0xf8:0x1f,false);
        CHECK(f.route.state==ROUTE_EXECUTING);
        float direction=mirror?1.0f:-1.0f;
        while(direction*(f.odometry.heading_rad-f.route.entry_heading)<1.0f)
            tick(&f,&c,&now,mirror?6:0,mirror?0:6,0,false);
        /* 出口在转向外侧出现；旧代码仍给内轮零，无法用位置反馈修正。 */
        tick(&f,&c,&now,mirror?6:0,mirror?0:6,mirror?0x02:0x40,false);
        CHECK(f.route.turn_capturing && f.pid.primed && f.route.state==ROUTE_EXECUTING);
        CHECK(f.left_target_cm_s>0 && f.right_target_cm_s>0);
        CHECK(f.base<=JUNCTION_CAPTURE_SPEED_CM_S);
        tick(&f,&c,&now,3,3,0xff,false);
        CHECK(!f.route.turn_capturing && !f.pid.primed && f.route.state==ROUTE_EXECUTING);
        tick(&f,&c,&now,3,3,0x42,false);
        CHECK(!f.route.turn_capturing && !f.pid.primed);
        while(direction*(f.odometry.heading_rad-f.route.entry_heading)<1.46f)
            tick(&f,&c,&now,mirror?6:0,mirror?0:6,0,false);
        for (unsigned i=0;i<3;++i) tick(&f,&c,&now,3,3,0x18,false);
        CHECK(f.route.state==ROUTE_CLEARING && !f.route.turn_capturing);
    }
}

static void test_l_corner_confirmation(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{10,2,mirror?RIGHT:LEFT,8},{10,2,STOP,8}};
        uint8_t edge=mirror?0xc0:0x03, short_corner=mirror?0x70:0x0e;
        RoutePlanner p;
        init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0);
        confirm(&p,edge,9,10);
        CHECK(p.state==ROUTE_FOLLOW); /* 没有中心线历史的孤立侧边线不能触发。 */
        confirm(&p,short_corner,9.2f,30);
        CHECK(p.state==ROUTE_EXECUTING); /* 三路中心+分支的 L 角也能确认。 */
        init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0);
        confirm(&p,0x18,9,10);
        CHECK(p.have_approach_center);
        confirm(&p,mirror?0x80:0x01,9.2f,30);
        CHECK(p.state==ROUTE_FOLLOW); /* 即使有中心历史，单点噪声也不能触发。 */
        confirm(&p,edge,9.4f,50);
        CHECK(p.state==ROUTE_EXECUTING); /* 中心线随后变成两路边缘分支。 */
        init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0);
        route_update(&p,mask(0x18),9,0,10);
        confirm(&p,edge,9.2f,30); CHECK(p.state==ROUTE_FOLLOW);
        confirm(&p,0x18,9.3f,50);
        confirm(&p,mirror?0x03:0xc0,9.5f,70);
        CHECK(p.state==ROUTE_EXECUTING && p.turn_forced); /* 骤变优先，方向取表而非分支。 */
        init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0);
        confirm(&p,0x18,9.3f,50);
        confirm(&p,edge,9.6f,50+JUNCTION_CENTER_MEMORY_MS+20);
        CHECK(p.state==ROUTE_FOLLOW); /* 旧中心线过期。 */
        confirm(&p,0x18,9.3f,400);
        confirm(&p,edge,11.1f,420); CHECK(p.state==ROUTE_EXECUTING && p.turn_forced);
        init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0);
        confirm(&p,0x18,11.2f,450);
        confirm(&p,edge,12.1f,470); CHECK(p.failure==ROUTE_FAILURE_WINDOW);

        /* 报告的边缘停车场景：中心 → 两路侧边线应进动作，不能等到丢线停车。 */
        RouteStep short_steps[]={{1,.3f,mirror?RIGHT:LEFT,8},{10,2,STOP,8}};
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        start(&f,&c,&now,short_steps,2);
        while(f.odometry.distance_cm-f.route.origin_cm<.9f)
            tick(&f,&c,&now,5,5,0x18,false);
        for(unsigned j=0;j<5 && f.route.state!=ROUTE_EXECUTING;++j)
            tick(&f,&c,&now,5,5,edge,false);
        CHECK(f.state==LINE_JUNCTION && f.route.state==ROUTE_EXECUTING && f.command.enabled);
        CHECK(mirror ? f.left_target_cm_s>f.right_target_cm_s :
                       f.right_target_cm_s>f.left_target_cm_s);
    }
    for (unsigned action=STRAIGHT;action<=STOP;++action) {
        RouteStep steps[]={{10,2,(Action)action,8},{10,2,STOP,8}};
        RoutePlanner p;
        init_route_once(&p,steps,action==STOP?1:2,NULL,NULL); route_start(&p,0,0);
        confirm(&p,0x18,9,10); confirm(&p,0x03,9.2f,30);
        CHECK(p.state==ROUTE_FOLLOW && !p.stop_pending && p.completed_laps==0);
        confirm(&p,0x0e,9.4f,50);
        CHECK(p.state==ROUTE_FOLLOW && !p.stop_pending);
    }
}

static void test_route_abrupt_priority(void)
{
    /* 丢线、全目标、多区域、仍有效的宽窄线以及反侧线：同周期按表强制转向。 */
    const uint8_t shapes[] = {0x00,0xff,0x42,0x1e,0x0f,0x03,0xc0};
    for (unsigned mirror=0;mirror<2;++mirror) for (unsigned k=0;k<sizeof(shapes);++k) {
        RouteStep steps[]={{1,.3f,mirror?RIGHT:LEFT,8},{10,2,STOP,8}};
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        while(f.odometry.distance_cm-f.route.origin_cm<.85f)
            tick(&f,&c,&now,5,5,0x18,false);
        tick(&f,&c,&now,5,5,shapes[k],false);
        CHECK(f.route.state==ROUTE_EXECUTING && f.route.turn_forced && f.state==LINE_JUNCTION);
        CHECK(!f.abnormal && !f.pid.primed && f.route.candidate_count==0);
        CHECK(route_current_action(&f.route)==(mirror?RIGHT:LEFT));
        CHECK(mirror ? f.left_target_cm_s>0 && f.right_target_cm_s==0 :
                       f.right_target_cm_s>0 && f.left_target_cm_s==0);
        /* 入口即使仍在线上也不能撤销动作；早到中心线不能提前退出。 */
        for (unsigned j=0;j<5;++j) tick(&f,&c,&now,3,3,0x18,false);
        CHECK(f.route.state==ROUTE_EXECUTING && !f.route.turn_capturing && !f.pid.primed);
        /* 已提交动作的超时不能被连续在线帧续期。 */
        route_update(&f.route,mask(0x18),f.odometry.distance_cm,f.odometry.heading_rad,
                     f.route.phase_ms+JUNCTION_ACTION_TIMEOUT_MS);
        CHECK(f.route.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
        route_start(&f.route,0,now);
        CHECK(!f.route.turn_forced && f.route.narrow_count==0);
    }
    RouteStep steps[]={{10,2,LEFT,8},{10,2,STOP,8}};
    RoutePlanner p;
    /* 平滑曲线逐步移动两权重，不走骤变强制分支；原有形状候选规则仍独立生效。 */
    init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0);
    confirm(&p,0x18,8.5f,10);
    const uint8_t curve[]={0x30,0x60,0xc0,0x60,0x30,0x18,0x0c,0x06,0x0c,0x18};
    for(unsigned i=0;i<sizeof(curve);++i) {
        route_update(&p,mask(curve[i]),8.7f+.05f*(float)i,0,25+5*i);
        CHECK((p.state==ROUTE_FOLLOW || p.state==ROUTE_CANDIDATE) && !p.turn_forced);
    }
    /* 窗口前骤变、无稳定前史、过期前史、越过窗口，均不能强制执行。 */
    init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0);
    confirm(&p,0x18,7,10); route_update(&p,mask(0),7.2f,0,25);
    CHECK(p.state==ROUTE_FOLLOW && !p.turn_forced);
    route_update(&p,mask(0),8.2f,0,30); CHECK(p.state==ROUTE_FOLLOW);
    confirm(&p,0xff,9,35); CHECK(p.state==ROUTE_EXECUTING && !p.turn_forced);
    init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0);
    route_update(&p,mask(0x18),9,0,5); route_update(&p,mask(0),9.05f,0,10);
    CHECK(p.state==ROUTE_FOLLOW && !p.turn_forced);
    confirm(&p,0x18,9.1f,15);
    route_update(&p,mask(0),9.3f,0,25+CONTROL_MAX_INTERVAL_MS+1);
    CHECK(p.state==ROUTE_FOLLOW && !p.turn_forced);
    confirm(&p,0x18,11.8f,50); route_update(&p,mask(0),12.01f,0,65);
    CHECK(p.failure==ROUTE_FAILURE_WINDOW && !p.turn_forced);
    /* 单边噪点不触发；时基回绕后有效相邻骤变可触发。 */
    init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,UINT32_MAX-20);
    confirm(&p,0x18,9,UINT32_MAX-15);
    route_update(&p,mask(0x01),9.2f,0,0); CHECK(p.state==ROUTE_FOLLOW);
    confirm(&p,0x18,9.3f,5); route_update(&p,mask(0),9.5f,0,20);
    CHECK(p.state==ROUTE_EXECUTING && p.turn_forced);
    route_update(&p,mask(0),9.6f,.5f,25);
    CHECK(p.failure==ROUTE_FAILURE_WRONG_WAY);
    init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,UINT32_MAX-20);
    confirm(&p,0x18,9,UINT32_MAX-10);
    route_update(&p,mask(0),9.2f,0,4);
    CHECK(p.state==ROUTE_EXECUTING && p.turn_forced);
    route_update(&p,mask(0),9.3f,-2.2f,9);
    CHECK(p.failure==ROUTE_FAILURE_TURN_LIMIT);
    for (unsigned action=STRAIGHT;action<=STOP;++action) {
        steps[0].action=(Action)action;
        init_route_once(&p,steps,action==STOP?1:2,NULL,NULL); route_start(&p,0,0);
        confirm(&p,0x18,9,5); route_update(&p,mask(0),9.2f,0,20);
        CHECK(p.state==ROUTE_FOLLOW && !p.turn_forced && !p.stop_pending);
    }
    /* 使用生产极性/多数滤波：孤立原始全丢线被滤除，持续骤变在滤波输出变化当帧触发。 */
    steps[0].action=LEFT;
    LineFollow f; EncoderCounts c={0}; uint32_t now=0;
    start(&f,&c,&now,steps,2);
    GraySensor sensor; gray_sensor_init(&sensor);
    for (unsigned i=0;i<5;++i) {
        now+=5; c.left+=5; c.right+=5;
        GrayObservation obs=gray_sensor_update(&sensor,raw_for_line(0x18),true,0);
        line_follow_update(&f,obs,c,now,.005f,false);
    }
    f.route.origin_cm=f.odometry.distance_cm-9;
    const uint8_t samples[]={0,0x18,0x18,0,0};
    for (unsigned i=0;i<sizeof(samples);++i) {
        now+=5; c.left+=5; c.right+=5;
        GrayObservation obs=gray_sensor_update(&sensor,raw_for_line(samples[i]),true,0);
        line_follow_update(&f,obs,c,now,.005f,false);
        CHECK(i+1U==sizeof(samples) ? f.route.turn_forced && f.state==LINE_JUNCTION :
                                    f.route.state==ROUTE_FOLLOW && !f.route.turn_forced);
    }
}

/* 探头输入由二维 L 形线与传感器安装位置计算，不根据控制器转角直接指定中心线。
 * 轮速仍假设理想跟随目标，没有物理摩擦、惯性或打滑，不能替代实车。 */
static void test_spatial_corner(void)
{
    const float offsets[]={6,9,12};
    for (unsigned mirror=0;mirror<2;++mirror) for (unsigned k=0;k<3;++k) {
        float front=offsets[k];
        RouteStep steps[]={{40-front,3,mirror?RIGHT:LEFT,40},{100,3,STOP,40}};
        OdometryCalibration geometry={4.8f,4.8f,1560,1560,14.5f,-1,1,true};
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        init_follow_once(&f,geometry,steps,2,NULL,NULL);
        GraySensor sensor; gray_sensor_init(&sensor);
        float x=0,y=0,h=0,rl=0,rr=0;
        bool captured=false;
        for (unsigned n=0;n<4000 && f.state!=LINE_STOPPED && f.route.index==0;++n) {
            float dl=f.left_target_cm_s*.005f, dr=f.right_target_cm_s*.005f;
            float dh=(dl-dr)/geometry.track_cm, ds=(dl+dr)*.5f;
            x+=ds*cosf(h+dh*.5f); y+=ds*sinf(h+dh*.5f); h+=dh;
            float cm_per_count=3.14159265359f*4.8f/1560.0f*ODOMETRY_DISTANCE_SCALE;
            rl+=dl/cm_per_count; rr+=dr/cm_per_count;
            int l=(int)rl,r=(int)rr; rl-=(float)l; rr-=(float)r;
            c.left=(uint16_t)(c.left-l); c.right=(uint16_t)(c.right+r);
            uint8_t line=0;
            for(unsigned j=0;j<8;++j) {
                float side=((float)j-3.5f)*.6f;
                float sx=x+front*cosf(h)-side*sinf(h);
                float sy=y+front*sinf(h)+side*cosf(h);
                bool inbound=sx<=40.9f && fabsf(sy)<=.9f;
                bool outbound=fabsf(sx-40)<=.9f && (mirror ? sy>=-.9f : sy<=.9f);
                if(inbound || outbound) line|=(uint8_t)(1U<<j);
            }
            GrayObservation obs=gray_sensor_update(&sensor,raw_for_line(line),
                line_follow_have_recent_position(&f,now),f.reliable_position);
            now+=5;
            line_follow_update(&f,obs,c,now,.005f,n==LINE_CONFIRM_SAMPLES-1U);
            captured=captured || f.route.turn_capturing;

        }
        if(f.route.index!=1) fprintf(stderr,"spatial failure mirror=%u front=%.1f x=%.2f y=%.2f h=%.2f state=%d failure=%d stop=%d\n",
            mirror,front,x,y,h,f.route.state,f.route.failure,f.stop_reason);
        CHECK(f.route.index==1 && f.state==LINE_TRACKING && captured);
        CHECK(fabsf(x+front*cosf(h)-40)<1.5f);
        CHECK(fabsf(fabsf(h)-JUNCTION_TURN_TARGET_RAD)<.3f);
    }
}

int main(void)
{
    test_odometry(); test_measured_distance_scale(); test_speed(); test_left_encoder_direction(); test_routes(); test_controller(); test_full_route_pipeline(); test_post_turn_stop_snapshot(); test_raw_pipeline();
    test_startup_ramp(); test_junction_approach_speed(); test_early_exit_acceleration(); test_long_straight_without_timeout(); test_configured_path(); test_route_repeats(); test_corner_distance_projection(); test_capture_cancellation(); test_near_angle_exit(); test_late_turn_clear_budget(); test_turn_capture(); test_l_corner_confirmation(); test_route_abrupt_priority(); test_spatial_corner();
    printf("PASS: odometry, nested loops, junction FSM, interference and route STOP; %u checks\n",checks);
    return 0;
}
