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
static const RouteStep long_route[] = {{1000.0f, 2.0f, STOP, 12, 0.0f, true}};
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
/* 独立检查三种路程与位姿：R=10、轮距 10，恒曲率 1 rad。 */
static void test_offset_odometry(void)
{
    for (unsigned mirror=0; mirror<2; ++mirror) {
        Odometry o; odometry_init(&o,calibration());
        EncoderCounts c={0};
        CHECK(odometry_update(&o,c,.005f));
        for (unsigned i=0;i<100;++i) {
            c.left+=(uint16_t)(mirror?15:5);
            c.right+=(uint16_t)(mirror?5:15);
            CHECK(odometry_update(&o,c,.005f));
        }
        NEAR(o.distance_cm,10);
        NEAR(o.left_distance_cm,mirror?15:5);
        NEAR(o.right_distance_cm,mirror?5:15);
        NEAR(o.x_cm,10*sinf(1));
        NEAR(o.y_cm,(mirror?1:-1)*10*(1-cosf(1)));
        NEAR(o.sensor_x_cm,10*sinf(1)+16*cosf(1));
        NEAR(o.sensor_y_cm,(mirror?1:-1)*(10*(1-cosf(1))+16*sinf(1)));
        NEAR(o.sensor_distance_cm,sqrtf(356));
        CHECK(o.sensor_distance_cm>o.distance_cm);
    }
}

/* 同一几何、同一入口分别验证逐项开关，以及循环 STOP 的有效动作。 */
static void test_corner_mode_switch(void)
{
    for (unsigned enabled=0;enabled<2;++enabled) for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{100,16,mirror?RIGHT:LEFT,8,0,enabled!=0},
                           {131,16,STOP,8,0,enabled!=0}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        f.route.state=ROUTE_EXECUTING;
        f.route.phase_ms=now;
        f.route.entry_cm=f.route.projection_previous_cm=f.odometry.distance_cm;
        f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
        f.base=8;
        tick(&f,&c,&now,0,0,0,false);
        CHECK(route_uses_corner_arc(&f.route)==(enabled!=0));
        if (enabled) {
            NEAR(f.left_target_cm_s,8); NEAR(f.right_target_cm_s,8);
        } else {
            NEAR(f.left_target_cm_s,mirror?16:0);
            NEAR(f.right_target_cm_s,mirror?0:16);
        }
        steps[0].action=CUSTOM_TURN;
        steps[0].angle_deg=mirror?90:-90;
        CHECK(route_uses_corner_arc(&f.route)==(enabled!=0));
        steps[0].angle_deg=mirror?120:-120;
        CHECK(!route_uses_corner_arc(&f.route));
        steps[0].action=STRAIGHT;
        CHECK(!route_uses_corner_arc(&f.route));
        f.route.index=1;
        f.route.repeat_count=1;
        CHECK(!route_uses_corner_arc(&f.route)); /* 最终轮 STOP。 */
        f.route.repeat_count=0;
        CHECK(route_uses_corner_arc(&f.route)==(enabled!=0));
    }
    const RouteStep omitted[]={{.distance=10,.tolerance=2,.action=LEFT,.speed=8},
                              {.distance=100,.tolerance=2,.action=STOP,.speed=8}};
    RoutePlanner p; init_route_once(&p,omitted,2,NULL,NULL);
    CHECK(p.valid && !route_uses_corner_arc(&p)); /* 省略新增字段时默认为 false。 */
}

/* 已标定几何下，切点前两轮前进；圆弧中窄线不改变 R=10 的规划轮速。 */
static void test_corner_kinematics(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{100,16,mirror?RIGHT:LEFT,60,0, true},{131,16,STOP,60,0, true}};
        for (unsigned phase=0;phase<4;++phase) {
            LineFollow f; EncoderCounts c; uint32_t now=0;
            start(&f,&c,&now,steps,2);
            f.odometry.calibration.track_cm=WHEEL_TRACK_CM;
            float angle=phase==0?0:.3f*(float)phase;
            f.route.state=ROUTE_EXECUTING;
            f.route.phase_ms=now;
            f.route.entry_cm=0;
            f.odometry.distance_cm=phase==0?5.9f:6+10*angle;
            f.odometry.heading_rad=(mirror?1:-1)*angle;
            f.route.projection_previous_cm=f.odometry.distance_cm;
            f.route.projection_previous_heading=f.odometry.heading_rad;
            f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
            f.base=JUNCTION_TURN_SPEED_CM_S;
            tick(&f,&c,&now,0,0,phase%2?0x04:0,false);
            CHECK(f.route.state==ROUTE_EXECUTING && !f.pid.primed);
            if (phase==0) {
                NEAR(f.left_target_cm_s,18); NEAR(f.right_target_cm_s,18);
            } else {
                NEAR(f.left_target_cm_s,mirror?31.05f:4.95f);
                NEAR(f.right_target_cm_s,mirror?4.95f:31.05f);
                NEAR(WHEEL_TRACK_CM*f.base/fabsf(f.left_target_cm_s-f.right_target_cm_s),10);
            }
            if (phase==3) {
                f.odometry.heading_rad=(mirror?1:-1)*(JUNCTION_TURN_TARGET_RAD+.01f);
                tick(&f,&c,&now,0,0,0,false);
                CHECK(f.route.corner_arc_complete && f.route.state==ROUTE_EXECUTING);
                f.odometry.heading_rad=(mirror?1:-1)*(JUNCTION_TURN_TARGET_RAD-.10f);
                tick(&f,&c,&now,0,0,0x04,false);
                CHECK(f.route.corner_arc_complete && f.pid.primed);
                CHECK(f.route.state==ROUTE_EXECUTING && f.base<=JUNCTION_CAPTURE_SPEED_CM_S);
            }
        }
    }
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
    RouteStep table[]={{10,2,LEFT,8, 0.0f, true},{100,2,STOP,8, 0.0f, true}};
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
        confirm(&p,0xff,origin+99,120); CHECK(p.state==ROUTE_FINISHED && p.stop_pending);
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
    RouteStep stop[]={{1,.3f,STOP,8, 0.0f, true}};
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
    RouteStep steps[]={{1,.3f,LEFT,8, 0.0f, true},{30,.3f,STOP,8, 0.0f, true}};
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
        RouteStep steps[]={{1,.3f,mirror?RIGHT:LEFT,8, 0.0f, true},{100,5,STOP,40, 0.0f, true}};
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
            /* 持续无线且转角不足：新在线退出规则不适用，仍须超时停车并锁存快照。 */
            tick(&f,&c,&now,3,3,0,false);
            if(f.state!=LINE_STOPPED) {
                CHECK(f.route.state==ROUTE_EXECUTING && !f.route.turn_capturing);
                CHECK(f.command.enabled);
            }
        }
        CHECK(straight_frames>100 && f.stop_reason==LINE_STOP_ROUTE);
        LineStopSnapshot saved=f.stop_snapshot;
        CHECK(saved.valid && saved.reason==LINE_STOP_ROUTE);
        CHECK(saved.state_before_stop==LINE_JUNCTION);
        CHECK(saved.route_state_before_update==ROUTE_EXECUTING);
        CHECK(saved.route_state==ROUTE_FAILED && saved.route_failure==ROUTE_FAILURE_ACTION_TIMEOUT);
        CHECK(saved.route_phase_age_ms>=JUNCTION_ACTION_TIMEOUT_MS);
        CHECK(saved.observation.line_mask==0 && saved.observation.kind==GRAY_NONE);
        CHECK(fabsf(saved.turn_rad)<JUNCTION_TURN_MIN_RAD);
        CHECK(saved.left_target_cm_s+saved.right_target_cm_s>0 && f.left_target_cm_s==0);
        for(unsigned i=0;i<20;++i) tick(&f,&c,&now,0,0,0,false);
        CHECK(f.stop_snapshot.sample_ms==saved.sample_ms);
        CHECK(f.stop_snapshot.observation.line_mask==0);
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
    const int speeds[] = {12, 25, 50};
    for (size_t s = 0; s < sizeof(speeds)/sizeof(speeds[0]); ++s) {
        RouteStep steps[] = {{1000, 2, STOP, speeds[s], 0.0f, true}};
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
    const int speeds[]={4,12,50};
    for (unsigned mirror=0;mirror<2;++mirror) for (unsigned n=0;n<3;++n) {
        RouteStep steps[]={{100,8,mirror?RIGHT:LEFT,speeds[n], 0.0f, true},{100,5,STOP,40, 0.0f, true}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        float previous=0;
        bool saw_deceleration=false, saw_window=false;
        while (f.odometry.distance_cm-f.route.origin_cm<103 && f.state!=LINE_STOPPED) {
            tick(&f,&c,&now,speeds[n]/2,speeds[n]/2,0x18,false);
            float progress=f.odometry.distance_cm-f.route.origin_cm;
            float remaining=steps[0].distance-progress;
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
        RouteStep short_step[]={{10,2,STOP,40, 0.0f, true}};
        start(&f,&c,&now,short_step,1);
        for (unsigned i=0;i<25;++i) tick(&f,&c,&now,20,20,0x18,false);
        CHECK(f.route.state==ROUTE_FOLLOW && f.command.enabled);
        float before=f.base;
        tick(&f,&c,&now,20,20,0xff,false); /* 仍在窗口外，不能提前确认 STOP。 */
        CHECK(f.route.state==ROUTE_FOLLOW && !f.route.stop_pending);
        /* HOLD 上限由用户配置；高于当前巡航时仍遵守单周期上升斜率。 */
        CHECK(f.base<=before+TARGET_RISE_CM_S2*.005f+.001f &&
              f.base<=fminf(40.0f,HOLD_SPEED_CM_S));
    }
}

/* 真实走完左右转确认，再用中心线验证离口中即提速、下一段低速/短段包络及里程去重。 */
static void test_early_exit_acceleration(void)
{
    const int speeds[]={4,12,50};
    for (unsigned mirror=0;mirror<2;++mirror) for (unsigned n=0;n<3;++n)
    for (unsigned short_next=0;short_next<2;++short_next) {
        RouteStep steps[]={{100,8,mirror?RIGHT:LEFT,40, 0.0f, true},
                           {short_next?40:100,2,STOP,speeds[n], 0.0f, true}};
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
        float remaining=steps[1].distance-f.route.outgoing_cm;
        if (short_next && remaining<JUNCTION_APPROACH_DISTANCE_CM && expected>JUNCTION_APPROACH_SPEED_CM_S) {
            float slow=JUNCTION_APPROACH_SPEED_CM_S;
            expected=sqrtf(slow*slow+(expected*expected-slow*slow)*fmaxf(0.0f,remaining)/JUNCTION_APPROACH_DISTANCE_CM);
        }
        if ((float)speeds[n]>before_exit) CHECK(f.base>before_exit); /* 首个离口周期就上升，不等 3 cm。 */
        else NEAR(f.base,(float)speeds[n]); /* 下一段低速不能沿用旧段 40 cm/s。 */
        uint32_t began=now;
        while (now-began<280U && f.route.state==ROUTE_CLEARING && f.state!=LINE_STOPPED) {
            float previous=f.base;
            tick(&f,&c,&now,5,5,0x18,false); /* 280 ms 只走 2.8 cm，仍处于去重区。 */
            CHECK(f.route.state==ROUTE_CLEARING && f.route.index==0);
            NEAR(f.route.origin_cm,old_origin);
            CHECK(f.base<=expected+.001f);
            CHECK(f.base>=previous-.001f);
            CHECK(f.base<=previous+TARGET_RISE_CM_S2*.005f+.001f);
        }
        CHECK(f.route.state==ROUTE_CLEARING && f.odometry.distance_cm-clear_cm<3.0f);
        NEAR(f.base,expected); /* 6→50 在 280 ms 内完成目标规划。 */
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
    RouteStep steps[]={{131,8,LEFT,12, 0.0f, true},{70,5,STOP,12, 0.0f, true}};
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
    route_update(&p,mask(0),131,-1.5f,p.phase_ms+JUNCTION_ACTION_TIMEOUT_MS);
    CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
}

/* 真实配置轮距和计数比例下的理想运动学回归；灰度与轮速为合成输入，不是实车证据。 */
static void test_reference_path(void)
{
    const RouteStep reference_steps[]={{75,10,LEFT,60,0, true},{219,16,LEFT,60,0, true},
                                       {75,10,LEFT,60,0, true},{219,16,STOP,60,0, true}};
    const size_t reference_count=sizeof(reference_steps)/sizeof(reference_steps[0]);
    const float distances[]={75,219,75,219};
    for (size_t i=0;i<reference_count;++i) {
        NEAR(reference_steps[i].distance,distances[i]); CHECK(reference_steps[i].speed>0 && reference_steps[i].speed<=WHEEL_TARGET_MAX_CM_S);
        CHECK(i+1U==reference_count ? reference_steps[i].action==STOP :
              (reference_steps[i].action==LEFT || reference_steps[i].action==RIGHT));
    }
    for (unsigned mirror=0;mirror<2;++mirror) for (uint32_t limit=0;limit<=2;++limit) {
        RouteStep steps[4];
        for (size_t i=0;i<reference_count;++i) {
            steps[i]=reference_steps[i];
            if (mirror) {
                if (steps[i].action==LEFT) steps[i].action=RIGHT;
                else if (steps[i].action==RIGHT) steps[i].action=LEFT;
            }
        }
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        OdometryCalibration geometry={4.8f,4.8f,1560,1560,14.5f,-1,1,true};
        init_follow_once(&f,geometry,steps,reference_count,NULL,NULL);
        for (unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i)
            tick(&f,&c,&now,0,0,0x18,i==LINE_CONFIRM_SAMPLES-1U);
        f.route.repeat_count=limit;
        f.route.repeat_action=mirror ? RIGHT : LEFT;
        unsigned expected_laps=limit ? limit : 2;
        float residual_l=0,residual_r=0;
        bool saw_fast=false, saw_arc=false, saw_approach=false, saw_reacceleration=false;
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
                if (progress>=step.distance) {
                    CHECK(f.base<=JUNCTION_APPROACH_SPEED_CM_S+.001f);
                    saw_approach=true;
                }
            } else if (f.route.state==ROUTE_EXECUTING && route_current_action(&f.route)==STRAIGHT) {
                line=0x18;
                CHECK(fabsf(f.left_target_cm_s-f.right_target_cm_s)<.1f);
            } else if (f.route.state==ROUTE_EXECUTING) {
                float turn=fabsf(f.odometry.heading_rad-f.route.entry_heading);
                line=turn>=1.48f ? 0x18 : 0;
                CHECK(f.left_target_cm_s>=0 && f.right_target_cm_s>=0);
                CHECK(f.base<=JUNCTION_TURN_SPEED_CM_S+.001f);
                if (turn>0.2f && turn<1.2f) {
                    CHECK(f.left_target_cm_s>0 && f.right_target_cm_s>0);
                    saw_arc=true;
                }
                if (f.base>JUNCTION_TURN_SPEED_CM_S-.01f) saw_fast=true;
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
                if (previous_index+1U<reference_count) ++completed_turns;
            }
            previous=f.route.state;
        }
        CHECK(saw_fast && saw_arc && saw_approach && saw_reacceleration && completed_turns==3U*expected_laps);
        CHECK(f.route.completed_laps==expected_laps && f.route.failure==ROUTE_FAILURE_NONE);
        if (limit) {
            CHECK(f.stop_reason==LINE_STOP_ROUTE_COMPLETE && f.route.index==3);
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
    const RouteStep steps[]={{100,16,LEFT,40, 0.0f, true},{131,16,STOP,40, 0.0f, true}};
    RoutePlanner p;
    init_route_once(&p,steps,2,NULL,NULL);
    route_start(&p,0,0);
    p.state=ROUTE_EXECUTING;
    p.entry_cm=p.projection_previous_cm=100;
    for(unsigned i=0;i<3;++i)
        route_update(&p,mask(0x18),110,-1.5f,JUNCTION_ACTION_TIMEOUT_MS-110+i*5);
    CHECK(p.state==ROUTE_CLEARING && p.phase_ms==JUNCTION_ACTION_TIMEOUT_MS-100);
    for(unsigned i=0;i<3;++i)
        route_update(&p,mask(0x18),110.288f,-1.5f,JUNCTION_ACTION_TIMEOUT_MS+i*5);
    CHECK(p.state==ROUTE_CLEARING && p.phase_ms==JUNCTION_ACTION_TIMEOUT_MS-100);
    route_update(&p,mask(0x18),113.1f,-1.5f,JUNCTION_ACTION_TIMEOUT_MS+100);
    CHECK(p.state==ROUTE_FOLLOW && p.index==1);
    /* 同样晚入离口但未离开足够距离，独立时限仍停车。 */
    route_start(&p,0,0);
    p.state=ROUTE_EXECUTING;
    p.entry_cm=p.projection_previous_cm=100;
    for(unsigned i=0;i<3;++i)
        route_update(&p,mask(0x18),110,-1.5f,JUNCTION_ACTION_TIMEOUT_MS-110+i*5);
    route_update(&p,mask(0x18),110.288f,-1.5f,JUNCTION_ACTION_TIMEOUT_MS-100+JUNCTION_CLEAR_TIMEOUT_MS);
    CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
}

static void test_near_angle_exit(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{100,16,mirror?RIGHT:LEFT,40, 0.0f, true},{131,16,STOP,40, 0.0f, true}};
        for(unsigned scenario=0;scenario<6;++scenario) {
            RoutePlanner p;
            init_route_once(&p,steps,2,NULL,NULL);
            route_start(&p,0,0);
            p.state=ROUTE_EXECUTING;
            p.entry_cm=p.projection_previous_cm=100;
            float heading=(mirror?1.0f:-1.0f)*(scenario==1 ? JUNCTION_TURN_MIN_RAD-JUNCTION_EXIT_ANGLE_MARGIN_RAD-.01f :
                JUNCTION_TURN_MIN_RAD-JUNCTION_EXIT_ANGLE_MARGIN_RAD+.0274f);
            p.projection_previous_heading=heading;
            /* 连续中心线、两路中心偏差=2，重现板上 0x30。 */
            for(unsigned i=0;i<28;++i) {
                float distance=100+(scenario==2?0:.03f*(float)i);
                uint8_t line=scenario==3?0xff:(mirror?0x0c:0x30);
                if(scenario==5) {
                    /* 位置 2/3 往返，最后三帧居中；不能因窄线横移持续清空证据。 */
                    line=i<24 && i%2 ? (mirror?0x04:0x20) : (mirror?0x0c:0x30);
                }
                if(scenario==4 && i%10==9) line=0; /* 孤立/中断证据不得累积。 */
                route_update(&p,mask(line),distance,heading,200+i*5);
                if (i<10) CHECK(p.state==ROUTE_EXECUTING);
            }
            if(scenario==0 || scenario==5) CHECK(p.state==ROUTE_CLEARING);
            else CHECK(p.state==ROUTE_EXECUTING);
        }
    }
}

/* 放宽后的出口仍同时检查时间、距离、角度与最终中心帧，左右镜像一致。 */
static void test_exit_evidence_boundaries(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{100,16,mirror?RIGHT:LEFT,50, 0.0f, true},{100,16,STOP,50, 0.0f, true}};
        RoutePlanner p;
        init_route_once(&p,steps,2,NULL,NULL);
        route_start(&p,0,0);
        p.state=ROUTE_EXECUTING;
        p.entry_cm=p.projection_previous_cm=100;
        float heading=(mirror?1.0f:-1.0f)*(JUNCTION_TURN_MIN_RAD-JUNCTION_EXIT_ANGLE_MARGIN_RAD+.01f);
        route_update(&p,mask(0x18),100,heading,200);
        route_update(&p,mask(0x18),100.26f,heading,249); /* 距离足够、时间未到。 */
        CHECK(p.state==ROUTE_EXECUTING && p.exit_count==0);
        route_update(&p,mask(0x18),100.24f,heading,250); /* 时间足够、净前进不足。 */
        CHECK(p.state==ROUTE_EXECUTING && p.exit_count==0);
        route_update(&p,mask(mirror?0x04:0x20),100.26f,heading,255);
        CHECK(p.state==ROUTE_EXECUTING && p.exit_count==0); /* 窄线仍须最终居中。 */
        for(unsigned i=0;i<3;++i) {
            route_update(&p,mask(0x18),100.27f+.01f*(float)i,heading,260+5*i);
            CHECK(i==2 ? p.state==ROUTE_CLEARING : p.state==ROUTE_EXECUTING);
        }
    }
}

/* 仅靠有界出口条件即可进入离口并在同周期升速，不等待正常角度或 3 cm。 */
static void test_relaxed_exit_acceleration(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{100,16,mirror?RIGHT:LEFT,50, 0.0f, true},{100,16,STOP,50, 0.0f, true}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        f.route.state=ROUTE_EXECUTING;
        f.route.phase_ms=now;
        f.route.entry_cm=f.route.projection_previous_cm=f.odometry.distance_cm;
        f.odometry.heading_rad=(mirror?1.0f:-1.0f)*(JUNCTION_TURN_MIN_RAD-JUNCTION_EXIT_ANGLE_MARGIN_RAD+.01f);
        f.route.projection_previous_heading=f.odometry.heading_rad;
        f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
        f.base=6;
        float before_exit=f.base;
        for(unsigned i=0;i<30 && f.route.state==ROUTE_EXECUTING;++i) {
            before_exit=f.base;
            tick(&f,&c,&now,3,3,0x18,false);
        }
        CHECK(f.route.state==ROUTE_CLEARING && f.route.index==0);
        CHECK(fabsf(f.odometry.heading_rad)<JUNCTION_TURN_MIN_RAD);
        CHECK(f.base>before_exit && f.base<=before_exit+TARGET_RISE_CM_S2*.005f+.001f);
        NEAR(f.odometry.distance_cm,f.route.clear_cm);
    }
}

/* 圆弧中途遇到窄线仍冻结位置 PID，轨迹航向由里程决定；退出门禁不变。 */
static void test_capture_cancellation(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{100,16,mirror?RIGHT:LEFT,40,0, true},{131,16,STOP,40,0, true}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        float direction=mirror?1.0f:-1.0f;
        f.route.state=ROUTE_EXECUTING;
        f.route.phase_ms=now;
        f.route.entry_cm=f.route.projection_previous_cm=f.odometry.distance_cm;
        f.odometry.heading_rad=direction;
        f.route.projection_previous_heading=f.odometry.heading_rad;
        f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
        f.base=6;
        for(unsigned i=0;i<8;++i) {
            tick(&f,&c,&now,3,3,mirror?0x08:0x10,false);
            CHECK(f.route.state==ROUTE_EXECUTING && !f.pid.primed);
            CHECK(f.base<=JUNCTION_TURN_SPEED_CM_S);
        }
        while(direction*f.odometry.heading_rad<1.46f)
            tick(&f,&c,&now,mirror?6:0,mirror?0:6,0,false);
        for(unsigned i=0;i<3;++i) tick(&f,&c,&now,3,3,0x18,false);
        CHECK(f.route.state==ROUTE_CLEARING && f.state!=LINE_STOPPED);
    }
}

/* 实车 +120° 快照：119.16°、窄线 position=-3 长时间存在，
 * 最小同向半差不能在有界角度/沿线证据充分后继续压住回线修正。 */
static void test_custom_capture_recentering(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) {
        float sign=mirror?-1.0f:1.0f;
        RouteStep steps[]={{55,5,CUSTOM_TURN,60,sign*120, true},{55,5,STOP,60,0, true}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        f.route.state=ROUTE_EXECUTING;
        f.route.phase_ms=now;
        f.route.entry_heading=0;
        f.route.entry_cm=f.route.projection_previous_cm=f.odometry.distance_cm;
        f.odometry.heading_rad=sign*2.07972598f;
        f.route.projection_previous_heading=f.odometry.heading_rad;
        f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
        f.base=6;
        for(unsigned i=0;i<30;++i) {
            tick(&f,&c,&now,3,3,mirror?0x20:0x04,false);
            CHECK(f.route.state==ROUTE_EXECUTING && f.state!=LINE_STOPPED);
            CHECK(f.left_target_cm_s>=0 && f.right_target_cm_s>=0);
            if(i<8) CHECK(sign*(f.left_target_cm_s-f.right_target_cm_s)>=
                          2*JUNCTION_CAPTURE_MIN_TURN_CM_S-.001f);
        }
        CHECK(f.route.exit_candidate_pending);
        CHECK(sign*(f.left_target_cm_s-f.right_target_cm_s)<0); /* 允许向出口线回正。 */
        /* 没有中心线不能直接离口；证据中断后再次启用最小同向转向。 */
        tick(&f,&c,&now,3,3,0,false);
        CHECK(!f.route.exit_candidate_pending && f.route.state==ROUTE_EXECUTING);
        tick(&f,&c,&now,3,3,mirror?0x20:0x04,false);
        CHECK(sign*(f.left_target_cm_s-f.right_target_cm_s)>=
              2*JUNCTION_CAPTURE_MIN_TURN_CM_S-.001f);
        for(unsigned i=0;i<30;++i) tick(&f,&c,&now,3,3,mirror?0x20:0x04,false);
        for(unsigned i=0;i<3;++i) tick(&f,&c,&now,3,3,0x18,false);
        CHECK(f.route.state==ROUTE_CLEARING && f.state!=LINE_STOPPED);
        CHECK(f.base>6);
        /* 离口仍须距离去重；随后能推进下一项而非停在慢速动作。 */
        for(unsigned i=0;i<110 && f.route.index==0;++i) tick(&f,&c,&now,3,3,0x18,false);
        CHECK(f.route.index==1 && f.route.state==ROUTE_FOLLOW && f.state!=LINE_STOPPED);
    }
}

/* 修正版实车：回线后角度降到 2.037906 rad，低于 120°-0.05，
 * 已成熟出口证据不能因小幅回正反复清零；无先前证据不能借用回差。 */
static void test_custom_exit_angle_hysteresis(void)
{
    for(unsigned mirror=0;mirror<2;++mirror) for(unsigned scenario=0;scenario<5;++scenario) {
        float sign=mirror?-1.0f:1.0f;
        RouteStep steps[]={{55,5,CUSTOM_TURN,60,sign*120, true},{55,5,STOP,60,0, true}};
        RoutePlanner p; init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0);
        p.state=ROUTE_EXECUTING; p.phase_ms=100;
        p.entry_cm=p.projection_previous_cm=60;
        float target=fabsf(route_turn_target_rad(&p));
        if(scenario!=1) {
            for(unsigned i=0;i<16;++i)
                route_update(&p,mask(mirror?0x20:0x04),60+.03f*i,
                             sign*(target-.04f),105+5*i);
            CHECK(p.exit_candidate_pending && p.state==ROUTE_EXECUTING);
        }
        float angle=scenario==2?target-.08f:2.03790617f;
        for(unsigned i=0;i<3;++i)
            route_update(&p,mask(scenario==3?0xff:scenario==4?0:mirror?0x30:0x0c),
                         60.5f+.03f*i,sign*angle,185+5*i);
        CHECK(scenario==0 ? p.state==ROUTE_CLEARING : p.state==ROUTE_EXECUTING);
        if(scenario!=0) CHECK(!p.exit_candidate_pending);
    }
    /* 控制器也须保持回线权，再由中心线确认进入离口并加速。 */
    for(unsigned mirror=0;mirror<2;++mirror) {
        float sign=mirror?-1.0f:1.0f;
        RouteStep steps[]={{55,5,CUSTOM_TURN,60,sign*120, true},{55,5,STOP,60,0, true}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        f.route.state=ROUTE_EXECUTING; f.route.phase_ms=now;
        f.route.entry_heading=0;
        f.route.entry_cm=f.route.projection_previous_cm=f.odometry.distance_cm;
        f.route.projection_previous_heading=f.odometry.heading_rad=sign*2.07972598f;
        f.startup_elapsed_s=STARTUP_RAMP_TIME_S; f.base=6;
        for(unsigned i=0;i<30;++i) tick(&f,&c,&now,3,3,mirror?0x20:0x04,false);
        CHECK(f.route.exit_candidate_pending);
        f.odometry.heading_rad=sign*2.03790617f;
        tick(&f,&c,&now,3,3,mirror?0x20:0x04,false);
        CHECK(f.route.exit_candidate_pending && f.route.state==ROUTE_EXECUTING);
        CHECK(sign*(f.left_target_cm_s-f.right_target_cm_s)<0);
        for(unsigned i=0;i<3;++i) tick(&f,&c,&now,3,3,mirror?0x30:0x0c,false);
        CHECK(f.route.state==ROUTE_CLEARING && f.state!=LINE_STOPPED);
        CHECK(f.base>6);
        for(unsigned i=0;i<110 && f.route.index==0;++i) tick(&f,&c,&now,3,3,0x18,false);
        CHECK(f.route.state==ROUTE_FOLLOW && f.route.index==1 && f.state!=LINE_STOPPED);
    }
}

/* 自定义转弯无线时不能在目标角后继续固定同向单轮扫线。 */
static void test_custom_turn_heading_limit(void)
{
    for(unsigned mirror=0;mirror<2;++mirror) {
        float sign=mirror?-1.0f:1.0f;
        RouteStep steps[]={{55,5,CUSTOM_TURN,60,sign*120, true},{100,5,STOP,60,0, true}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        f.route.state=ROUTE_EXECUTING; f.route.phase_ms=now;
        f.route.entry_heading=.3f;
        f.route.entry_cm=f.route.projection_previous_cm=f.odometry.distance_cm;
        f.startup_elapsed_s=STARTUP_RAMP_TIME_S; f.base=3;
        float target=fabsf(route_turn_target_rad(&f.route));
        const float errors[]={.2f,.05f,0,-.05f};
        float previous=100;
        for(unsigned i=0;i<4;++i) {
            f.odometry.heading_rad=.3f+sign*(target-errors[i]);
            tick(&f,&c,&now,0,0,0,false);
            CHECK(f.route.state==ROUTE_EXECUTING && !f.route.turn_capturing);
            CHECK(f.left_target_cm_s>=0 && f.right_target_cm_s>=0);
            float diff=sign*(f.left_target_cm_s-f.right_target_cm_s);
            CHECK(diff<previous); previous=diff;
            if(i==2) CHECK(fabsf(diff)<.001f);
            if(i==3) CHECK(diff<0);
        }
        /* 理想轮速反馈下，持续无线仍停留在目标附近；不能假成功，最终动作超时。 */
        f.odometry.heading_rad=.3f+sign*(target-.2f);
        for(unsigned i=0;i<300 && f.state!=LINE_STOPPED;++i) {
            int l=(int)lroundf(f.left_target_cm_s*.5f);
            int r=(int)lroundf(f.right_target_cm_s*.5f);
            tick(&f,&c,&now,l,r,0,false);
            CHECK(sign*(f.odometry.heading_rad-.3f)<target+.05f);
            CHECK(f.route.state==ROUTE_EXECUTING);
        }
        now=f.route.phase_ms+route_action_timeout_ms(&f.route);
        line_follow_update(&f,mask(0),c,now,.005f,false);
        CHECK(f.stop_reason==LINE_STOP_ROUTE && f.route.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
    }
}

static void test_corner_distance_projection(void)
{
    for (unsigned mirror=0; mirror<2; ++mirror) {
        RouteStep steps[]={{100,16,mirror?RIGHT:LEFT,40, 0.0f, true},{131,16,STOP,40, 0.0f, true}};
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
        NEAR(p.outgoing_cm,10.0f+GRAY_FORWARD_OFFSET_CM);
        for (unsigned i=0;i<3;++i)
            route_update(&p,mask(0x18),100.0f+arc,sign*JUNCTION_TURN_TARGET_RAD,110+i*5);
        CHECK(p.state==ROUTE_CLEARING);
        for (unsigned i=0;i<3;++i)
            route_update(&p,mask(0x18),103.1f+arc,sign*JUNCTION_TURN_TARGET_RAD,130+i*5);
        CHECK(p.state==ROUTE_FOLLOW && p.index==1);
        NEAR(103.1f+arc-p.origin_cm,13.1f+GRAY_FORWARD_OFFSET_CM);
    }
}

/* 末项大容差不提前消费；非最终轮在末端左转，骤变也只计轮一次。 */
static void test_loop_end_corner(void)
{
    const RouteStep steps[]={{219,16,STOP,50, 0.0f, true}};
    RoutePlanner p;
    route_init(&p,steps,1,NULL,NULL);
    p.repeat_count=0; /* 本用例显式测试无限轮，与生产轮数无关。 */
    CHECK(p.repeat_action==LEFT);
    route_start(&p,0,0);
    confirm(&p,0xff,204,10);
    CHECK(p.state==ROUTE_FOLLOW && p.completed_laps==0);
    for (unsigned i=0;i<3;++i) route_update(&p,mask(0x18),213.9f,0,30+5*i);
    route_update(&p,mask(0),214,0,45);
    CHECK(p.state==ROUTE_EXECUTING && p.turn_forced && p.lap_boundary);
    CHECK(p.completed_laps==1 && route_current_action(&p)==LEFT && !p.stop_pending);
    for (unsigned i=0;i<3;++i) route_update(&p,mask(0x18),224,-1.5f,50+5*i);
    CHECK(p.state==ROUTE_CLEARING && p.completed_laps==1);
    for (unsigned i=0;i<3;++i) route_update(&p,mask(0x18),227.1f,-1.5f,70+5*i);
    CHECK(p.state==ROUTE_FOLLOW && p.index==0 && !p.lap_boundary);
    p.repeat_count=1;
    route_start(&p,0,0);
    confirm(&p,0xff,204,10);
    CHECK(p.state==ROUTE_FOLLOW && !p.stop_pending);
    confirm(&p,0xff,219,30);
    CHECK(p.state==ROUTE_FINISHED && p.completed_laps==1 && p.stop_pending);

    LineFollow f; EncoderCounts c={0}; uint32_t now=0;
    start(&f,&c,&now,steps,1);
    for (unsigned i=0;i<250;++i) tick(&f,&c,&now,25,25,0x18,false);
    f.route.origin_cm=f.odometry.distance_cm-203;
    tick(&f,&c,&now,25,25,0x18,false);
    CHECK(f.route.state==ROUTE_FOLLOW); NEAR(f.base,50);
    f.route.origin_cm=f.odometry.distance_cm-214;
    tick(&f,&c,&now,25,25,0x18,false);
    CHECK(f.base<50 && f.base>10);
}

static void test_route_repeats(void)
{
    const RouteStep stop[]={{10,2,STOP,8, 0.0f, true}};
    RoutePlanner p;
    route_init(&p,stop,1,NULL,NULL);
    CHECK(p.repeat_count == ROUTE_REPEAT_COUNT);
    CHECK(p.repeat_action == LEFT);
    p.repeat_action = STRAIGHT;
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
    route_update(&p,mask(0),13,0,p.phase_ms+route_action_timeout_ms(&p));
    CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT && !p.stop_pending);

    /* 生产控制链中，前几轮不能停车或发成功回调，末轮才清空电机并分派一次。 */
    RouteStep short_stop[]={{10,.3f,STOP,8, 0.0f, true}};
    for (uint32_t limit=0;limit<=2;++limit) {
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        unsigned before=callbacks;
        start(&f,&c,&now,short_stop,1); f.route.repeat_count=limit;
        f.route.repeat_action=STRAIGHT;
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
        RouteStep steps[]={{1,.3f,mirror?RIGHT:LEFT,8, 0.0f, true},{10,2,STOP,8, 0.0f, true}};
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        while(f.odometry.distance_cm-f.route.origin_cm<.85f)
            tick(&f,&c,&now,5,5,0x18,false);
        for(unsigned i=0;i<5 && f.route.state!=ROUTE_EXECUTING;++i)
            tick(&f,&c,&now,5,5,mirror?0xf8:0x1f,false);
        CHECK(f.route.state==ROUTE_EXECUTING);
        float direction=mirror?1.0f:-1.0f;
        while(direction*(f.odometry.heading_rad-f.route.entry_heading)<1.0f)
            tick(&f,&c,&now,mirror?12:2,mirror?2:12,0,false);
        /* 出口在转向外侧出现；旧代码仍给内轮零，无法用位置反馈修正。 */
        tick(&f,&c,&now,mirror?12:2,mirror?2:12,mirror?0x02:0x40,false);
        CHECK(f.route.turn_capturing && !f.pid.primed && f.route.state==ROUTE_EXECUTING);
        CHECK(f.left_target_cm_s>0 && f.right_target_cm_s>0);
        CHECK(f.base<=JUNCTION_TURN_SPEED_CM_S);
        tick(&f,&c,&now,3,3,0xff,false);
        CHECK(!f.route.turn_capturing && !f.pid.primed && f.route.state==ROUTE_EXECUTING);
        tick(&f,&c,&now,3,3,0x42,false);
        CHECK(!f.route.turn_capturing && !f.pid.primed);
        while(direction*(f.odometry.heading_rad-f.route.entry_heading)<1.46f)
            tick(&f,&c,&now,mirror?12:2,mirror?2:12,0,false);
        for (unsigned i=0;i<3;++i) tick(&f,&c,&now,3,3,0x18,false);
        CHECK(f.route.state==ROUTE_CLEARING && !f.route.turn_capturing);
    }
}

static void test_l_corner_confirmation(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{10,2,mirror?RIGHT:LEFT,8, 0.0f, true},{10,2,STOP,8, 0.0f, true}};
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
        RouteStep short_steps[]={{1,.3f,mirror?RIGHT:LEFT,8, 0.0f, true},{10,2,STOP,8, 0.0f, true}};
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        start(&f,&c,&now,short_steps,2);
        while(f.odometry.distance_cm-f.route.origin_cm<.9f)
            tick(&f,&c,&now,5,5,0x18,false);
        for(unsigned j=0;j<5 && f.route.state!=ROUTE_EXECUTING;++j)
            tick(&f,&c,&now,5,5,edge,false);
        CHECK(f.state==LINE_JUNCTION && f.route.state==ROUTE_EXECUTING && f.command.enabled);
        NEAR(f.left_target_cm_s,f.right_target_cm_s); /* 先到圆弧切点。 */
    }
    for (unsigned action=STRAIGHT;action<=STOP;++action) {
        RouteStep steps[]={{10,2,(Action)action,8, 0.0f, true},{10,2,STOP,8, 0.0f, true}};
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
        RouteStep steps[]={{1,.3f,mirror?RIGHT:LEFT,8, 0.0f, true},{10,2,STOP,8, 0.0f, true}};
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        while(f.odometry.distance_cm-f.route.origin_cm<.85f)
            tick(&f,&c,&now,5,5,0x18,false);
        tick(&f,&c,&now,5,5,shapes[k],false);
        CHECK(f.route.state==ROUTE_EXECUTING && f.route.turn_forced && f.state==LINE_JUNCTION);
        CHECK(!f.abnormal && !f.pid.primed && f.route.candidate_count==0);
        CHECK(route_current_action(&f.route)==(mirror?RIGHT:LEFT));
        CHECK(f.left_target_cm_s>0 && f.right_target_cm_s>0);
        NEAR(f.left_target_cm_s,f.right_target_cm_s); /* 先前进至切点。 */
        /* 入口即使仍在线上也不能撤销动作；早到中心线不能提前退出。 */
        for (unsigned j=0;j<5;++j) tick(&f,&c,&now,3,3,0x18,false);
        CHECK(f.route.state==ROUTE_EXECUTING && !f.route.turn_capturing && !f.pid.primed);
        /* 已提交动作超时时若无线，仍必须失败，不能刷新预算。 */
        route_update(&f.route,mask(0),f.odometry.distance_cm,f.odometry.heading_rad,
                     f.route.phase_ms+JUNCTION_ACTION_TIMEOUT_MS);
        CHECK(f.route.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
        route_start(&f.route,0,now);
        CHECK(!f.route.turn_forced && f.route.narrow_count==0);
    }
    RouteStep steps[]={{10,2,LEFT,8, 0.0f, true},{10,2,STOP,8, 0.0f, true}};
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
    const float offsets[]={GRAY_FORWARD_OFFSET_CM};
    for (unsigned mirror=0;mirror<2;++mirror) for (unsigned k=0;k<sizeof(offsets)/sizeof(offsets[0]);++k) {
        float front=offsets[k];
        RouteStep steps[]={{40-front,3,mirror?RIGHT:LEFT,40, 0.0f, true},{100,3,STOP,40, 0.0f, true}};
        OdometryCalibration geometry={4.8f,4.8f,1560,1560,14.5f,-1,1,true};
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        init_follow_once(&f,geometry,steps,2,NULL,NULL);
        GraySensor sensor; gray_sensor_init(&sensor);
        float x=0,y=0,h=0,rl=0,rr=0;
        bool captured=false, online_exit=false;
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
            online_exit=online_exit || f.route.turn_ended_on_line;

        }
        if(f.route.index!=1) fprintf(stderr,"spatial failure mirror=%u front=%.1f x=%.2f y=%.2f h=%.2f state=%d failure=%d stop=%d\n",
            mirror,front,x,y,h,f.route.state,f.route.failure,f.stop_reason);
        CHECK(f.route.index==1 && f.state==LINE_TRACKING && captured);
        CHECK(fabsf(x+front*cosf(h)-40)<1.5f);
        /* 在线退出优先后，不再承诺完成精确 90°；仍须沿出口线推进且方向正确。 */
        CHECK(mirror ? h>0 : h<0);
        if(!online_exit) CHECK(fabsf(fabsf(h)-JUNCTION_TURN_TARGET_RAD)<.3f);
    }
}

/* 自定义角度的几何出口仍保留符号与完整圈数；时间在线退出由独立用例覆盖。 */
static void test_custom_turns(void)
{
    const float angles[] = {-360, -270, -180, -135, -90, -45, -1, 1, 45, 90, 135, 180, 270, 360};
    for (unsigned k=0;k<sizeof(angles)/sizeof(angles[0]);++k) {
        RouteStep steps[]={{10,2,CUSTOM_TURN,8,angles[k], true},{100,2,STOP,8,0, true}};
        RoutePlanner p;
        init_route_once(&p,steps,2,NULL,NULL); CHECK(p.valid); route_start(&p,0,0);
        float target=angles[k]*JUNCTION_TURN_TARGET_RAD/90.0f;
        float sign=angles[k]<0 ? -1.0f : 1.0f;
        NEAR(route_turn_target_rad(&p),target);
        CHECK(route_current_action(&p)==(sign<0 ? LEFT : RIGHT));
        confirm(&p,sign<0 ? 0x1f : 0xf8,9,10);
        CHECK(p.state==ROUTE_EXECUTING);
        /* 在半角遇中心线，仍不能误当作完整出口。 */
        for (unsigned i=0;i<3;++i) route_update(&p,mask(0x18),9.2f,target*.5f,30+5*i);
        CHECK(p.state==ROUTE_EXECUTING);
        for (unsigned i=0;i<3;++i) route_update(&p,mask(0x18),9.3f,target,50+5*i);
        CHECK(p.state==ROUTE_CLEARING);
        for (unsigned i=0;i<3;++i) route_update(&p,mask(0x18),12.5f,target,70+5*i);
        CHECK(p.state==ROUTE_FOLLOW && p.index==1);
        /* 恒曲率圆弧投影至自定义出口轴，左右镜像结果相同。 */
        init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0); confirm(&p,0xff,9,10);
        float entry=p.entry_cm;
        for (unsigned i=1;i<=40;++i)
            route_update(&p,mask(0),entry+5.0f*fabsf(target)*(float)i/40.0f,
                         target*(float)i/40.0f,30+5*i);
        NEAR(p.outgoing_cm,5.0f*sinf(fabsf(target)) + GRAY_FORWARD_OFFSET_CM*(1.0f-cosf(target)));
        CHECK(p.state==ROUTE_EXECUTING);
        /* 有界出口须累计时间和前进；证据中断必须清空。 */
        float near=target-sign*fminf(.025f,fabsf(target)*.05f);
        route_update(&p,mask(0x18),entry+40,near,300);
        CHECK(p.state==ROUTE_EXECUTING && p.exit_candidate_pending);
        route_update(&p,mask(0xff),entry+40.1f,near,305);
        CHECK(!p.exit_candidate_pending);
        for (unsigned i=0;i<15;++i)
            route_update(&p,mask(0x18),entry+40.2f+.03f*(float)i,near,310+5*i);
        CHECK(p.state==ROUTE_CLEARING);
        /* 与控制器接通，正角左轮目标较大，负角右轮目标较大。 */
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        init_follow_once(&f,calibration(),steps,2,NULL,NULL);
        start(&f,&c,&now,steps,2);
        f.route.origin_cm=f.odometry.distance_cm-9;
        for (unsigned i=0;i<3;++i) tick(&f,&c,&now,6,6,0x18,false);
        tick(&f,&c,&now,6,6,0,false);
        CHECK(f.route.state==ROUTE_EXECUTING);
        if (fabsf(angles[k])==90.0f) NEAR(f.left_target_cm_s,f.right_target_cm_s);
        else CHECK(sign*(f.left_target_cm_s-f.right_target_cm_s)>0);
        CHECK(f.left_target_cm_s>=0 && f.right_target_cm_s>=0);
        /* 错向、过转和有限动作预算仍会停车。 */
        init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0); confirm(&p,0xff,9,10);
        route_update(&p,mask(0),9.2f,-sign*(JUNCTION_WRONG_WAY_RAD+.01f),30);
        CHECK(p.failure==ROUTE_FAILURE_WRONG_WAY);
        init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0); confirm(&p,0xff,9,10);
        route_update(&p,mask(0),9.2f,sign*(route_turn_threshold_rad(&p,JUNCTION_TURN_MAX_RAD)+.01f),30);
        CHECK(p.failure==ROUTE_FAILURE_TURN_LIMIT);
        init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0); confirm(&p,0xff,9,10);
        uint32_t budget=route_action_timeout_ms(&p);
        CHECK(budget>=JUNCTION_ACTION_TIMEOUT_MS && budget<=4*JUNCTION_ACTION_TIMEOUT_MS);
        route_update(&p,mask(0),9.2f,0,p.phase_ms+budget);
        CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
    }
    const float invalid[]={0,361,-361,NAN,INFINITY,-INFINITY};
    for (unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);++i) {
        RouteStep steps[]={{10,2,CUSTOM_TURN,8,invalid[i], true},{100,2,STOP,8,0, true}};
        RoutePlanner p; init_route_once(&p,steps,2,NULL,NULL);
        CHECK(!p.valid && p.failure==ROUTE_FAILURE_INVALID);
    }
}

/* 短下一段窗口已靠近时，固定 3 cm 离口会耗尽可用检测距离。 */
static void test_short_segment_handoff(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) for (unsigned old_case=0;old_case<2;++old_case) {
        float sign=mirror?1.0f:-1.0f;
        RouteStep steps[]={{82,5,CUSTOM_TURN,60,sign*90, true},
                           {old_case?15:10,5,CUSTOM_TURN,50,sign*60, true},
                           {100,5,STOP,40,0, true}};
        RoutePlanner p; init_route_once(&p,steps,3,NULL,NULL); route_start(&p,0,0);
        p.state=ROUTE_CLEARING; p.phase_ms=100;
        p.entry_heading=-sign*JUNCTION_TURN_TARGET_RAD;
        p.clear_cm=p.projection_previous_cm=100;
        p.projection_previous_heading=0;
        p.outgoing_cm=old_case?16.85f:10.4f;
        for (unsigned i=0;i<3;++i) route_update(&p,mask(mirror?0x60:0x06),100+.1f*(float)i,0,105+5*i);
        CHECK(p.state==ROUTE_CLEARING && p.index==0); /* 距离不足，不得只凭帧数切换。 */
        route_update(&p,mask(mirror?0x60:0x06),100.51f,0,120);
        CHECK(p.state==ROUTE_FOLLOW && p.index==1 && p.handoff_early);
        NEAR(p.segment_start_progress_cm,p.outgoing_cm);
        NEAR(100.51f-p.origin_cm,(old_case?16.85f:10.4f)+.51f);
        CHECK(!p.have_approach_center); /* 偏侧窄线不能伪造中心分支记忆。 */
        RoutePlanner missed=p;
        route_update(&missed,mask(0x18),missed.origin_cm+steps[1].distance+steps[1].tolerance+.01f,0,125);
        CHECK(missed.state==ROUTE_FAILED && missed.failure==ROUTE_FAILURE_WINDOW);
        RoutePlanner stale=p;
        route_update(&stale,mask(0),100.61f,0,120+CONTROL_MAX_INTERVAL_MS+1);
        CHECK(stale.state==ROUTE_FOLLOW && !stale.turn_forced);
        /* 交接时保留已去重的窄线历史，下一帧骤变仍可按表触发。 */
        route_update(&p,mask(0),100.61f,0,125);
        CHECK(p.state==ROUTE_EXECUTING && p.turn_forced && p.index==1);
        NEAR(route_turn_target_rad(&p),sign*JUNCTION_TURN_TARGET_RAD*2.0f/3.0f);
        LineFollow f; init_follow_once(&f,calibration(),steps,3,NULL,NULL);
        f.route=p; f.state=LINE_JUNCTION; f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
        f.odometry.distance_cm=100.61f;
        line_follow_update(&f,mask(0),(EncoderCounts){0},130,.005f,false);
        CHECK(f.state==LINE_JUNCTION && f.command.enabled && f.stop_reason==LINE_STOP_NONE);
        CHECK(sign*(f.left_target_cm_s-f.right_target_cm_s)>0);
        CHECK(f.left_target_cm_s>=0 && f.right_target_cm_s>=0);

    }
    /* 短去重不适用于歧义、静止、直行、远路口或已越窗；超时也保留。 */
    for (unsigned scenario=0;scenario<9;++scenario) {
        RouteStep steps[]={{82,5,CUSTOM_TURN,60,-90, true},{10,5,CUSTOM_TURN,50,-60, true},{100,5,STOP,40,0, true}};
        if (scenario==5) steps[0].action=STRAIGHT;
        if (scenario==6) steps[1].distance=100;
        RoutePlanner p; init_route_once(&p,steps,3,NULL,NULL); route_start(&p,0,0);
        p.state=ROUTE_CLEARING; p.phase_ms=100;
        p.entry_heading=scenario==5?0:JUNCTION_TURN_TARGET_RAD;
        p.clear_cm=p.projection_previous_cm=100;
        p.outgoing_cm=scenario==7?16:10.4f;
        const uint8_t shapes[]={0xff,0xff,0x19,0,0x18,0x18,0x18,0x18,0x18};
        for (unsigned i=0;i<3;++i) {
            float cm=scenario==4 ? 100 : 100.6f;
            uint32_t now=scenario==8 ? 100+JUNCTION_CLEAR_TIMEOUT_MS : 105+5*i;
            route_update(&p,mask(shapes[scenario]),cm,0,now);
        }
        CHECK(p.index==0 && !p.handoff_early);
        CHECK(scenario==8 ? p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_ACTION_TIMEOUT :
                            p.state==ROUTE_CLEARING);
    }
    /* 一帧中心线不能抵消中断；需重新收满三帧。 */
    RouteStep steps[]={{82,5,CUSTOM_TURN,60,-90, true},{10,5,CUSTOM_TURN,50,-60, true},{100,5,STOP,40,0, true}};
    RoutePlanner p; init_route_once(&p,steps,3,NULL,NULL); route_start(&p,0,0);
    p.state=ROUTE_CLEARING; p.phase_ms=100; p.entry_heading=JUNCTION_TURN_TARGET_RAD;
    p.clear_cm=p.projection_previous_cm=100; p.outgoing_cm=10.4f;
    route_update(&p,mask(0x18),100.6f,0,105);
    route_update(&p,mask(0),100.6f,0,110);
    for (unsigned i=0;i<3;++i) {
        route_update(&p,mask(0x18),100.6f,0,115+5*i);
        CHECK(i<2 ? p.state==ROUTE_CLEARING : p.state==ROUTE_FOLLOW && p.handoff_early);
    }
    /* 快照保留本段继承距离，START 后清空早交接标志。 */
    LineFollow f; init_follow_once(&f,calibration(),steps,3,NULL,NULL);
    f.route=p;
    line_follow_stop(&f,LINE_STOP_REQUEST);
    CHECK(f.stop_snapshot.handoff_early);
    NEAR(f.stop_snapshot.segment_start_progress_cm,p.segment_start_progress_cm);
    route_start(&p,200,200);
    CHECK(!p.handoff_early);
    NEAR(p.segment_start_progress_cm,0);

}

/* 第二次 -90° 在 0x98 多区域观测下继续扫线至 2.101 rad；有界出口不得放开普通 PID。 */
static void test_branch_exit_capture(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) for (unsigned selected=0;selected<2;++selected) {
        float sign=mirror?1.0f:-1.0f;
        RouteStep steps[]={{82,5,CUSTOM_TURN,60,sign*90, true},{10,5,CUSTOM_TURN,50,sign*60, true},{100,5,STOP,40,0, true}};
        LineFollow f; init_follow_once(&f,calibration(),steps,3,NULL,NULL);
        f.state=LINE_JUNCTION; f.startup_elapsed_s=STARTUP_RAMP_TIME_S; f.base=6;
        f.route.state=ROUTE_EXECUTING; f.route.phase_ms=100;
        f.route.entry_cm=f.route.projection_previous_cm=f.odometry.distance_cm=100;
        f.route.outgoing_cm=9;
        f.route.projection_previous_heading=f.odometry.heading_rad=sign*(JUNCTION_TURN_TARGET_RAD+.03f);
        f.pid.primed=true; f.pid.integral=7;
        GrayObservation obs=gray_classify(mirror?0x19:0x98,selected!=0,0);
        CHECK(obs.region_count==2 && !route_is_narrow_line(obs));
        EncoderCounts c={0}; uint32_t now=105;
        line_follow_update(&f,obs,c,now,.005f,false);
        CHECK(f.route.turn_exit_branch && f.route.turn_capturing && f.route.exit_line_mask==0x18);
        CHECK(f.state==LINE_JUNCTION && !f.pid.primed && f.command.enabled);
        NEAR(f.pid.integral,0);
        CHECK(f.left_target_cm_s>0 && f.right_target_cm_s>0); /* 不再退回 0/6 单轮扫线。 */
        for (unsigned i=0;i<50 && f.route.state==ROUTE_EXECUTING;++i) {
            now+=5; c.left+=3; c.right+=3;
            line_follow_update(&f,obs,c,now,.005f,false);
            CHECK(!f.pid.primed && f.state!=LINE_STOPPED);
            if (now<205) CHECK(f.route.state==ROUTE_EXECUTING);
        }
        CHECK(f.route.state==ROUTE_CLEARING && f.route.turn_exit_branch);
        for (unsigned i=0;i<50 && f.route.state==ROUTE_CLEARING;++i) {
            now+=5; c.left+=3; c.right+=3;
            line_follow_update(&f,obs,c,now,.005f,false);
            CHECK(!f.pid.primed && f.state!=LINE_STOPPED);
        }
        CHECK(f.route.index==1 && f.route.state==ROUTE_FOLLOW && f.route.handoff_early);
    }
    /* 各条件独立：早角度、过大角度、远下一项、错误侧、非中心区域、宽区及多碎片。 */
    for (unsigned scenario=0;scenario<9;++scenario) {
        RouteStep steps[]={{82,5,CUSTOM_TURN,60,-90, true},{10,5,CUSTOM_TURN,50,-60, true},{100,5,STOP,40,0, true}};
        if (scenario==2) steps[1].distance=100;
        RoutePlanner p; init_route_once(&p,steps,3,NULL,NULL); route_start(&p,0,0);
        p.state=ROUTE_EXECUTING; p.phase_ms=100;
        p.entry_cm=p.projection_previous_cm=100; p.outgoing_cm=9;
        float angle=scenario==0?1.3f:scenario==1?2.10114932f:JUNCTION_TURN_TARGET_RAD+.03f;
        const uint8_t shapes[]={0x98,0x98,0x98,0x19,0x88,0xff,0x99,0x98,0x98};
        for (unsigned i=0;i<35;++i) {
            float cm=100+(scenario==7?0:.03f*(float)i);
            uint8_t shape=scenario==8 && i%10==9 ? 0x99 : shapes[scenario];
            route_update(&p,gray_classify(shape,false,0),cm,-angle,105+5*i);
        }
        CHECK(scenario==1 ? p.failure==ROUTE_FAILURE_TURN_LIMIT : p.state==ROUTE_EXECUTING);
        if (scenario<7) CHECK(!p.turn_exit_branch);
    }
    /* 0x98 在普通巡线阶段仍是多区域；不能因此直接获得出口或转弯授权。 */
    RouteStep steps[]={{82,5,CUSTOM_TURN,60,-90, true},{10,5,CUSTOM_TURN,50,-60, true},{100,5,STOP,40,0, true}};
    LineFollow f; EncoderCounts c={0}; uint32_t now=0;
    start(&f,&c,&now,steps,3);
    tick(&f,&c,&now,3,3,0x98,false);
    CHECK(f.route.state==ROUTE_FOLLOW && f.state==LINE_GAP_HOLD && !f.route.turn_exit_branch);
}

/* 用户要求：满一秒在线即离口；超时在线也继续，不能跳过路径去重或关闭故障。 */
static void test_turn_online_exit(void)
{
    for (unsigned mode=0;mode<4;++mode) {
        RouteStep steps[]={{10,2,mode<2?(mode?RIGHT:LEFT):CUSTOM_TURN,40,
                           mode<2?0:(mode==2?-135:135), true}, {100,5,STOP,40,0, true}};
        float heading=(mode==0 || mode==2)?-.4f:.4f;
        RoutePlanner p; init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0);
        p.state=ROUTE_EXECUTING; p.phase_ms=0; p.entry_cm=p.projection_previous_cm=10;
        /* 一秒之前，三路偏侧单线也不能使用时间退出；满一秒当天即结束。 */
        for(unsigned i=0;i<3;++i) route_update(&p,mask(0x38),10,heading,990+5*i);
        CHECK(p.state==ROUTE_CLEARING && p.phase_ms==1000 && p.turn_ended_on_line);
        CHECK(p.index==0 && p.failure==ROUTE_FAILURE_NONE && !p.stop_pending && !p.turn_capturing);
        for(unsigned i=0;i<3;++i) route_update(&p,mask(0x38),13.1f,heading,1005+5*i);
        CHECK(p.state==ROUTE_FOLLOW && p.index==1 && !p.turn_ended_on_line);
        NEAR(p.origin_cm,13.1f-p.outgoing_cm);
        /* 超时这一帧在线，不受旧最小转角门槛约束；离口预算只建立一次。 */
        route_start(&p,0,0); p.state=ROUTE_EXECUTING; p.phase_ms=0;
        uint32_t timeout=route_action_timeout_ms(&p);
        route_update(&p,mask(0x38),10,heading,timeout);
        CHECK(p.state==ROUTE_CLEARING && p.phase_ms==timeout && p.turn_ended_on_line);
        route_update(&p,mask(0x38),10,heading,timeout+JUNCTION_CLEAR_TIMEOUT_MS);
        CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
        /* 无线、7/8 路、分离多区域均不能放行，选区也不能把多区域变在线。 */
        const uint8_t invalid[]={0,0x7f,0xff,0x81,0x98};
        for(unsigned i=0;i<sizeof(invalid);++i) {
            route_start(&p,0,0); p.state=ROUTE_EXECUTING; p.phase_ms=0;
            route_update(&p,gray_classify(invalid[i],true,0),10,heading,timeout);
            CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
        }
        /* 中断/迟到采样清计数，孤立单帧不能触发一秒退出。 */
        route_start(&p,0,0); p.state=ROUTE_EXECUTING; p.phase_ms=0;
        route_update(&p,mask(0x38),10,heading,990);
        route_update(&p,mask(0x38),10,heading,995);
        route_update(&p,mask(0),10,heading,1000);
        route_update(&p,mask(0x38),10,heading,1005);
        CHECK(p.state==ROUTE_EXECUTING && p.online_samples==1);
        route_update(&p,mask(0x38),10,heading,1030);
        CHECK(p.state==ROUTE_EXECUTING && p.online_samples==1);
        /* 过转与反向保护优先于超时在线放行。 */
        for(unsigned wrong=0;wrong<2;++wrong) {
            route_start(&p,0,0); p.state=ROUTE_EXECUTING; p.phase_ms=0;
            float sign=heading<0?-1:1;
            float angle=wrong?-sign*(JUNCTION_WRONG_WAY_RAD+.01f):
                sign*(route_turn_threshold_rad(&p,JUNCTION_TURN_MAX_RAD)+.01f);
            route_update(&p,mask(0x38),10,angle,timeout);
            CHECK(p.failure==(wrong?ROUTE_FAILURE_WRONG_WAY:ROUTE_FAILURE_TURN_LIMIT));
        }
    }
    /* 直行仍使用原预算，不能借转弯在线规则提前消费路径。 */
    const RouteStep straight[]={{10,2,STRAIGHT,40,0, true},{100,5,STOP,40,0, true}};
    RoutePlanner p; init_route_once(&p,straight,2,NULL,NULL); route_start(&p,0,0);
    p.state=ROUTE_EXECUTING; p.phase_ms=0;
    for(unsigned i=0;i<3;++i) route_update(&p,mask(0x38),0,0,990+5*i);
    CHECK(p.state==ROUTE_EXECUTING);
    route_update(&p,mask(0x38),0,0,JUNCTION_STRAIGHT_TIMEOUT_MS);
    CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
    /* 真实控制链：执行一秒后退出动作、输出仍启用，离口完成后进入下一项。 */
    const RouteStep steps[]={{1,.3f,LEFT,40,0, true},{100,5,STOP,40,0, true}};
    LineFollow f; EncoderCounts c; uint32_t now=0;
    start(&f,&c,&now,steps,2);
    while(f.odometry.distance_cm-f.route.origin_cm<.85f) tick(&f,&c,&now,3,3,0x18,false);
    for(unsigned i=0;i<5 && f.route.state!=ROUTE_EXECUTING;++i) tick(&f,&c,&now,3,3,0xff,false);
    CHECK(f.route.state==ROUTE_EXECUTING);
    now=f.route.phase_ms+JUNCTION_LINE_EXIT_AFTER_MS-15;
    for(unsigned i=0;i<3;++i) tick(&f,&c,&now,3,3,0x38,false);
    CHECK(f.route.state==ROUTE_CLEARING && f.command.enabled && f.state!=LINE_STOPPED);
    CHECK(!f.stop_snapshot.valid && f.route.turn_ended_on_line);
    for(unsigned i=0;i<120 && f.route.state==ROUTE_CLEARING;++i) tick(&f,&c,&now,3,3,0x38,false);
    CHECK(f.route.index==1 && f.route.state==ROUTE_FOLLOW && f.command.enabled);
}

/* 短段继承规划轴；残余偏航不是反向运动，实际反向仍须停车。 */
static void test_short_segment_heading_reference(void)
{
    for(unsigned mirror=0;mirror<2;++mirror) for(unsigned same=0;same<2;++same) {
        Action first=mirror?RIGHT:LEFT;
        Action next=same?first:(mirror?LEFT:RIGHT);
        float sign=mirror?1:-1;
        RouteStep steps[]={{100,12,first,50,0,false},{20,10,next,50,0,true},
                           {100,10,STOP,50,0,true}};
        RoutePlanner p;init_route_once(&p,steps,3,NULL,NULL);route_start(&p,0,0);
        p.state=ROUTE_CLEARING;
        p.clear_cm=p.projection_previous_cm=100;
        p.projection_previous_heading=sign;
        p.outgoing_cm=19;
        for(unsigned i=0;i<3;++i) route_update(&p,mask(0x18),100.5f+.01f*(float)i,sign,10+i*5);
        CHECK(p.index==1 && p.state==ROUTE_FOLLOW && p.segment_heading_valid);
        NEAR(p.segment_heading_rad,sign*JUNCTION_TURN_TARGET_RAD);
        route_update(&p,mask(0),100.6f,sign,25);
        CHECK(p.state==ROUTE_EXECUTING && p.turn_forced);
        NEAR(p.entry_heading,sign*JUNCTION_TURN_TARGET_RAD);
        NEAR(p.action_start_heading,sign);
        route_update(&p,mask(0),100.65f,sign,30);
        CHECK(p.state==ROUTE_EXECUTING && p.failure==ROUTE_FAILURE_NONE);
        float next_sign=next==RIGHT?1:-1;
        route_update(&p,mask(0),100.7f,sign-next_sign*(JUNCTION_WRONG_WAY_RAD+.01f),35);
        CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_WRONG_WAY);
        route_start(&p,0,40);
        CHECK(!p.segment_heading_valid);
    }
    /* 本次实车冻结数据回放：有效单线不能取消越窗保护。 */
    RouteStep steps[]={{20,10,LEFT,50,0,true},{60,16,STOP,60,0,true}};
    RoutePlanner p;init_route_once(&p,steps,2,NULL,NULL);route_start(&p,344.782257f,12220);
    route_update(&p,mask(0x10),374.834961f,-3.536263f,12740);
    CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_WINDOW);
}

/* 二维 20 cm 反向两连弯，传感器输入来自真实线段几何，轮速理想跟随。 */
static void test_spatial_short_s(void)
{
    for(unsigned mirror=0;mirror<2;++mirror) {
        float sign=mirror?-1:1;
        RouteStep steps[]={{24,12,CUSTOM_TURN,50,sign*90,false},
                           {20,10,mirror?RIGHT:LEFT,50,0,true},{100,10,STOP,50,0,true}};
        OdometryCalibration geometry={4.8f,4.8f,1560,1560,14.5f,-1,1,true};
        LineFollow f; EncoderCounts c={0}; uint32_t now=0;
        init_follow_once(&f,geometry,steps,3,NULL,NULL);
        GraySensor sensor; gray_sensor_init(&sensor);
        float x=0,y=0,h=0,rl=0,rr=0;
        for(unsigned n=0;n<5000 && f.state!=LINE_STOPPED && f.route.index<2;++n) {
            float dl=f.left_target_cm_s*.005f,dr=f.right_target_cm_s*.005f;
            float dh=(dl-dr)/14.5f,ds=(dl+dr)*.5f;
            x+=ds*cosf(h+dh*.5f);y+=ds*sinf(h+dh*.5f);h+=dh;
            float cm=3.14159265359f*4.8f/1560*ODOMETRY_DISTANCE_SCALE;
            rl+=dl/cm;rr+=dr/cm;
            int l=(int)rl,r=(int)rr;rl-=l;rr-=r;
            c.left=(uint16_t)(c.left-l);c.right=(uint16_t)(c.right+r);
            uint8_t line=0;
            for(unsigned j=0;j<8;++j) {
                float side=((float)j-3.5f)*.6f;
                float sx=x+16*cosf(h)-side*sinf(h),sy=sign*(y+16*sinf(h)+side*cosf(h));
                bool inbound=sx<=40.9f && fabsf(sy)<=.9f;
                bool middle=fabsf(sx-40)<=.9f && sy>=-.9f && sy<=20.9f;
                bool outbound=sx>=39.1f && fabsf(sy-20)<=.9f;
                if(inbound||middle||outbound)line|=(uint8_t)(1U<<j);
            }
            GrayObservation obs=gray_sensor_update(&sensor,raw_for_line(line),line_follow_have_recent_position(&f,now),f.reliable_position);
            now+=5;line_follow_update(&f,obs,c,now,.005f,n==LINE_CONFIRM_SAMPLES-1U);
        }
        CHECK(f.route.index==2 && f.route.state==ROUTE_FOLLOW);
        CHECK(f.route.failure==ROUTE_FAILURE_NONE && f.state!=LINE_STOPPED);
        CHECK(fabsf(sign*(y+16*sinf(h))-20)<1.5f);
        CHECK(x+16*cosf(h)>40 && fabsf(h)<.2f);
    }
}

/* 实车复测冻结状态：索引4、30.0836 cm越窗，停车后三帧在线自动续当前项。 */
static void setup_window_stop(LineFollow *f, EncoderCounts *c, uint32_t *now)
{
    static const RouteStep steps[]={{104,16,LEFT,60,0,true},{131,16,LEFT,60,0,true},
        {58,12,LEFT,60,0,true},{55,12,CUSTOM_TURN,50,90,false},
        {20,10,LEFT,50,0,true},{60,16,STOP,60,0,true}};
    start(f,c,now,steps,6);
    f->route.index=4;
    f->route.completed_laps=2;
    f->route.origin_cm=339.6651f;
    f->odometry.distance_cm=369.748688f;
    f->route.segment_heading_rad=-3.4474678f;
    f->route.segment_heading_valid=true;
    f->odometry.heading_rad=-3.776957f;
    f->startup_elapsed_s=STARTUP_RAMP_TIME_S;
    f->base=19;
    tick(f,c,now,3,3,0x10,false);
    CHECK(f->state==LINE_STOPPED && !f->command.enabled);
    CHECK(f->route.failure==ROUTE_FAILURE_WINDOW && f->route.index==4);
    CHECK(f->stop_snapshot.valid && f->stop_snapshot.route_state_before_update==ROUTE_FOLLOW);
}
static void test_route_auto_resume(void)
{
    LineFollow f; EncoderCounts c; uint32_t now=0;
    setup_window_stop(&f,&c,&now);
    LineStopSnapshot stopped=f.stop_snapshot;
    float origin=f.route.origin_cm;
    for(unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) {
        tick(&f,&c,&now,0,0,0x10,false);
        if(i+1<LINE_CONFIRM_SAMPLES || !ROUTE_AUTO_RESUME_ENABLED)
            CHECK(f.state==LINE_STOPPED && !f.command.enabled);
    }
#if ROUTE_AUTO_RESUME_ENABLED
    CHECK(f.state==LINE_TRACKING && f.command.enabled && f.route.index==4);
    CHECK(f.start_result==LINE_START_AUTO_RESUMED && f.auto_resume_count==1);
    CHECK(f.route.completed_laps==2 && f.route.failure==ROUTE_FAILURE_NONE);
    CHECK(f.route.recovery_window_active && !f.stop_snapshot.valid);
    NEAR(f.route.origin_cm,origin);
    CHECK(f.auto_resume_snapshot.sample_ms==stopped.sample_ms);
    CHECK(f.auto_resume_snapshot.route_failure==ROUTE_FAILURE_WINDOW);
    CHECK(f.base<=JUNCTION_APPROACH_SPEED_CM_S*.005f/STARTUP_RAMP_TIME_S+.001f);
    for(unsigned i=0;i<80;++i) {
        tick(&f,&c,&now,3,3,0x10,false);
        CHECK(f.state==LINE_TRACKING && f.route.index==4 && f.auto_resume_count==1);
    }
    /* 原窗口外也可确认当前路口，不跳项，不调用 route_start。 */
    tick(&f,&c,&now,3,3,0x1f,false);
    CHECK(f.route.state==ROUTE_EXECUTING && f.route.index==4 && f.command.enabled);
    NEAR(f.route.entry_heading,-3.4474678f);
    /* 按键停车可以取消自动恢复。 */
    line_follow_stop(&f,LINE_STOP_REQUEST);
    for(unsigned i=0;i<5;++i) tick(&f,&c,&now,0,0,0x18,false);
    CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_REQUEST);
    /* 首帧/两帧/被歧义中断的有效线不能自动启动。 */
    const uint8_t invalid[]={0,0xff,0x7f,0x81,0x98};
    for(unsigned k=0;k<sizeof(invalid);++k) {
        setup_window_stop(&f,&c,&now);
        tick(&f,&c,&now,0,0,0x18,false);
        tick(&f,&c,&now,0,0,0x18,false);
        tick(&f,&c,&now,0,0,invalid[k],false);
        tick(&f,&c,&now,0,0,0x18,false);
        CHECK(f.state==LINE_STOPPED && !f.command.enabled && f.auto_resume_count==0);
    }
    setup_window_stop(&f,&c,&now);
    tick(&f,&c,&now,0,0,0x18,false);
    tick(&f,&c,&now,0,0,0x18,false);
    now+=CONTROL_MAX_INTERVAL_MS;
    tick(&f,&c,&now,0,0,0x18,false);
    CHECK(f.state==LINE_STOPPED && f.valid_count==1);
    /* 最终STOP、丢线、过转/反向、非法路径和故障不得自动重启。 */
    const LineStopReason reasons[]={LINE_STOP_REQUEST,LINE_STOP_ROUTE_COMPLETE,
        LINE_STOP_LOST,LINE_STOP_ENCODER,LINE_STOP_FAULT,LINE_STOP_ROUTE,LINE_STOP_ROUTE,LINE_STOP_ROUTE};
    const RouteFailure failures[]={ROUTE_FAILURE_NONE,ROUTE_FAILURE_NONE,
        ROUTE_FAILURE_NONE,ROUTE_FAILURE_NONE,ROUTE_FAILURE_NONE,
        ROUTE_FAILURE_TURN_LIMIT,ROUTE_FAILURE_WRONG_WAY,ROUTE_FAILURE_INVALID};
    for(unsigned k=0;k<sizeof(reasons)/sizeof(reasons[0]);++k) {
        setup_window_stop(&f,&c,&now);
        f.stop_reason=reasons[k];f.route.failure=failures[k];
        for(unsigned i=0;i<5;++i) tick(&f,&c,&now,0,0,0x18,false);
        CHECK(f.state==LINE_STOPPED && !f.command.enabled && f.auto_resume_count==0);
    }
    /* 新的窗口仍有上界；再次停车锁存新证据，不能被上一次快照遮住。 */
    setup_window_stop(&f,&c,&now);
    for(unsigned i=0;i<3;++i) tick(&f,&c,&now,0,0,0x18,false);
    f.odometry.distance_cm=f.route.recovery_window_origin_cm+20.01f;
    tick(&f,&c,&now,0,0,0x18,false);
    CHECK(f.state==LINE_STOPPED && f.stop_snapshot.valid);
    CHECK(f.stop_snapshot.sample_ms>f.auto_resume_snapshot.sample_ms);
    CHECK(f.stop_snapshot.route_failure==ROUTE_FAILURE_WINDOW);
    /* 人工START仍明确从第0项开始，并清理恢复窗口与诊断会话。 */
    for(unsigned i=0;i<3;++i) tick(&f,&c,&now,0,0,0x18,i==2);
    CHECK(f.route.index==0 && f.route.completed_laps==0 && f.auto_resume_count==0);
    CHECK(!f.route.recovery_window_active && !f.auto_resume_snapshot.valid);
#else
    (void)stopped;(void)origin;
    CHECK(f.route.index==4 && f.stop_snapshot.valid && f.auto_resume_count==0);
#endif
}
static void test_route_resume_phases(void)
{
    const RouteStep steps[]={{10,2,LEFT,20,0,false},{100,5,STOP,20,0,false}};
    for(unsigned phase=ROUTE_FOLLOW;phase<=ROUTE_CLEARING;++phase) {
        RoutePlanner p;init_route_once(&p,steps,2,NULL,NULL);route_start(&p,0,0);
        p.state=ROUTE_FAILED;
        p.failure=phase==ROUTE_FOLLOW?ROUTE_FAILURE_WINDOW:
            phase==ROUTE_CANDIDATE?ROUTE_FAILURE_CONFIRM_TIMEOUT:ROUTE_FAILURE_ACTION_TIMEOUT;
        p.projection_previous_cm=15;p.completed_laps=3;
        CHECK(route_resume(&p,(RouteState)phase,16,0,100));
        CHECK(p.index==0 && p.completed_laps==3 && p.failure==ROUTE_FAILURE_NONE);
        if(phase<=ROUTE_CANDIDATE) {
            CHECK(p.state==ROUTE_FOLLOW && p.recovery_window_active);
            NEAR(p.origin_cm,0);NEAR(p.recovery_window_origin_cm,16);
        } else {
            CHECK(p.state==ROUTE_CLEARING && p.turn_ended_on_line);
            NEAR(p.clear_cm,16);CHECK(p.phase_ms==100);
            route_update(&p,mask(0x18),16,0,100+JUNCTION_CLEAR_TIMEOUT_MS);
            CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
        }
    }
}

int main(void)
{
    test_route_auto_resume(); test_route_resume_phases(); test_short_segment_heading_reference(); test_spatial_short_s(); test_offset_odometry(); test_corner_mode_switch(); test_corner_kinematics(); test_turn_online_exit();
    test_custom_turn_heading_limit(); test_custom_exit_angle_hysteresis(); test_custom_capture_recentering(); test_branch_exit_capture(); test_short_segment_handoff(); test_custom_turns(); test_odometry(); test_measured_distance_scale(); test_speed(); test_left_encoder_direction(); test_routes(); test_controller(); test_full_route_pipeline(); test_post_turn_stop_snapshot(); test_raw_pipeline();
    test_startup_ramp(); test_junction_approach_speed(); test_early_exit_acceleration(); test_long_straight_without_timeout(); test_reference_path(); test_route_repeats(); test_loop_end_corner(); test_corner_distance_projection(); test_capture_cancellation(); test_near_angle_exit(); test_exit_evidence_boundaries(); test_relaxed_exit_acceleration(); test_late_turn_clear_budget(); test_turn_capture(); test_l_corner_confirmation(); test_route_abrupt_priority(); test_spatial_corner();
    printf("PASS: odometry, nested loops, junction FSM, interference and route STOP; %u checks\n",checks);
    return 0;
}
