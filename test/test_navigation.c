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
        float heading=a==LEFT ? -1.2f : a==RIGHT ? 1.2f : 0;
        float distance=a==STRAIGHT ? 12.3f : 10.0f;
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
    line_follow_init(&f,calibration(),long_route,1,NULL,NULL);
    for(unsigned i=0;i<10;++i) tick(&f,&c,&now,0,0,0,true);
    CHECK(f.state==LINE_IDLE);
    for(unsigned i=0;i<5;++i) tick(&f,&c,&now,0,0,0x18,false);
    CHECK(f.state==LINE_IDLE); /* no queued start */
    start(&f,&c,&now,long_route,1);
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
    while(f.odometry.heading_rad-angle> -1.15f && f.state!=LINE_STOPPED)
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
        GrayObservation obs=gray_sensor_update(&sensor,0xe7,
            line_follow_have_recent_position(&f,now),f.reliable_position);
        line_follow_update(&f,obs,c,now,.005f,i==11);
    }
    CHECK(f.state==LINE_TRACKING);
    for(unsigned i=0;i<5;++i) {
        now+=5; c.left+=3; c.right+=3;
        GrayObservation obs=gray_sensor_update(&sensor,(uint8_t)~0x19U,
            line_follow_have_recent_position(&f,now),f.reliable_position);
        line_follow_update(&f,obs,c,now,.005f,false);
    }
    CHECK(f.state==LINE_GAP_HOLD && f.route.index==0);
    for(unsigned i=0;i<5;++i) {
        now+=5; c.left+=3; c.right+=3;
        GrayObservation obs=gray_sensor_update(&sensor,0xe7,
            line_follow_have_recent_position(&f,now),f.reliable_position);
        line_follow_update(&f,obs,c,now,.005f,false);
    }
    CHECK(f.state==LINE_TRACKING && !f.abnormal);
}
int main(void)
{
    test_odometry(); test_speed(); test_routes(); test_controller(); test_full_route_pipeline(); test_raw_pipeline();
    printf("PASS: odometry, nested loops, junction FSM, interference and route STOP; %u checks\n",checks);
    return 0;
}
