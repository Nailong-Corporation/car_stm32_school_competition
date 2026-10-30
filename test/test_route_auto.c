/* Reuse the encoder/controller fixtures and keep the full navigation main available. */
#define main navigation_regression_main
#include "test_navigation.c"
#undef main
#include <string.h>

static void test_auto_detection(void)
{
    const float tolerances[] = {0, -100, 100000, NAN, INFINITY};
    for (unsigned t=0;t<sizeof(tolerances)/sizeof(tolerances[0]);++t) {
        for (unsigned mirror=0;mirror<2;++mirror) {
            RouteStep steps[]={{AUTO,tolerances[t],mirror?RIGHT:LEFT,50,0,false},
                               {AUTO,tolerances[t],STOP,50,0,false}};
            RoutePlanner p;
            init_route_once(&p,steps,2,NULL,NULL);
            CHECK(p.valid);
            route_start(&p,0,0);
            /* No distance ceiling, no spontaneous turn on narrow line or line loss. */
            for (unsigned i=1;i<=5;++i) route_update(&p,mask(0x18),10000+i,0,i*5);
            CHECK(p.state==ROUTE_FOLLOW);
            route_update(&p,mask(0),10006,0,30);
            CHECK(p.state==ROUTE_FOLLOW);
            route_update(&p,mask(mirror?0x0f:0xf0),10007,0,35);
            CHECK(p.state==ROUTE_FOLLOW); /* Opposite branch. */
            route_update(&p,mask(mirror?0xf8:0x1f),10008,0,40);
            CHECK(p.state==ROUTE_EXECUTING && !p.turn_forced);
            NEAR(p.entry_cm,10008);
        }
    }
    /* Every action can use AUTO; final STOP still dispatches only once. */
    const Action actions[]={LEFT,RIGHT,CUSTOM_TURN,STRAIGHT,STOP};
    for (unsigned i=0;i<sizeof(actions)/sizeof(actions[0]);++i) {
        RouteStep steps[]={{AUTO,0,actions[i],50,-60,false},{100,5,STOP,50,0,false}};
        RoutePlanner p;
        init_route_once(&p,steps,actions[i]==STOP?1:2,NULL,NULL);
        CHECK(p.valid);
        route_start(&p,0,0);
        route_update(&p,mask(0x18),0,0,5);
        CHECK(p.state==ROUTE_FOLLOW);
        route_update(&p,mask(0xff),2,0,10);
        CHECK(p.state==(actions[i]==STOP?ROUTE_FINISHED:ROUTE_EXECUTING));
        NEAR(p.entry_cm,actions[i]==STOP?0:2);
        if (actions[i]==STOP) {
            CHECK(p.stop_pending && p.completed_laps==1);
            route_update(&p,mask(0xff),3,0,15);
            CHECK(p.completed_laps==1);
        }
    }
    RouteStep repeat[]={{AUTO,0,STOP,50,0,false}};
    RoutePlanner p;
    route_init(&p,repeat,1,NULL,NULL); p.repeat_count=2;
    route_start(&p,0,0);
    route_update(&p,mask(0xff),0,0,5);
    CHECK(p.state==ROUTE_EXECUTING && p.lap_boundary && p.completed_laps==1);

    /* The numeric mode keeps its distance and tolerance validation/window. */
    RouteStep numeric[]={{100,5,LEFT,50,0,false},{100,5,STOP,50,0,false}};
    init_route_once(&p,numeric,2,NULL,NULL); CHECK(p.valid);
    route_start(&p,0,0);
    route_update(&p,mask(0xff),1,0,5); CHECK(p.state==ROUTE_FOLLOW);
    route_update(&p,mask(0x18),106,0,10);
    CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_WINDOW);
    numeric[0].tolerance=NAN;
    init_route_once(&p,numeric,2,NULL,NULL); CHECK(!p.valid);
    numeric[0].distance=-2; numeric[0].tolerance=0;
    init_route_once(&p,numeric,2,NULL,NULL); CHECK(!p.valid);
}

static void test_auto_speed_and_handoff(void)
{
    for (unsigned arc=0;arc<2;++arc) for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{AUTO,NAN,mirror?RIGHT:LEFT,50,0,arc!=0},
                           {AUTO,NAN,STOP,80,0,false}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
        f.base=f.left_target_cm_s=f.right_target_cm_s=50;
        f.route.origin_cm=f.odometry.distance_cm-10000;
        tick(&f,&c,&now,0,0,0x18,false);
        CHECK(f.route.state==ROUTE_FOLLOW); NEAR(f.base,50);
        tick(&f,&c,&now,0,0,mirror?0xf8:0x1f,false);
        CHECK(f.route.state==ROUTE_EXECUTING && f.state==LINE_JUNCTION);
        CHECK(f.entry_trace.frozen && f.entry_trace.count>0);
        CHECK(f.base<=JUNCTION_APPROACH_SPEED_CM_S+.001f);
        if (!arc) CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=JUNCTION_APPROACH_SPEED_CM_S+.001f);
        /* CLEARING cannot mistake AUTO=-1 for an already overrun next window,
         * and must perform normal departure debounce rather than early handoff. */
        f.route.state=ROUTE_CLEARING; f.route.phase_ms=now;
        f.route.entry_heading=f.odometry.heading_rad-(mirror?1:-1)*JUNCTION_TURN_TARGET_RAD;
        f.route.clear_cm=f.odometry.distance_cm;
        f.route.projection_previous_cm=f.odometry.distance_cm;
        f.route.projection_previous_heading=f.odometry.heading_rad;
        f.route.outgoing_cm=100;
        tick(&f,&c,&now,0,0,0x18,false);
        CHECK(f.route.state==ROUTE_CLEARING && f.route.index==0);
        f.route.clear_cm=f.odometry.distance_cm-JUNCTION_CLEAR_DISTANCE_CM;
        f.route.clear_center_pending=true;
        f.route.clear_center_ms=now-JUNCTION_CLEAR_CENTER_MS;
        f.route.clear_center_cm=f.odometry.distance_cm-JUNCTION_CLEAR_CENTER_CM;
        f.route.exit_count=LINE_CONFIRM_SAMPLES;
        tick(&f,&c,&now,0,0,0x18,false);
        CHECK(f.route.state==ROUTE_FOLLOW && f.route.index==1);
        CHECK(!f.entry_trace.frozen && f.entry_trace.count==0 && f.entry_trace.event_count==0);
        CHECK(!f.route.handoff_early && !f.route.segment_heading_valid);
        tick(&f,&c,&now,0,0,0x18,false);
        CHECK(f.route.state==ROUTE_FOLLOW);
        CHECK(f.entry_trace.count==1 && f.entry_trace.samples[0].index==1);
    }
    /* AUTO STRAIGHT follows the line before detection; its reference is the
     * detected junction, and a following AUTO does not inherit a distance gate. */
    RouteStep steps[]={{AUTO,INFINITY,STRAIGHT,50,0,false},{AUTO,0,STOP,50,0,false}};
    LineFollow f; EncoderCounts c; uint32_t now=0;
    start(&f,&c,&now,steps,2);
    f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
    tick(&f,&c,&now,0,0,0x30,false);
    CHECK(f.route.state==ROUTE_FOLLOW && f.pid.primed);
    CHECK(f.left_target_cm_s>f.right_target_cm_s);
    tick(&f,&c,&now,0,0,0xff,false);
    CHECK(f.route.state==ROUTE_EXECUTING);
    NEAR(f.route.entry_cm,f.odometry.distance_cm);
    f.route.index=1;
    CHECK(!route_waiting_after_straight(&f.route,f.odometry.distance_cm));
}

static void test_mixed_handoff(void)
{
    for (unsigned first_auto=0;first_auto<2;++first_auto) {
        RouteStep steps[]={{first_auto?AUTO:100,5,LEFT,50,0,true},
                           {first_auto?100:AUTO,first_auto?5:NAN,STOP,50,0,false}};
        RoutePlanner p;
        init_route_once(&p,steps,2,NULL,NULL); CHECK(p.valid);
        route_start(&p,0,0);
        p.state=ROUTE_CLEARING; p.phase_ms=100;
        p.clear_cm=20-JUNCTION_CLEAR_DISTANCE_CM;
        p.entry_heading=0;
        p.projection_previous_cm=20;
        p.projection_previous_heading=-JUNCTION_TURN_TARGET_RAD;
        p.outgoing_cm=10; p.exit_count=LINE_CONFIRM_SAMPLES;
        route_update(&p,mask(0x18),20,-JUNCTION_TURN_TARGET_RAD,105);
        CHECK(p.index==1 && p.state==ROUTE_FOLLOW);
        NEAR(p.origin_cm,10); NEAR(p.segment_start_progress_cm,10);
        route_update(&p,mask(0xff),21,-JUNCTION_TURN_TARGET_RAD,110);
        if (first_auto) {
            CHECK(p.state==ROUTE_FOLLOW); /* Numeric next item still needs its window. */
            route_update(&p,mask(0xff),110,-JUNCTION_TURN_TARGET_RAD,115);
            route_update(&p,mask(0xff),110.1f,-JUNCTION_TURN_TARGET_RAD,120);
            route_update(&p,mask(0xff),110.2f,-JUNCTION_TURN_TARGET_RAD,125);
        }
        CHECK(p.state==ROUTE_FINISHED && p.stop_pending);
    }
    RouteStep straight[]={{AUTO,0,STRAIGHT,50,0,false},{AUTO,NAN,STOP,50,0,false}};
    RoutePlanner p;
    init_route_once(&p,straight,2,NULL,NULL); route_start(&p,0,0);
    route_update(&p,mask(0xff),10,0,5);
    CHECK(p.state==ROUTE_EXECUTING); NEAR(p.entry_cm,10);
    route_update(&p,mask(0x18),10,0,10);
    CHECK(p.index==0);
    route_update(&p,mask(0x18),13.1f,0,15);
    route_update(&p,mask(0x18),13.2f,0,20);
    route_update(&p,mask(0x18),13.3f,0,25);
    route_update(&p,mask(0x18),16.4f,0,30);
    CHECK(p.state==ROUTE_FOLLOW && p.index==1);
}

static void test_numeric_arc_immediate_entry(void)
{
    for (unsigned mirror=0; mirror<2; ++mirror) {
        RouteStep steps[]={{65,5,mirror?RIGHT:LEFT,200,0,true},
                           {219,16,STOP,200,0,true}};
        RoutePlanner p;
        init_route_once(&p,steps,2,NULL,NULL); CHECK(p.valid);
        route_start(&p,0,0);
        GrayObservation branch=mask(mirror?0xf8:0x1f);
        route_update(&p,branch,59.99f,0,5);
        CHECK(p.state==ROUTE_FOLLOW);
        /* No preceding narrow-line run: this is shape entry, not abrupt entry.
         * The first matching frame at the lower window bound must turn. */
        route_update(&p,branch,60,0,10);
        CHECK(p.state==ROUTE_EXECUTING && !p.turn_forced);
        NEAR(p.entry_cm,60);
        route_start(&p,0,0);
        route_update(&p,mask(mirror?0x0f:0xf0),65,0,5);
        CHECK(p.state==ROUTE_FOLLOW); /* Wrong side still rejected. */
        route_update(&p,mask(0),65.1f,0,10);
        CHECK(p.state==ROUTE_FOLLOW); /* No shape or narrow history. */
        route_start(&p,0,0);
        route_update(&p,branch,70.01f,0,5);
        CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_WINDOW);
        steps[0].corner_arc_enabled=false;
        init_route_once(&p,steps,2,NULL,NULL);
        route_start(&p,0,0);
        route_update(&p,branch,65,0,5);
        CHECK(p.state==ROUTE_CANDIDATE); /* Ordinary turns retain confirmation. */
    }
}

static void test_missed_entry_trace(void)
{
    RouteStep steps[]={{65,5,LEFT,200,0,true},{219,16,STOP,200,0,true}};
    LineFollow f; EncoderCounts c; uint32_t now=0;
    start(&f,&c,&now,steps,2);
    f.route.origin_cm=f.odometry.distance_cm-58;
    /* Lose the line before the distance gate: opening the gate later must
     * not invent a new abrupt edge. Preserve the actual sequence instead. */
    for (unsigned i=0;i<300;++i) tick(&f,&c,&now,2,2,i<5?0x18:0,false);
    CHECK(f.route.state==ROUTE_FOLLOW && !f.entry_trace.frozen);
    CHECK(f.entry_trace.count==ENTRY_TRACE_CAPACITY);
    bool saw_loss=false, saw_window=false;
    for (unsigned i=0;i<f.entry_trace.event_count;++i) {
        EntryTraceSample e=f.entry_trace.events[i];
        if(e.mask==0 && e.narrow_before>=LINE_CONFIRM_SAMPLES && e.window_band==0) {
            saw_loss=true;
            CHECK(e.progress_cm<60);
            CHECK(now-e.ms>128U*5U); /* Missing from the fixed-time ring now. */
        }
        if(e.mask==0 && e.window_band==1) saw_window=true;
    }
    CHECK(saw_loss && saw_window);
    f.route.origin_cm=f.odometry.distance_cm-70.01f;
    tick(&f,&c,&now,5,5,0,false);
    CHECK(f.state==LINE_STOPPED && f.route.failure==ROUTE_FAILURE_WINDOW);
    CHECK(f.entry_trace.frozen);
    EntryTrace saved=f.entry_trace;
    unsigned last=(saved.next+ENTRY_TRACE_CAPACITY-1U)%ENTRY_TRACE_CAPACITY;
    CHECK(saved.samples[last].state_after==ROUTE_FAILED);
    CHECK(saved.samples[last].progress_cm>70 && saved.samples[last].mask==0);
    for (unsigned i=0;i<LINE_CONFIRM_SAMPLES+1U;++i)
        tick(&f,&c,&now,0,0,0x18,false);
    CHECK(f.auto_resume_count==1);
    CHECK(memcmp(&saved,&f.entry_trace,sizeof(saved))==0);
    line_follow_stop(&f,LINE_STOP_REQUEST); /* main.c applies button stop directly. */
    for (unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i)
        tick(&f,&c,&now,0,0,0x18,i==LINE_CONFIRM_SAMPLES-1U);
    CHECK(f.start_result==LINE_START_ACCEPTED);
    CHECK(!f.entry_trace.frozen && f.entry_trace.count<ENTRY_TRACE_CAPACITY);
}

int main(void)
{
    test_missed_entry_trace();
    test_numeric_arc_immediate_entry();
    test_auto_detection();
    test_auto_speed_and_handoff();
    test_mixed_handoff();
    printf("AUTO route passed: invert=%d approach=%d (%u checks)\n",
           GRAY_INVERT_ENABLED,TURN_APPROACH_SLOWDOWN_ENABLED,checks);
    return 0;
}
