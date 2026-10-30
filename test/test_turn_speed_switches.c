/* Reuse navigation fixtures; this main exercises the independent switch matrix. */
#define main navigation_regression_main
#include "test_navigation.c"
#undef main

int main(void)
{
    /* User confirmed162 cm is a branch to skip; only190 cm is the target. */
    {
        RouteStep steps[]={{190,16,LEFT,200,0,false},{65,12,STOP,200,0,false}};
        const uint32_t times[]={14532,14557,14562,14572,14577,14882,15317,15337,15357};
        const float progress[]={159.995102f,161.832336f,162.163467f,162.804367f,
            163.114136f,174.094360f,182.201355f,182.532471f,182.788818f};
        const uint8_t masks[]={0x18,0x1a,0x1f,0x18,0,0,8,0x20,0};
        RoutePlanner p; init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,times[0]);
        for (unsigned i=0;i<sizeof(times)/sizeof(times[0]);++i) {
            uint32_t end=i+1U<sizeof(times)/sizeof(times[0]) ? times[i+1U] : times[i]+5;
            for(uint32_t t=times[i];t<end;t+=5) {
                float fraction=(float)(t-times[i])/(float)(end-times[i]);
                float cm=progress[i]+(i+1U<sizeof(times)/sizeof(times[0]) ?
                    fraction*(progress[i+1U]-progress[i]) : 0);
                route_update(&p,mask(masks[i]),cm,0,t);
            }
        }
        CHECK(p.state==ROUTE_FOLLOW && !p.turn_forced);
        for(unsigned i=0;i<3;++i) route_update(&p,mask(0x1f),190+.1f*i,0,15362+5*i);
        CHECK(p.state==ROUTE_EXECUTING); /* Real target still confirms. */
        for(unsigned stale=0;stale<3;++stale) {
            init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0);
            route_update(&p,mask(0x1f),190,0,5);
            route_update(&p,mask(0x1f),190.05f,0,10);
            route_update(&p,mask(0x18),190.1f,0,15);
            uint32_t ms=20; float cm=190.15f;
            if(stale==1) {
                ms=15+JUNCTION_LOSS_BRANCH_MAX_MS;
                for(uint32_t t=20;t<ms;t+=5) route_update(&p,mask(0x18),190.1f,0,t);
            }
            if(stale==2) cm=190.05f+JUNCTION_LOSS_BRANCH_MAX_CM+.01f;
            route_update(&p,mask(0),cm,0,ms);
            CHECK(stale==0 ? p.state==ROUTE_EXECUTING && p.turn_forced : p.state==ROUTE_FOLLOW);
        }
        for(unsigned mirror=0;mirror<2;++mirror) {
            steps[0].action=mirror?RIGHT:LEFT;
            init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0);
            for(unsigned i=0;i<3;++i) route_update(&p,mask(0x18),190,0,5+5*i);
            route_update(&p,mask(mirror?0x1f:0xf8),190.05f,0,20);
            route_update(&p,mask(0),190.1f,0,25);
            CHECK(p.state==ROUTE_FOLLOW && !p.loss_branch_ready);
            init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0);
            route_update(&p,mask(mirror?0xf8:0x1f),190,0,5);
            route_update(&p,mask(mirror?0xf8:0x1f),190.05f,0,10);
            CHECK(p.loss_branch_ready);
            route_update(&p,mask(mirror?0x1f:0xf8),190.1f,0,15);
            route_update(&p,mask(0),190.15f,0,20);
            CHECK(p.state==ROUTE_FOLLOW && !p.loss_branch_ready);
        }
    }
    {
        RouteStep steps[]={{190,16,LEFT,200,0,false},{65,12,STOP,200,0,false}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2); f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
        f.base=f.left_target_cm_s=f.right_target_cm_s=200;
        f.route.origin_cm=f.odometry.distance_cm-174;
        tick(&f,&c,&now,0,0,0x18,false);
#if TURN_APPROACH_SLOWDOWN_ENABLED
        NEAR(f.base,JUNCTION_APPROACH_SPEED_CM_S);
#else
        NEAR(f.base,200);
#endif
    }
    /* Real entry trace: left branch at55.35 cm, gone at57.53 cm.
     * The old60 cm gate cannot see it; the revised53 cm gate must accept it. */
    for (unsigned revised=0;revised<2;++revised) {
        RouteStep steps[]={{65,revised?12:5,LEFT,200,0,true},{219,16,STOP,200,0,true}};
        RoutePlanner p;
        init_route_once(&p,steps,2,NULL,NULL); route_start(&p,0,0);
        route_update(&p,mask(0x18),54,0,5);
        route_update(&p,mask(0x18),54.5f,0,10);
        route_update(&p,mask(0x18),55,0,15);
        route_update(&p,mask(0x1f),55.35f,0,20);
        CHECK(p.state==(revised?ROUTE_EXECUTING:ROUTE_FOLLOW));
        if (!revised) {
            route_update(&p,mask(0),57.53f,0,25);
            route_update(&p,mask(0),60.07f,0,30);
            CHECK(p.state==ROUTE_FOLLOW);
            route_update(&p,mask(0),70.07f,0,35);
            CHECK(p.failure==ROUTE_FAILURE_WINDOW);
        }
    }
    {
        RouteStep steps[]={{65,12,LEFT,200,0,true},{219,16,STOP,200,0,true}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
        const float positions[]={20,40,52,53};
        for (unsigned i=0;i<sizeof(positions)/sizeof(positions[0]);++i) {
            f.base=f.left_target_cm_s=f.right_target_cm_s=200;
            f.route.origin_cm=f.odometry.distance_cm-positions[i];
            tick(&f,&c,&now,0,0,0x18,false);
#if TURN_APPROACH_SLOWDOWN_ENABLED
            float expected=sqrtf(JUNCTION_APPROACH_SPEED_CM_S*JUNCTION_APPROACH_SPEED_CM_S+
                2*JUNCTION_APPROACH_DECEL_CM_S2*(53-positions[i]));
            NEAR(f.base,expected);
            if(i==3) NEAR(f.base,JUNCTION_APPROACH_SPEED_CM_S);
#else
            NEAR(f.base,200);
#endif
        }
    }
    test_steering_with_saturated_outer_feedback();
    test_exit_requires_stable_center_and_distance_bounds();
    test_spatial_exit_recapture();
    test_encoder_tracking_pid_at_maximum_speed();
    SpeedControl s; MotorCommand out;
    speed_control_init(&s);
    CHECK(speed_control_update(&s,20,20,80,20,.005f,&out));
#if MOTOR_REVERSE_BRAKE_ENABLED
    NEAR(out.left,-REVERSE_BRAKE_MAX_PWM);
#else
    CHECK(out.left>=0);
#endif
    CHECK(speed_control_update(&s,20,20,24,20,.005f,&out));
#if MOTOR_REVERSE_BRAKE_ENABLED
    NEAR(out.left,-4);
#else
    CHECK(out.left>=0);
#endif
    CHECK(speed_control_update(&s,20,20,22,20,.005f,&out));
    CHECK(out.left>=0);
    CHECK(speed_control_update(&s,0,0,80,80,.005f,&out));
    CHECK(!out.enabled && out.left==0 && out.right==0);
    CHECK(!speed_control_update(&s,20,20,-2,20,.005f,&out));
    CHECK(s.fault==SPEED_FAULT_REVERSE && !out.enabled);
    CHECK(!speed_control_update(&s,-1,20,20,20,.005f,&out));
    CHECK(!out.enabled);

    for (unsigned arc=0;arc<2;++arc) for (unsigned mirror=0;mirror<2;++mirror) {
        RouteStep steps[]={{100,8,mirror?RIGHT:LEFT,50,0,arc!=0},{100,8,STOP,80,0,false}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
        f.base=f.left_target_cm_s=f.right_target_cm_s=50;
        f.route.origin_cm=f.odometry.distance_cm-99;
        tick(&f,&c,&now,0,0,0x18,false);
#if TURN_APPROACH_SLOWDOWN_ENABLED
        CHECK(f.base<50 && f.base>=JUNCTION_APPROACH_SPEED_CM_S);
#else
        NEAR(f.base,50);
#endif
        f.route.state=ROUTE_EXECUTING;
        f.route.phase_ms=now;
        f.route.entry_cm=f.route.projection_previous_cm=f.odometry.distance_cm;
        f.route.entry_heading=f.odometry.heading_rad;
        tick(&f,&c,&now,0,0,0,false);
#if TURN_APPROACH_SLOWDOWN_ENABLED
        CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=JUNCTION_APPROACH_SPEED_CM_S+.001f || arc);
#else
        if (!arc) NEAR(fmaxf(f.left_target_cm_s,f.right_target_cm_s),50);
#endif
        /* Confirmed exit starts acceleration immediately, before route index advances. */
        f.route.state=ROUTE_CLEARING;
        f.route.phase_ms=now;
        f.route.outgoing_cm=0;
        f.route.clear_cm=f.odometry.distance_cm;
        f.route.entry_heading=f.odometry.heading_rad-(mirror?1:-1)*JUNCTION_TURN_TARGET_RAD;
        f.base=f.left_target_cm_s=f.right_target_cm_s=10;
        tick(&f,&c,&now,0,0,0x18,false);
#if TURN_EXIT_ACCEL_ENABLED
        NEAR(fmaxf(f.left_target_cm_s,f.right_target_cm_s),10+TURN_EXIT_ACCEL_CM_S2*.005f);
#else
        CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=10+TARGET_RISE_CM_S2*.005f+.001f);
#endif
    }
    /* Fast acceleration continues into FOLLOW after index handoff, then stops at target. */
    {
        RouteStep steps[]={{100,8,LEFT,80,0,false},{100,8,STOP,80,0,false}};
        LineFollow f; EncoderCounts c; uint32_t now=0;
        start(&f,&c,&now,steps,2);
        f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
        f.exit_accelerating=TURN_EXIT_ACCEL_ENABLED;
        f.base=f.left_target_cm_s=f.right_target_cm_s=10;
        tick(&f,&c,&now,0,0,0x18,false);
        NEAR(f.base,10+(TURN_EXIT_ACCEL_ENABLED?TURN_EXIT_ACCEL_CM_S2:TARGET_RISE_CM_S2)*.005f);
        f.base=f.left_target_cm_s=f.right_target_cm_s=79;
        tick(&f,&c,&now,0,0,0x18,false);
#if TURN_EXIT_ACCEL_ENABLED
        NEAR(f.base,80);
        CHECK(!f.exit_accelerating);
#endif
        f.exit_accelerating=true;
        line_follow_stop(&f,LINE_STOP_REQUEST);
        CHECK(!f.exit_accelerating && !f.command.enabled);
    }
    /* Explicit STRAIGHT retains table speed even at nominal distance. */
    RouteStep straight[]={{100,8,STRAIGHT,50,0,false},{100,8,STOP,80,0,false}};
    LineFollow f; EncoderCounts c; uint32_t now=0;
    start(&f,&c,&now,straight,2);
    f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
    f.base=f.left_target_cm_s=f.right_target_cm_s=50;
    f.route.origin_cm=f.odometry.distance_cm-100;
    tick(&f,&c,&now,0,0,0x18,false);
    NEAR(f.base,50);
    printf("PASS: turn approach=%d exit=%d reverse brake=%d (%u checks)\n",
        TURN_APPROACH_SLOWDOWN_ENABLED,TURN_EXIT_ACCEL_ENABLED,MOTOR_REVERSE_BRAKE_ENABLED,checks);
    return 0;
}
