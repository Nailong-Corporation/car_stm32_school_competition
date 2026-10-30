/* Reuse navigation fixtures; this main exercises the independent switch matrix. */
#define main navigation_regression_main
#include "test_navigation.c"
#undef main

int main(void)
{
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
