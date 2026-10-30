#include "gray_sensor.h"
#include "line_follow.h"
#include "motion_config.h"
#include "tilt_guard.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* Build only with MPU6050_ENABLED=1. Encoder wheel difference must never supply
 * yaw to route, pose, heading hold or turn gates when MPU mode is selected. */
int main(void)
{
    OdometryCalibration geometry = {4.8f,4.8f,1560,1560,14.5f,-1,1,true};
    Odometry odo;
    odometry_init(&odo, geometry);
    EncoderCounts count = {30000,30000};
    assert(odometry_update_with_heading(&odo,count,.005f,true,0.0f));
    count.left -= 20;
    assert(odometry_update_with_heading(&odo,count,.005f,true,0.2f));
    assert(fabsf(odo.heading_rad-0.2f)<.000001f);
    assert(odo.distance_cm>0 && odo.y_cm>0);
    assert(fabsf(odo.sensor_x_cm-odo.x_cm-GRAY_FORWARD_OFFSET_CM*cosf(0.2f))<.00001f);
    assert(fabsf(odo.sensor_y_cm-odo.y_cm-GRAY_FORWARD_OFFSET_CM*sinf(0.2f))<.00001f);
    float old_y = odo.y_cm;
    count.left -= 20;
    assert(odometry_update_with_heading(&odo,count,.005f,true,0.2f));
    assert(fabsf(odo.heading_rad-0.2f)<.000001f && odo.y_cm>old_y);
    assert(!odometry_update_with_heading(&odo,count,.005f,true,NAN));

    RouteStep steps[] = {{200,15,RIGHT,120,0,false},{100,10,STOP,40,0,false}};
    LineFollow f;
    line_follow_init(&f,geometry,steps,2,NULL,NULL);
    GrayObservation center = gray_classify(0x18,true,0);
    GrayObservation blank = gray_classify(0,true,0);
    count = (EncoderCounts){30000,30000};
    uint32_t now = 0;
    for (unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) {
        now += 5;
        line_follow_update_heading(&f,center,count,now,.005f,
                                   i==LINE_CONFIRM_SAMPLES-1U,true,0.0f);
    }
    assert(f.state==LINE_TRACKING && f.start_result==LINE_START_ACCEPTED);
    f.route.state=ROUTE_EXECUTING;
    f.route.phase_ms=now;
    f.route.entry_heading=f.route.action_start_heading=0;
    f.route.entry_cm=f.route.projection_previous_cm=f.odometry.distance_cm;
    f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
    f.base=f.left_target_cm_s=f.right_target_cm_s=120;
    for (unsigned i=0;i<50;++i) {
        count.left -= 20; /* Encoder suggests turning right, IMU reports no turn. */
        now += 5;
        line_follow_update_heading(&f,blank,count,now,.005f,false,true,0.0f);
        assert(f.state!=LINE_STOPPED);
        assert(f.route.state==ROUTE_EXECUTING && fabsf(f.odometry.heading_rad)<.000001f);
    }
    for (unsigned i=0;i<5 && f.route.state==ROUTE_EXECUTING;++i) {
        now += 5;
        line_follow_update_heading(&f,center,count,now,.005f,false,true,1.60f);
    }
    assert(f.route.state==ROUTE_CLEARING && fabsf(f.odometry.heading_rad-1.60f)<.000001f);
    now += 5;
    line_follow_update_heading(&f,center,count,now,.005f,false,false,1.60f);
    assert(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_IMU &&
           f.stop_snapshot.valid && !f.command.enabled);
    now += 5;
    line_follow_update_heading(&f,center,count,now,.005f,true,true,1.60f);
    assert(f.state==LINE_STOPPED && f.start_result==LINE_START_REJECTED_FAULT);

    LineFollow tilted;
    line_follow_init(&tilted,geometry,steps,2,NULL,NULL);
    count=(EncoderCounts){30000,30000};
    now=0;
    for(unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) {
        now+=5;
        line_follow_update_heading(&tilted,center,count,now,.005f,
                i==LINE_CONFIRM_SAMPLES-1U,true,0.0f);
    }
    assert(tilted.state==LINE_TRACKING && tilted.command.enabled);
    line_follow_stop(&tilted,LINE_STOP_TILT);
    assert(tilted.state==LINE_STOPPED && tilted.stop_reason==LINE_STOP_TILT &&
           tilted.stop_snapshot.reason==LINE_STOP_TILT && !tilted.command.enabled);
    now+=5;
    line_follow_update_heading(&tilted,center,count,now,.005f,true,true,0.0f);
    assert(tilted.state==LINE_STOPPED && tilted.start_result==LINE_START_REJECTED_FAULT);

    TiltGuard guard = {0};
    for (unsigned i=0;i<100;++i) assert(tilt_guard_update(&guard,.9f,0,-.6f,.005f));
    assert(guard.tripped && !tilt_guard_acknowledge(&guard));
    for (unsigned i=0;i<100;++i) assert(tilt_guard_update(&guard,0,0,-1,.005f));
    assert(guard.tripped && guard.can_restart); /* No automatic restart after level. */
    for (unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) {
        now+=5;
        line_follow_update_heading(&tilted,center,count,now,.005f,false,true,0);
    }
    assert(tilted.state==LINE_STOPPED && !tilted.command.enabled);
    assert(tilt_guard_acknowledge(&guard)); /* Same explicit START path as main. */
    line_follow_acknowledge_tilt(&tilted);
    now+=5;
    line_follow_update_heading(&tilted,center,count,now,.005f,true,true,0);
    assert(tilted.state==LINE_TRACKING && tilted.start_result==LINE_START_ACCEPTED);
    assert(tilted.command.enabled && !tilted.stop_snapshot.valid);
    line_follow_stop(&tilted,LINE_STOP_IMU);
    line_follow_acknowledge_tilt(&tilted);
    assert(tilted.stop_reason==LINE_STOP_IMU && !tilted.command.enabled);

    /* Captured board sequence: pickup before first START, then 0x30 on line. */
    LineFollow idle_tilt;
    line_follow_init(&idle_tilt,geometry,steps,2,NULL,NULL);
    line_follow_stop(&idle_tilt,LINE_STOP_TILT);
    assert(idle_tilt.stop_snapshot.state_before_stop==LINE_IDLE);
    GrayObservation offset_center=gray_classify(0x30,true,0);
    for (unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) {
        now+=5;
        line_follow_update_heading(&idle_tilt,offset_center,count,now,.005f,
                                   i==LINE_CONFIRM_SAMPLES-1U,true,0);
    }
    assert(idle_tilt.state==LINE_STOPPED && !idle_tilt.command.enabled);
    line_follow_acknowledge_tilt(&idle_tilt); /* Fresh level check is in main. */
    now+=5;
    line_follow_update_heading(&idle_tilt,offset_center,count,now,.005f,true,true,0);
    assert(idle_tilt.start_result==LINE_START_ACCEPTED && idle_tilt.state==LINE_TRACKING);
    assert(idle_tilt.command.enabled);

    puts("PASS: MPU yaw/IMU fault lock; tilt remains stopped until level + explicit START; IDLE pickup/0x30 restart");
    return 0;
}
