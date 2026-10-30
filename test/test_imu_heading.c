#include "gray_sensor.h"
#include "line_follow.h"
#include "motion_config.h"
#include "tilt_guard.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* Healthy IMU owns yaw; first failure latches continuous encoder fallback. */
static void test_turn_fallback(void)
{
    for (unsigned mirror=0;mirror<2;++mirror) {
        for (unsigned invalid=0;invalid<2;++invalid) {
            const float sign=mirror?-1.0f:1.0f;
            OdometryCalibration geometry={4.8f,4.8f,1560,1560,14.5f,-1,1,true};
            RouteStep steps[]={{1,6,mirror?LEFT:RIGHT,40,0,false,0},{1,6,STOP,40,0,false,0}};
            LineFollow f;
            line_follow_init(&f,geometry,steps,2,NULL,NULL);
            EncoderCounts c={30000,30000};
            GrayObservation center=gray_classify(0x18,true,0), blank=gray_classify(0,true,0);
            uint32_t now=0;
            for(unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) {
                now+=5;
                line_follow_update_heading(&f,center,c,now,.005f,i==LINE_CONFIRM_SAMPLES-1,true,4.0f);
            }
            assert(f.command.enabled && f.heading_source==HEADING_IMU);
            f.route.state=ROUTE_EXECUTING;f.route.phase_ms=now;
            f.route.entry_heading=f.route.action_start_heading=4.0f-sign*.8f;
            f.route.entry_cm=f.odometry.distance_cm;
            /* Include the failed sample's encoder increment, without resetting yaw. */
            c.left-=(mirror?2:4);c.right+=(mirror?4:2);now+=5;
            line_follow_update_heading(&f,blank,c,now,.005f,false,invalid!=0,invalid?NAN:0.0f);
            assert(f.state!=LINE_STOPPED && f.heading_source==HEADING_ENCODER);
            assert(f.heading_fallback_reason==HEADING_FALLBACK_DATA && f.heading_fallback_ms==now);
            float expected=4.0f+sign*2*f.odometry.left_cm_per_count/geometry.track_cm*ODOMETRY_HEADING_SCALE;
            assert(fabsf(f.odometry.heading_rad-expected)<.00001f);
            assert(f.route.index==0 && f.route.state==ROUTE_EXECUTING);
            uint32_t fallback_ms=now;
            for(unsigned i=0;i<200 && sign*(expected-f.route.entry_heading)<JUNCTION_TURN_TARGET_RAD;++i) {
                if(mirror) c.right+=8; else c.left-=8;
                expected+=sign*8*f.odometry.left_cm_per_count/geometry.track_cm*ODOMETRY_HEADING_SCALE;
                now+=5;
                /* Later finite but unrelated IMU yaw must not be accepted again. */
                line_follow_update_heading(&f,blank,c,now,.005f,false,true,-100.0f);
                assert(f.state!=LINE_STOPPED && f.route.state==ROUTE_EXECUTING);
                assert(fabsf(f.odometry.heading_rad-expected)<.0001f);
            }
            for(unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) {
                c.left-=2;c.right+=2;now+=5;
                line_follow_update_heading(&f,center,c,now,.005f,false,true,100.0f);
            }
            assert(f.route.state==ROUTE_CLEARING && f.state==LINE_TRACKING && f.command.enabled);
            assert(f.heading_fallback_ms==fallback_ms && !f.stop_snapshot.valid);
            for(unsigned i=0;i<100 && f.route.index==0;++i) {
                c.left-=10;c.right+=10;now+=5;
                line_follow_update_heading(&f,center,c,now,.005f,false,true,100.0f);
                assert(f.state!=LINE_STOPPED && f.heading_source==HEADING_ENCODER);
            }
            assert(f.route.state==ROUTE_FOLLOW && f.route.index==1 && f.route.junctions_seen==0);
            line_follow_stop(&f,LINE_STOP_REQUEST);
            assert(f.stop_snapshot.heading_source==HEADING_ENCODER &&
                   f.stop_snapshot.heading_fallback_ms==fallback_ms);
            for(unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) {
                now+=5;
                line_follow_update_heading(&f,center,c,now,.005f,i==LINE_CONFIRM_SAMPLES-1,true,0);
            }
            assert(f.start_result==LINE_START_ACCEPTED && f.heading_source==HEADING_ENCODER);
            /* Encoder and task faults must still stop and remain locked. */
            c.left+=32768U;now+=5;
            line_follow_update_heading(&f,center,c,now,.005f,false,false,0);
            assert(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_ENCODER && !f.command.enabled);
            now+=5;
            line_follow_update_heading(&f,center,c,now,.005f,true,false,0);
            assert(f.start_result==LINE_START_REJECTED_FAULT && !f.command.enabled);
            line_follow_init(&f,geometry,steps,2,NULL,NULL);
            for(unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i)
                line_follow_update_heading(&f,center,c,(i+1U)*5U,.005f,i==LINE_CONFIRM_SAMPLES-1,false,0);
            assert(f.command.enabled);
            line_follow_update_heading(&f,center,c,100,.020f,false,false,0);
            assert(f.stop_reason==LINE_STOP_FAULT && !f.command.enabled);
        }
    }
}
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

    RouteStep steps[] = {{1, 6.0f, RIGHT, 120, 0, false, 0.0f},{1, 6.0f, STOP, 40, 0, false, 0.0f}};
    LineFollow f;
    line_follow_init(&f,geometry,steps,2,NULL,NULL);
    GrayObservation center = gray_classify(0x18,true,0);
    GrayObservation blank = gray_classify(0,true,0);
    count = (EncoderCounts){30000,30000};
    uint32_t now = 0;
    for (unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) {
        count.left -= 4; count.right += 4;
        now += 5;
        line_follow_update_heading(&f,center,count,now,.005f,
                                   i==LINE_CONFIRM_SAMPLES-1U,true,0.0f);
    }
    assert(f.state==LINE_TRACKING && f.start_result==LINE_START_ACCEPTED);
    f.route.state=ROUTE_EXECUTING;
    f.route.phase_ms=now;
    f.route.entry_heading=f.route.action_start_heading=0;
    f.route.entry_cm=f.odometry.distance_cm;
    f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
    f.base=f.left_target_cm_s=f.right_target_cm_s=120;
    for (unsigned i=0;i<50;++i) {
        count.left -= 20; /* Encoder suggests turning right, IMU reports no turn. */
        now += 5;
        line_follow_update_heading(&f,blank,count,now,.005f,false,true,0.0f);
        assert(f.state!=LINE_STOPPED);
        assert(f.route.state==ROUTE_EXECUTING && fabsf(f.odometry.heading_rad)<.000001f);
    }
    for (unsigned i=0;i<20 && f.route.state==ROUTE_EXECUTING;++i) {
        count.left -= 4; count.right += 4;
        now += 5;
        line_follow_update_heading(&f,center,count,now,.005f,false,true,1.60f);
    }
    assert(f.route.state==ROUTE_CLEARING && fabsf(f.odometry.heading_rad-1.60f)<.000001f);
    now += 5;
    line_follow_update_heading(&f,center,count,now,.005f,false,false,1.60f);
    assert(f.state!=LINE_STOPPED && f.heading_source==HEADING_ENCODER &&
           !f.stop_snapshot.valid && f.command.enabled);
    assert(fabsf(f.odometry.heading_rad-1.60f)<.000001f);
    now += 5;
    line_follow_update_heading(&f,center,count,now,.005f,true,true,1.60f);
    assert(f.state!=LINE_STOPPED && f.heading_source==HEADING_ENCODER);

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

    line_follow_stop(&idle_tilt,LINE_STOP_TILT);
    now+=5;
    line_follow_update_heading(&idle_tilt,center,count,now,.005f,true,false,0);
    assert(idle_tilt.heading_source==HEADING_ENCODER && idle_tilt.stop_reason==LINE_STOP_TILT);
    assert(idle_tilt.state==LINE_STOPPED && !idle_tilt.command.enabled);

    test_turn_fallback();
    puts("PASS: MPU yaw, startup/runtime encoder fallback, continuous mirrored turns, sticky source and encoder fault lock; tilt restart gates");
    return 0;
}
