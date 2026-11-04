#include "gray_test_input.h"
#include "line_follow.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* Software mechanism, not a measurement of the parked vehicle's PID history. */
static void test_no_forward_torque_above_target(void)
{
    for (unsigned side=0;side<2;++side) {
        SpeedControl s;MotorCommand out;speed_control_init(&s);
        s.left.integral=s.right.integral=SPEED_INTEGRAL_LIMIT;
        float lm=side?15:32.1849327f,rm=side?32.1849327f:15;
        assert(speed_control_update(&s,15.2f,15.2f,lm,rm,.005f,&out));
        assert(side?out.right==0:out.left==0);
        assert(side?s.right.integral==0:s.left.integral==0);
        /* The other wheel still has its independent speed loop. */
        assert(side?out.left>0:out.right>0);
        assert(speed_control_update(&s,15.2f,15.2f,14,14,.005f,&out));
        assert(out.left>0 && out.right>0 && s.fault==SPEED_FAULT_NONE);
    }
}

static GrayObservation mask(uint8_t m) {return gray_classify(m,true,0);}
static void test_finite_deceleration(Action action,float deceleration,bool expect_stop)
{
    RouteStep steps[]={{1,50,action,50,0,false,65},{1,50,STOP,50,0,false,0}};
    OdometryCalibration cal={1/(3.14159265359f*ODOMETRY_DISTANCE_SCALE),
        1/(3.14159265359f*ODOMETRY_DISTANCE_SCALE),100,100,14,1,1,true};
    LineFollow f;EncoderCounts c={0};uint32_t now=0;
    line_follow_init(&f,cal,steps,action==STOP?1:2,NULL,NULL);f.route.repeat_count=1;
    for(unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) {
        now+=5;line_follow_update(&f,mask(0x18),c,now,.005f,i==LINE_CONFIRM_SAMPLES-1);
    }
    assert(f.state==LINE_TRACKING);
    /* Bounded passive deceleration 60 cm/s², 50 ms command delay. This is a
     * finite-response counterexample to the old instantaneous-target fixture,
     * not an identified motor model or a promise for other batteries/surfaces. */
    float vl=0,vr=0,cl=0,cr=0;
    float delay_l[10]={0},delay_r[10]={0};unsigned slot=0;
    bool crossed=false;
    for(unsigned i=0;i<5000 && f.odometry.distance_cm<65;++i) {
        float dl=delay_l[slot]/SPEED_FEEDFORWARD;
        float dr=delay_r[slot]/SPEED_FEEDFORWARD;
        delay_l[slot]=f.command.enabled?f.command.left:0;
        delay_r[slot]=f.command.enabled?f.command.right:0;
        slot=(slot+1)%10;
        vl+=fmaxf(-deceleration*.005f,fminf(160*.005f,(dl-vl)*.005f/.15f));
        vr+=fmaxf(-deceleration*.005f,fminf(160*.005f,(dr-vr)*.005f/.15f));
        vl=fmaxf(0,vl);vr=fmaxf(0,vr);
        cl+=vl*.005f/.01f;cr+=vr*.005f/.01f;
        unsigned lc=(unsigned)cl,rc=(unsigned)cr;cl-=lc;cr-=rc;
        c.left=(uint16_t)(c.left+lc);c.right=(uint16_t)(c.right+rc);now+=5;
        line_follow_update(&f,mask(0x18),c,now,.005f,false);
        if(f.state==LINE_STOPPED && expect_stop) {
            assert(f.route.failure==ROUTE_FAILURE_BRAKE_DISTANCE);
            assert(!f.command.enabled);
            return;
        }
        if(f.state==LINE_STOPPED) fprintf(stderr,"finite response action=%u d=%.3f speed=%.3f/%.3f reason=%u\n",
            (unsigned)action,f.odometry.distance_cm,f.odometry.left_speed_cm_s,
            f.odometry.right_speed_cm_s,(unsigned)f.route.failure);
        assert(f.state!=LINE_STOPPED);
        assert(f.route.state==ROUTE_FOLLOW && f.route.junctions_seen==0);
        assert(f.command.left>=0 && f.command.right>=0);
        float d=f.odometry.distance_cm-f.route.final_segment_origin_cm;
        if(d>=65-BRAKING_DISTANCE_MARGIN_CM-BRAKING_REVERSE_GUARD_CM) {
            crossed=true;
            assert(fmaxf(f.odometry.left_speed_cm_s,f.odometry.right_speed_cm_s)<=
                   route_entry_speed_limit(&f.route)+REVERSE_BRAKE_MARGIN_CM_S);
        }
    }
    assert(!expect_stop && crossed && f.odometry.distance_cm>=65);
}
static void test_stop_effort_snapshot(void)
{
    LineFollow f={0};f.state=LINE_TRACKING;
    f.command=(MotorCommand){27,31,true};
    f.speed.left.integral=4;f.speed.right.integral=5;
    line_follow_stop(&f,LINE_STOP_REQUEST);
    assert(f.stop_snapshot.left_command_pwm==27 && f.stop_snapshot.right_command_pwm==31);
    assert(f.stop_snapshot.left_speed_integral==4 && f.stop_snapshot.right_speed_integral==5);
    assert(f.command.left==0 && f.command.right==0 && !f.command.enabled);
    assert(f.speed.left.integral==0 && f.speed.right.integral==0);
    line_follow_stop(&f,LINE_STOP_REQUEST);
    assert(f.stop_snapshot.left_command_pwm==27 && f.stop_snapshot.left_speed_integral==4);
}
int main(int argc,char **argv)
{
    (void)argv;
    if(argc==1) test_no_forward_torque_above_target();
    test_stop_effort_snapshot();
    test_finite_deceleration(LEFT,60,false);test_finite_deceleration(RIGHT,60,false);
    test_finite_deceleration(STOP,60,false);
    /* Insufficient physical deceleration must still fail at the original deadline. */
    test_finite_deceleration(LEFT,15,true);
    puts("PASS: passive overspeed output and finite-response approach braking");
    return 0;
}
