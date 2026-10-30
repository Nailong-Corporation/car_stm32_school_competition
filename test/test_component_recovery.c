#define main navigation_regressions
#include "test_navigation.c"
#undef main

static void prepare(RoutePlanner *p)
{
 static const RouteStep steps[]={{1,50,CUSTOM_TURN,200,90,false,0},{1,50,STOP,200,0,false,0}};
 init(p,steps,2);
 p->state=ROUTE_EXECUTING;p->phase_ms=100;p->last_sample_ms=100;
 p->entry_heading=0;p->capture_line_seen=true;p->capture_line_ms=100;p->capture_line_cm=0;
}

static void test_recovery_limits(void)
{
 for(unsigned bound=0;bound<2;++bound) {
  RoutePlanner p;prepare(&p);
  route_update(&p,mask(0x3b),0,1.6f,105);
  CHECK(p.component_recovery_active && p.component_recovery_used);
  float heading=p.component_recovery_heading;
  for(unsigned i=0;i<90;++i) {
   uint32_t t=110+i*5;float d=.02f*i;
   route_update(&p,mask(i%9==0?0x18:i%2?0:0xff),d,1.6f,t);
   CHECK(p.state==ROUTE_EXECUTING && p.component_recovery_active);
   CHECK(p.component_recovery_ms==105 && p.component_recovery_heading==heading);
   CHECK(p.junctions_seen==0 && p.index==0);
  }
  route_update(&p,mask(0x18),bound?COMPONENT_RECOVERY_MAX_CM:2,1.6f,
      bound?600:105+COMPONENT_RECOVERY_TIMEOUT_MS);
  CHECK(p.failure==ROUTE_FAILURE_EXIT_RECAPTURE); /* deadline before recovery */
  route_update(&p,mask(0x18),2,1.6f,1400);CHECK(p.state==ROUTE_FAILED);
 }
 RoutePlanner p;prepare(&p);
 route_update(&p,mask(0),0,1.6f,105);CHECK(!p.component_recovery_active);
 route_update(&p,mask(0x3b),0,1.6f,500);CHECK(p.failure==ROUTE_FAILURE_EXIT_LOST);
 prepare(&p);p.capture_line_seen=false;
 route_update(&p,mask(0xff),0,1.6f,105);CHECK(!p.component_recovery_active);
 prepare(&p);route_update(&p,mask(0xff),0,1.0f,105);
 CHECK(!p.component_recovery_active); /* cannot terminate a half-finished turn */
 prepare(&p);route_update(&p,mask(0xff),0,1.6f,105);
 route_update(&p,mask(0xff),1,-.36f,110);CHECK(p.failure==ROUTE_FAILURE_WRONG_WAY);
 prepare(&p);route_update(&p,mask(0xff),0,1.6f,105);
 route_update(&p,mask(0xff),1,2.2f,110);CHECK(p.failure==ROUTE_FAILURE_TURN_LIMIT);
 prepare(&p);route_update(&p,mask(0xff),0,1.6f,105);
 route_update(&p,mask(0xff),1,1.6f,100+JUNCTION_ACTION_TIMEOUT_MS);
 CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
}

static void test_line_and_custom_gate(void)
{
 RoutePlanner p;prepare(&p);
 route_update(&p,mask(0xff),0,1.6f,105);
 for(unsigned i=0;i<3;++i) route_update(&p,mask(0x03),.1f*i,1.6f,110+5*i);
 CHECK(p.component_recovery_active); /* edge is not an exit */
 route_update(&p,mask(0x18),.4f,1.6f,125);
 route_update(&p,mask(0xff),.5f,1.6f,130);
 for(unsigned i=0;i<2;++i) route_update(&p,mask(0x18),.6f+.1f*i,1.6f,135+5*i);
 CHECK(p.component_recovery_active);
 route_update(&p,mask(0x18),.8f,1.6f,145);
 CHECK(!p.component_recovery_active && p.component_recovery_used && p.state==ROUTE_EXECUTING);
 route_update(&p,mask(0xff),.9f,1.6f,150);
 CHECK(!p.component_recovery_active); /* no second budget on the same action */

 prepare(&p);route_update(&p,mask(0xff),0,1.5f,105);
 for(unsigned i=0;i<8;++i) route_update(&p,mask(0x18),.02f*i,1.5f,110+5*i);
 CHECK(p.state==ROUTE_EXECUTING && !p.line_exit_confirmed); /* custom angle evidence retained */
}

static void test_control_and_clearance(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  RouteStep steps[]={{1,50,mirror?RIGHT:LEFT,200,0,false,0},{1,50,STOP,200,0,false,0}};
  LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,steps,2);
  f.startup_elapsed_s=1;f.left_target_cm_s=f.right_target_cm_s=f.base=16;
  f.route.state=ROUTE_EXECUTING;f.route.phase_ms=t;
  f.route.entry_heading=mirror?-1.6f:1.6f;
  f.route.capture_line_seen=true;f.route.capture_line_ms=t;f.route.capture_line_cm=f.odometry.distance_cm;
  tick(&f,&c,&t,5,5,0x3b,false);
  CHECK(f.exit_recovery_active && f.route.component_recovery_active && !f.pid.primed);
  NEAR(f.left_target_cm_s,f.right_target_cm_s);
  float h=f.route.component_recovery_heading;
  for(unsigned i=0;i<90;++i) tick(&f,&c,&t,5,5,i%2?0:0xff,false);
  CHECK(f.state!=LINE_STOPPED && f.route.index==0 && f.route.junctions_seen==0);
  NEAR(f.route.component_recovery_heading,h);
  for(unsigned i=0;i<5;++i) tick(&f,&c,&t,5,5,0x18,false);
  CHECK(f.route.state==ROUTE_CLEARING && !f.route.component_recovery_active);
  CHECK(f.state==LINE_TRACKING && f.route.index==0);

  start(&f,&c,&t,steps,2);f.startup_elapsed_s=1;
  f.route.state=ROUTE_CLEARING;f.route.phase_ms=t;f.route.clear_cm=f.odometry.distance_cm;
  f.route.entry_heading=mirror?-1.6f:1.6f;f.route.line_exit_confirmed=true;
  tick(&f,&c,&t,5,5,0x3b,false);
  for(unsigned i=0;i<90;++i) tick(&f,&c,&t,5,5,0x3b,false);
  CHECK(f.route.state==ROUTE_CLEARING && f.state!=LINE_STOPPED && !f.abnormal);
  for(unsigned i=0;i<50 && f.route.index==0;++i) tick(&f,&c,&t,5,5,0x18,false);
  CHECK(f.route.index==1 && f.route.state==ROUTE_FOLLOW);
 }
 RouteStep steps[]={{2,50,LEFT,200,0,false,0},{1,50,STOP,200,0,false,0}};
 RoutePlanner p;init(&p,steps,2);float d=0;uint32_t t=0;
 junction(&p,&d,&t,0xff,0);CHECK(p.passing_junction && p.state==ROUTE_CLEARING);
 update(&p,&d,&t,0xff,0,100);
 CHECK(p.component_recovery_active && p.state==ROUTE_CLEARING && p.junctions_seen==1);
 update(&p,&d,&t,0x18,0,45);CHECK(p.state==ROUTE_FOLLOW && p.junctions_seen==1);
}

#if MPU6050_ENABLED
static void test_imu_fallback_and_feedback_fault(void)
{
 const RouteStep steps[]={{1,50,LEFT,200,0,false,0},{1,50,STOP,200,0,false,0}};
 LineFollow f;EncoderCounts c={0};uint32_t t=0;
 line_follow_init(&f,calibration(),steps,2,NULL,NULL);
 for(unsigned i=0;i<3;++i) {
  t+=5;line_follow_update_heading(&f,mask(0x18),c,t,.005f,i==2,true,0);
 }
 f.startup_elapsed_s=1;f.route.state=ROUTE_EXECUTING;f.route.phase_ms=t;
 f.route.entry_heading=1.6f;f.route.capture_line_seen=true;f.route.capture_line_ms=t;
 t+=5;c.left+=5;c.right+=5;
 line_follow_update_heading(&f,mask(0x3b),c,t,.005f,false,true,0);
 CHECK(f.heading_source==HEADING_IMU && f.exit_recovery_active);
 float anchor=f.route.component_recovery_heading;
 t+=5;c.left+=5;c.right+=5;
 line_follow_update_heading(&f,mask(0),c,t,.005f,false,false,NAN);
 CHECK(f.heading_source==HEADING_ENCODER && f.route.component_recovery_active);
 NEAR(f.route.component_recovery_heading,anchor);
 t+=5;c.left=(uint16_t)(c.left+32768U);
 line_follow_update_heading(&f,mask(0xff),c,t,.005f,false,false,NAN);
 CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_ENCODER);
 CHECK(f.stop_snapshot.exit_recovery_active);
 t+=5;line_follow_update_heading(&f,mask(0x18),c,t,.005f,true,false,NAN);
 CHECK(f.state==LINE_STOPPED && f.start_result==LINE_START_REJECTED_FAULT);
}
#endif

int main(void)
{
 test_recovery_limits();test_line_and_custom_gate();test_control_and_clearance();
#if MPU6050_ENABLED
 test_imu_fallback_and_feedback_fault();
#endif
 printf("PASS: bounded component straight recovery, mirrored handoff, one-shot budget and angle/fault gates (%u checks)\n",checks);
 return 0;
}
