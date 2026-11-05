#define main navigation_regressions
#include "test_navigation.c"
#undef main
#include <string.h>

static void test_approach_and_entry(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  RouteStep s[]={{1,50,mirror?RIGHT:LEFT,50,0,false,0},{1,50,STOP,50,0,false,0}};
  LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
  f.startup_elapsed_s=1;f.base=f.left_target_cm_s=f.right_target_cm_s=50;
  tick(&f,&c,&t,0,0,0x18,false);
  CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=JUNCTION_SETTLE_SPEED_CM_S+.001f);
  RoutePlanner p;float d=0;init(&p,s,2);t=0;
  route_set_speed(&p,30,30);junction(&p,&d,&t,mirror?0xf8:0x1f,0);
  CHECK(p.junctions_seen==1 && p.state==ROUTE_BRAKING);
  route_set_speed(&p,10,10);update(&p,&d,&t,mirror?0xf8:0x1f,0,1);
  CHECK(p.state==ROUTE_EXECUTING);
 }
 /* Over-speed wheels can produce zero/zero PWM even with nonzero targets.
  * Existing motor_apply then chooses standby; do not call this active yaw. */
 SpeedControl speed;MotorCommand out;speed_control_init(&speed);
 CHECK(speed_control_update(&speed,0,16,40,40,.005f,&out));
 CHECK(out.enabled && out.left==0 && out.right==0);
 CHECK(speed_control_update(&speed,0,16,16,16,.005f,&out));
 CHECK(out.enabled && out.left==0 && out.right>0);
}
static void test_early_strong_evidence(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  uint8_t branch=mirror?0xf8:0x1f;
  RouteStep s[]={{1,50,mirror?RIGHT:LEFT,50,0,false,0},{1,50,STOP,50,0,false,0}};
  RoutePlanner p;float d=0;uint32_t t=0;init(&p,s,2);
  update(&p,&d,&t,0x18,0,4);
  update(&p,&d,&t,branch,0,1);CHECK(p.state==ROUTE_CANDIDATE && p.junctions_seen==0);
  update(&p,&d,&t,branch,0,1);CHECK(p.state==ROUTE_EXECUTING && p.junctions_seen==1);
  /* A side -> both -> other-side sequence must not confirm at the middle. */
  init(&p,s,2);d=0;t=0;update(&p,&d,&t,0x18,0,4);
  update(&p,&d,&t,branch,0,1);update(&p,&d,&t,0xff,0,1);
  update(&p,&d,&t,mirror?0x1f:0xf8,0,1);CHECK(p.junctions_seen==0);
  /* Non-target/weak/stationary/sampling-gap evidence keeps its original gates. */
  s[0].junction_count=2;init(&p,s,2);d=0;t=0;update(&p,&d,&t,0x18,0,4);
  update(&p,&d,&t,branch,0,2);CHECK(p.junctions_seen==0);
  update(&p,&d,&t,branch,0,1);CHECK(p.junctions_seen==1 && p.passing_junction);
  s[0].junction_count=1;init(&p,s,2);d=0;t=0;update(&p,&d,&t,0x18,0,4);
  update(&p,&d,&t,mirror?0xc0:0x03,0,3);CHECK(p.junctions_seen==0);
  update(&p,&d,&t,mirror?0xc0:0x03,0,1);CHECK(p.junctions_seen==1);
  init(&p,s,2);d=0;t=0;update(&p,&d,&t,0x18,0,4);
  for(unsigned i=0;i<3;++i) {t+=5;route_update(&p,mask(branch),d,0,t);}
  CHECK(p.junctions_seen==0);
  init(&p,s,2);d=0;t=0;update(&p,&d,&t,0x18,0,4);update(&p,&d,&t,branch,0,1);
  t+=CONTROL_MAX_INTERVAL_MS+1;route_update(&p,mask(branch),d+.1f,0,t);
  CHECK(p.state==ROUTE_FOLLOW && p.junctions_seen==0);
 }
}
static void prepare_capture(LineFollow *f,EncoderCounts *c,uint32_t *t,const RouteStep *s,float sign)
{
 start(f,c,t,s,2);f->startup_elapsed_s=1;
 f->base=f->left_target_cm_s=f->right_target_cm_s=16;
 f->route.state=ROUTE_EXECUTING;f->route.phase_ms=*t;
 f->route.entry_heading=f->route.action_start_heading=-sign*.80f;
}
static void test_continuous_handoff(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  float sign=mirror?1.0f:-1.0f;
  RouteStep s[]={{1,50,mirror?RIGHT:LEFT,50,0,false,0},{1,50,STOP,50,0,false,0}};
  LineFollow f;EncoderCounts c;uint32_t t;prepare_capture(&f,&c,&t,s,sign);
  const uint8_t sweep[]={0x04,0x08,0x10,0x10};
  for(unsigned i=0;i<4;++i) {
   uint8_t line=mirror?(uint8_t)(sweep[i]==0x04?0x20:sweep[i]==0x08?0x10:0x08):sweep[i];
   tick(&f,&c,&t,8,8,line,false);
  }
  CHECK(f.route.state==ROUTE_CLEARING && f.state==LINE_TRACKING);
  CHECK((mirror?-1:1)*f.pid.derivative>1); /* the line is still sweeping laterally */
  /* Settling yaw must not vanish when position control changes FSM phase. */
  f.pid.config.kd=0;f.turn_yaw_rate_rad_s=sign;
  tick(&f,&c,&t,8,8,0x18,false);
  CHECK(sign*f.correction<-.1f);
  float anchor=f.route.line_exit_heading_rad;
  f.odometry.heading_rad=anchor+sign*.1f;
  tick(&f,&c,&t,0,0,0,false);
  CHECK(f.abnormal && f.state==LINE_GAP_HOLD);
  NEAR(f.hold_heading_rad,anchor);
  CHECK(sign*f.correction<0);
  CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=16.001f);
  for(unsigned i=0;i<2;++i) tick(&f,&c,&t,0,0,0x18,false);
  CHECK(f.abnormal); /* no single-frame automatic recovery */
  tick(&f,&c,&t,0,0,0x18,false);CHECK(!f.abnormal && f.state==LINE_TRACKING);
  for(unsigned i=0;i<100;++i) tick(&f,&c,&t,4,4,0,false);
  CHECK(f.state!=LINE_STOPPED && f.route.state==ROUTE_CLEARING);
  for(unsigned i=0;i<600 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&t,4,4,0,false);
  CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_ROUTE);
  CHECK(f.route.navigation_warning==ROUTE_FAILURE_ACTION_TIMEOUT && f.route.failure==ROUTE_FAILURE_EXIT_LOST);
 }
}
static void test_planned_entry_limits(void)
{
 RouteStep s[]={{1,50,LEFT,200,0,false,1000},{1,50,STOP,50,0,false,0}};
 LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
 f.startup_elapsed_s=1;f.left_target_cm_s=f.right_target_cm_s=f.base=50;f.braking_active=false;
 tick(&f,&c,&t,0,0,0x18,false);CHECK(f.base>16); /* measured long leg may cruise */
 f.route.final_segment_origin_cm=f.odometry.distance_cm-1000;
 tick(&f,&c,&t,0,0,0x18,false);CHECK(f.base<=16.001f);
 RoutePlanner p;init(&p,s,2);NEAR(route_entry_speed_limit(&p),16);
 s[0].turn_speed=6;init(&p,s,2);NEAR(route_entry_speed_limit(&p),6);
 s[0].turn_speed=50;s[0].action=STRAIGHT;init(&p,s,2);NEAR(route_entry_speed_limit(&p),50);
 s[0].action=CUSTOM_TURN;s[0].angle_deg=90;init(&p,s,2);NEAR(route_entry_speed_limit(&p),16);
 p.index=1;NEAR(route_entry_speed_limit(&p),50); /* final STOP */
 p.repeat_count=0;p.repeat_action=LEFT;NEAR(route_entry_speed_limit(&p),16);
 p.repeat_action=STRAIGHT;NEAR(route_entry_speed_limit(&p),50);
}
static void test_reverse_evidence_at_handoff(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  RouteStep s[]={{1,50,mirror?RIGHT:LEFT,50,0,false,0},{1,50,STOP,50,0,false,0}};
  LineFollow f;EncoderCounts c;uint32_t t;prepare_capture(&f,&c,&t,s,mirror?1:-1);
  for(unsigned i=0;i<2;++i) tick(&f,&c,&t,0,0,0x18,false);
  if(mirror) {f.odometry.left_speed_cm_s=-.7f;f.speed.left_reverse_s=.045f;}
  else {f.odometry.right_speed_cm_s=-.7f;f.speed.right_reverse_s=.045f;}
  tick(&f,&c,&t,0,0,0x18,false);
  CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_ENCODER);
  CHECK(f.speed.fault==SPEED_FAULT_NONE); /* stop reset, snapshot keeps original fault */
  CHECK(f.stop_snapshot.speed_fault==SPEED_FAULT_REVERSE);
  CHECK(f.stop_snapshot.speed_fault_wheel==(mirror?1:2));
 }
}
#if MPU6050_ENABLED
static void test_real_imu_handoff(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  float sign=mirror?1:-1;
  RouteStep s[]={{1,50,mirror?RIGHT:LEFT,50,0,false,0},{1,50,STOP,50,0,false,0}};
  LineFollow f;EncoderCounts c={0};uint32_t t=0;
  line_follow_init(&f,calibration(),s,2,NULL,NULL);f.route.repeat_count=1;
  for(unsigned i=0;i<3;++i) {t+=5;line_follow_update_heading(&f,mask(0x18),c,t,.005f,i==2,true,0);}
  f.startup_elapsed_s=1;f.route.state=ROUTE_EXECUTING;f.route.phase_ms=t;
  f.route.entry_heading=f.route.action_start_heading=-sign*.8f;
  f.base=f.left_target_cm_s=f.right_target_cm_s=16;
  const uint8_t sequence[]={0x04,0x08,0x10,0x10};
  for(unsigned i=0;i<4;++i) {
   uint8_t m=mirror?(uint8_t)(sequence[i]==4?0x20:sequence[i]==8?0x10:0x08):sequence[i];
   t+=5;c.left+=8;c.right+=8;
   line_follow_update_heading(&f,mask(m),c,t,.005f,false,true,0);
  }
  CHECK(f.heading_source==HEADING_IMU && f.route.state==ROUTE_CLEARING);
  CHECK((mirror?-1:1)*f.pid.derivative>1);
  f.pid.config.kd=0;t+=5;c.left+=8;c.right+=8;
  line_follow_update_heading(&f,mask(0x18),c,t,.005f,false,true,sign*.01f);
  CHECK(sign*f.correction<0);
  t+=5;c.left+=8;c.right+=8;
  line_follow_update_heading(&f,mask(0x18),c,t,.005f,false,false,NAN);
  CHECK(f.heading_source==HEADING_ENCODER && f.route.state==ROUTE_CLEARING);
  CHECK(sign*f.correction<0 && f.state!=LINE_STOPPED);
 }
}
#endif
int main(int argc,char **argv)
{
 if(argc<2 || strcmp(argv[1],"approach")==0) test_approach_and_entry();
 if(argc<2 || strcmp(argv[1],"evidence")==0) test_early_strong_evidence();
 if(argc<2 || strcmp(argv[1],"history")==0) test_continuous_handoff();
 if(argc<2) {test_planned_entry_limits();test_reverse_evidence_at_handoff();}
#if MPU6050_ENABLED
 if(argc<2) test_real_imu_handoff();
#endif
 printf("PASS: slow measured entry, early same-side strong junction, continuous capture and bounded recovery (%u checks)\n",checks);
 return 0;
}
