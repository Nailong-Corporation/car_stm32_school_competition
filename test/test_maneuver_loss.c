#define main navigation_previous_main
#include "test_navigation.c"
#undef main
#include <string.h>

static void maneuver_start(LineFollow *f,EncoderCounts *c,uint32_t *t,const RouteStep *s)
{
 *c=(EncoderCounts){0};*t=0;line_follow_init(f,calibration(),s,2,NULL,NULL);
 f->route.repeat_count=1;
 for(unsigned i=0;i<3;++i) {
  *t+=5;line_follow_update_heading(f,mask(0x18),*c,*t,.005f,i==2,true,0);
 }
 CHECK(f->state==LINE_TRACKING);
 f->startup_elapsed_s=1;
}
static void heading_tick(LineFollow *f,EncoderCounts *c,uint32_t *t,float sign,uint8_t m)
{
 c->left=(uint16_t)(c->left+(sign>0?5:4));
 c->right=(uint16_t)(c->right+(sign>0?4:5));*t+=5;
 line_follow_update_heading(f,mask(m),*c,*t,.005f,false,true,
                            f->odometry.heading_rad+sign*.001f);
 CHECK(f->left_target_cm_s>=0 && f->right_target_cm_s>=0);
 CHECK(f->left_target_cm_s<=WHEEL_TARGET_MAX_CM_S && f->right_target_cm_s<=WHEEL_TARGET_MAX_CM_S);
}
static void executing_loss(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) for(unsigned custom=0;custom<2;++custom) {
  float sign=mirror?1:-1;
  RouteStep s[]={{1,50,custom?CUSTOM_TURN:mirror?RIGHT:LEFT,50,custom?sign*135:0,false,65},
                 {1,50,STOP,50,0,false,0}};
  LineFollow f;EncoderCounts c;uint32_t t;maneuver_start(&f,&c,&t,s);
  f.route.state=ROUTE_EXECUTING;f.route.phase_ms=t;f.route.entry_cm=f.odometry.distance_cm;
  f.route.entry_heading=f.odometry.heading_rad-sign*(custom?1.6f:.9f);
  heading_tick(&f,&c,&t,sign,mirror?0xc0:0x03);
  CHECK(f.route.capture_line_seen && !f.route.line_exit_confirmed);
  float before=f.odometry.heading_rad;
  for(unsigned i=0;i<220;++i) {
   heading_tick(&f,&c,&t,sign,i%30==29?0xff:0);
   CHECK(f.state!=LINE_STOPPED && f.route.state==ROUTE_EXECUTING);
  }
  CHECK(sign*(f.odometry.heading_rad-before)>.1f);
  CHECK(sign*(f.left_target_cm_s-f.right_target_cm_s)>0);
#if MPU6050_ENABLED
  CHECK(f.heading_source==HEADING_IMU);
#endif
  /* Capture loss cannot skip the original exit and custom-angle gates. */
  CHECK(!f.route.line_exit_confirmed && f.route.index==0);
  for(unsigned i=0;i<1000 && f.route.state==ROUTE_EXECUTING;++i) {
   c.left+=4;c.right+=4;t+=5;
   line_follow_update_heading(&f,mask(0),c,t,.005f,false,true,f.odometry.heading_rad);
   CHECK(f.state!=LINE_STOPPED);
  }
  CHECK(f.route.state==ROUTE_RECOVERING && f.route.navigation_warning==ROUTE_FAILURE_ACTION_TIMEOUT);
  for(unsigned i=0;i<300 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&t,4,4,0,false);
  CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_ROUTE && f.route.failure==ROUTE_FAILURE_EXIT_LOST);
 }
}
static void clearing_loss(void)
{
 for(unsigned straight=0;straight<2;++straight) {
  RouteStep s[]={{1,50,straight?STRAIGHT:LEFT,50,0,false,65},{1,50,STOP,50,0,false,0}};
  LineFollow f;EncoderCounts c;uint32_t t;maneuver_start(&f,&c,&t,s);
  f.route.state=ROUTE_CLEARING;f.route.phase_ms=f.route.capture_line_ms=t;
  f.route.clear_cm=f.route.capture_line_cm=f.odometry.distance_cm;
  f.route.line_exit_confirmed=!straight;
  for(unsigned i=0;i<90;++i) {
   tick(&f,&c,&t,4,4,0,false);
   CHECK(f.state!=LINE_STOPPED && f.route.state==ROUTE_CLEARING);
  }
  CHECK(f.route.index==0 && !f.route.next_segment_origin_valid);
  /* No line: original clearance limit aborts to RECOVERING, then loss stops. */
  for(unsigned i=0;i<700 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&t,4,4,0,false);
  CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_ROUTE);
  CHECK(f.route.navigation_warning==ROUTE_FAILURE_ACTION_TIMEOUT);
  CHECK(f.route.failure==ROUTE_FAILURE_EXIT_LOST && !f.command.enabled);
 }
}
static void handoff_deadline(void)
{
 for(unsigned speed=0;speed<2;++speed) {
  RouteStep s[]={{1,50,LEFT,50,0,false,65},{1,50,STOP,50,0,false,0}};
  LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
  for(unsigned i=0;i<30;++i) tick(&f,&c,&t,speed?10:6,speed?10:6,0x18,false);
  tick(&f,&c,&t,speed?10:6,speed?10:6,0x1f,false);
  CHECK(f.route.state==ROUTE_CANDIDATE);
  f.abnormal=true;f.last_reliable_ms=t+5-INTERFERENCE_TIMEOUT_MS;
  tick(&f,&c,&t,speed?10:6,speed?10:6,0x1f,false);
  CHECK(f.state!=LINE_STOPPED && f.route.junctions_seen==1);
  CHECK(f.route.state==(speed?ROUTE_BRAKING:ROUTE_EXECUTING));
  if(speed) CHECK(f.left_target_cm_s==0 && f.right_target_cm_s==0);
 }
}
static void hard_faults(void)
{
 RouteStep s[]={{1,50,LEFT,50,0,false,65},{1,50,STOP,50,0,false,0}};
 LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
 f.route.state=ROUTE_EXECUTING;f.route.phase_ms=t;
 tick(&f,&c,&t,30000,0,0,false);
 CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_ENCODER);
 start(&f,&c,&t,s,2);f.route.state=ROUTE_CLEARING;f.route.phase_ms=t;
 line_follow_update(&f,mask(0),c,t+20,.020f,false);
 CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_FAULT);
}
static void straight_exit_gate(void)
{
 RouteStep s[]={{1,50,STRAIGHT,50,0,false,65},{1,50,STOP,50,0,false,0}};
 RoutePlanner p;init(&p,s,2);float d=0;uint32_t t=0;
 junction(&p,&d,&t,0xff,0);
 update(&p,&d,&t,0,0,90);CHECK(p.state==ROUTE_EXECUTING && !p.line_exit_confirmed);
 update(&p,&d,&t,0x18,0,2);CHECK(p.state==ROUTE_EXECUTING);
 update(&p,&d,&t,0,0,1);update(&p,&d,&t,0x18,0,2);CHECK(p.state==ROUTE_EXECUTING);
 update(&p,&d,&t,0x18,0,1);CHECK(p.state==ROUTE_CLEARING && p.line_exit_confirmed);
 CHECK(p.next_segment_origin_valid);NEAR(p.next_segment_origin_cm,d);
}
int main(int argc,char **argv)
{
 if(argc<2 || strcmp(argv[1],"executing")==0) executing_loss();
 if(argc<2 || strcmp(argv[1],"clearing")==0) clearing_loss();
 if(argc<2 || strcmp(argv[1],"handoff")==0) handoff_deadline();
 if(argc<2 || strcmp(argv[1],"hard")==0) hard_faults();
 if(argc<2 || strcmp(argv[1],"straight")==0) straight_exit_gate();
 printf("PASS: maneuver loss ownership, IMU/encoder progress, bounded recovery, handoff and hard faults (%u checks)\n",checks);
 return 0;
}
