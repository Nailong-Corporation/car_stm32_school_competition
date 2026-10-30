#include "gray_test_input.h"
#include "line_follow.h"
#include "braking_planner.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static unsigned checks, callbacks;
#define CHECK(c) do { ++checks; assert(c); } while(0)
#define NEAR(a,b) CHECK(fabsf((a)-(b))<.001f)
static GrayObservation mask(uint8_t m) {return gray_classify(m,true,0);}
static OdometryCalibration calibration(void)
{return (OdometryCalibration){1/(3.14159265359f*ODOMETRY_DISTANCE_SCALE),1/(3.14159265359f*ODOMETRY_DISTANCE_SCALE),100,100,10,1,1,true};}
static void tick(LineFollow *f,EncoderCounts *c,uint32_t *now,int l,int r,uint8_t m,bool request)
{
 c->left=(uint16_t)(c->left+l);c->right=(uint16_t)(c->right+r);*now+=5;
 line_follow_update(f,mask(m),*c,*now,.005f,request);
 CHECK(f->left_target_cm_s>=0 && f->right_target_cm_s>=0);
 CHECK(f->left_target_cm_s<=WHEEL_TARGET_MAX_CM_S && f->right_target_cm_s<=WHEEL_TARGET_MAX_CM_S);
}
static void start(LineFollow *f,EncoderCounts *c,uint32_t *now,const RouteStep *s,size_t n)
{
 *c=(EncoderCounts){0};*now=0;line_follow_init(f,calibration(),s,n,NULL,NULL);f->route.repeat_count=1;
 for(unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) tick(f,c,now,0,0,0x18,i==LINE_CONFIRM_SAMPLES-1);
 CHECK(f->state==LINE_TRACKING);
}
static void update(RoutePlanner *p,float *d,uint32_t *t,uint8_t m,float h,unsigned n)
{for(unsigned i=0;i<n;++i){*d+=.06f;*t+=5;route_update(p,mask(m),*d,h,*t);}}
static void init(RoutePlanner *p,const RouteStep *s,size_t n)
{route_init(p,s,n,NULL,NULL);p->repeat_count=1;route_start(p,0,0);CHECK(p->valid);}
static void junction(RoutePlanner *p,float *d,uint32_t *t,uint8_t branch,float h)
{update(p,d,t,0x18,h,4);update(p,d,t,branch,h,(branch&0x18U)?3:4);}
static void clear(RoutePlanner *p,float *d,uint32_t *t,float h)
{update(p,d,t,0x18,h,40);} /* 2.0 cm + 50 ms 离口去重证据（JUNCTION_COUNT_CLEAR_*）。 */
static void stop_callback(size_t i,void *c)
{(void)i;(void)c;++callbacks;}
static void test_counts(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  RouteStep s[]={{2,6,mirror?LEFT:RIGHT,40,0,false,0},{1,6,STOP,40,0,false,0}};
  RoutePlanner p;init(&p,s,2);float d=0;uint32_t t=0;
  /* Opposite-side first junction counts despite waiting for the other direction. */
  junction(&p,&d,&t,mirror?0xf8:0x1f,0);
  CHECK(p.junctions_seen==1 && p.state==ROUTE_CLEARING && p.index==0);
  update(&p,&d,&t,0xff,0,12);CHECK(p.junctions_seen==1);
  clear(&p,&d,&t,0);CHECK(p.state==ROUTE_FOLLOW && p.final_segment_active);
  junction(&p,&d,&t,mirror?0x1f:0xf8,0);
  CHECK(p.junctions_seen==2 && p.state==ROUTE_EXECUTING);
  CHECK(route_current_action(&p)==(mirror?LEFT:RIGHT));
  /* An ordinary turn hands control to a proven line before estimated 90deg. */
  float sign=mirror?-1:1;
  update(&p,&d,&t,0x18,sign*.8f,20);CHECK(p.state==ROUTE_CLEARING);
  update(&p,&d,&t,0x18,sign*(JUNCTION_TURN_TARGET_RAD+.01f),15);
  CHECK(p.state==ROUTE_CLEARING);
  clear(&p,&d,&t,sign*(JUNCTION_TURN_TARGET_RAD+.01f));
  CHECK(p.index==1 && p.state==ROUTE_FOLLOW && p.junctions_seen==0);
  CHECK(d<10); /* Short next junction is not gated by old distances. */
  junction(&p,&d,&t,0xff,sign*(JUNCTION_TURN_TARGET_RAD+.01f));
  CHECK(p.state==ROUTE_FINISHED && p.stop_pending);
  p.stop_callback=stop_callback;route_dispatch_stop_callback(&p);route_dispatch_stop_callback(&p);
 }
 CHECK(callbacks==2);
}
static void test_noise_and_straight(void)
{
 RouteStep s[]={{1,6,STRAIGHT,20,0,false,0},{1,6,STOP,20,0,false,0}};
 RoutePlanner p;init(&p,s,2);float d=0;uint32_t t=0;
 update(&p,&d,&t,0x18,0,4);
 update(&p,&d,&t,0,0,2);CHECK(p.junctions_seen==0 && p.state==ROUTE_FOLLOW);
 update(&p,&d,&t,0xff,0,1);update(&p,&d,&t,0x18,0,4);
 CHECK(p.junctions_seen==0);
 d=10000;update(&p,&d,&t,0x18,0,1);CHECK(p.state==ROUTE_FOLLOW && p.index==0);
 junction(&p,&d,&t,0xff,0);CHECK(p.state==ROUTE_EXECUTING);
 update(&p,&d,&t,0xff,0,1);CHECK(p.state==ROUTE_CLEARING);
 clear(&p,&d,&t,0);CHECK(p.index==1);
 /* A gap in sampling breaks candidate continuity. */
 update(&p,&d,&t,0x18,0,4);update(&p,&d,&t,0xff,0,1);
 t+=CONTROL_MAX_INTERVAL_MS+1;update(&p,&d,&t,0xff,0,1);
 CHECK(p.junctions_seen==0);
}
static void test_l_edges_and_confirmation(void)
{
 RouteStep s[]={{1,12,LEFT,40,0,false,0},{1,12,STOP,40,0,false,0}};
 for(unsigned side=0;side<2;++side) {
  RoutePlanner p;init(&p,s,2);float d=0;uint32_t t=0;
  junction(&p,&d,&t,side?0xc0:0x03,0);
  CHECK(p.state==ROUTE_EXECUTING && p.junctions_seen==1);
 }
 RoutePlanner p;init(&p,s,2);float d=0;uint32_t t=0;
 update(&p,&d,&t,0x18,0,4);
 t+=JUNCTION_CENTER_MEMORY_MS+1;
 update(&p,&d,&t,0x03,0,4);CHECK(p.junctions_seen==0);
 init(&p,s,2);d=0;t=0;update(&p,&d,&t,0x18,0,4);
 for(unsigned i=0;i<4;++i){t+=5;route_update(&p,mask(0xff),d,0,t);}
 CHECK(p.junctions_seen==0); /* stationary broad shape cannot confirm */
 t+=JUNCTION_CONFIRM_TIMEOUT_MS;route_update(&p,mask(0xff),d,0,t);
 CHECK(p.junctions_seen==0);
 route_set_speed(&p,NAN,0);route_update(&p,mask(0x18),d,0,t+5);
 CHECK(p.state==ROUTE_FAILED);
}
static void test_entry_brake(void)
{
 RouteStep s[]={{1,6,LEFT,40,0,false,0},{1,6,STOP,40,0,false,0}};
 RoutePlanner p;init(&p,s,2);float d=0;uint32_t t=0;
 route_set_speed(&p,20,7);junction(&p,&d,&t,0xff,0);
 CHECK(p.state==ROUTE_BRAKING && p.junctions_seen==1);
 route_set_speed(&p,5,5);update(&p,&d,&t,0xff,0,1);CHECK(p.state==ROUTE_EXECUTING);
 init(&p,s,2);d=0;t=0;route_set_speed(&p,20,20);junction(&p,&d,&t,0xff,0);
 t+=JUNCTION_ENTRY_BRAKE_TIMEOUT_MS;route_update(&p,mask(0xff),d,0,t);
 CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_ENTRY_SPEED);
 CHECK(!route_resume(&p,ROUTE_BRAKING,d,0,t));
 init(&p,s,2);d=0;t=0;route_set_speed(&p,20,20);junction(&p,&d,&t,0xff,0);
 d+=JUNCTION_ENTRY_BRAKE_MAX_CM+1;route_update(&p,mask(0xff),d,0,t+5);
 CHECK(p.state==ROUTE_FAILED);
}
static void test_angles_and_failures(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) for(unsigned degrees=60;degrees<=360;degrees+=60) {
  RouteStep s[]={{1,6,CUSTOM_TURN,20,mirror?-(float)degrees:(float)degrees,false,0},{1,6,STOP,20,0,false,0}};
  RoutePlanner p;init(&p,s,2);float d=0;uint32_t t=0;float sign=mirror?-1:1;
  junction(&p,&d,&t,0xff,0);float target=route_turn_target_rad(&p);
  NEAR(target,sign*degrees*JUNCTION_TURN_TARGET_RAD/90);
  update(&p,&d,&t,0x18,target*.9f,20);CHECK(p.state==ROUTE_EXECUTING);
  update(&p,&d,&t,0x18,target+sign*.01f,15);CHECK(p.state==ROUTE_CLEARING);
 }
 RouteStep s[]={{1,6,LEFT,20,0,false,0},{1,6,STOP,20,0,false,0}};
 RoutePlanner p;float d=0;uint32_t t=0;init(&p,s,2);junction(&p,&d,&t,0xff,0);
 update(&p,&d,&t,0,1,1);CHECK(p.failure==ROUTE_FAILURE_WRONG_WAY);
 init(&p,s,2);d=0;t=0;junction(&p,&d,&t,0xff,0);
 update(&p,&d,&t,0,-4,1);CHECK(p.failure==ROUTE_FAILURE_TURN_LIMIT);
 init(&p,s,2);d=0;t=0;junction(&p,&d,&t,0xff,0);
 t+=JUNCTION_ACTION_TIMEOUT_MS;route_update(&p,mask(0x18),d,-.5f,t);
 CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT); /* Online timeout cannot waive angle. */
}
/* Fast sweep through the exit: full angle + three adjacent 5 ms center samples,
 * without the former 50 ms delay; index remains locked until clearance. */
static void test_fast_exit(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  RouteStep s[]={{1,50,mirror?RIGHT:LEFT,200,0,false,0},{1,50,STOP,200,0,false,0}};
  RoutePlanner p;float d=0;uint32_t t=0;float sign=mirror?1:-1;
  init(&p,s,2);junction(&p,&d,&t,0xff,0);
  update(&p,&d,&t,0x18,sign*1.76f,2);
  CHECK(p.state==ROUTE_EXECUTING && p.exit_count==2);
  update(&p,&d,&t,0x18,sign*1.80f,1);
  CHECK(p.state==ROUTE_CLEARING && p.index==0 && p.line_exit_confirmed);
  CHECK(p.clear_cm==d);
  clear(&p,&d,&t,sign*1.80f);CHECK(p.state==ROUTE_FOLLOW && p.index==1);
  init(&p,s,2);d=0;t=0;junction(&p,&d,&t,0xff,0);
  update(&p,&d,&t,0x18,sign*1.76f,2);
  update(&p,&d,&t,0,sign*1.76f,1);
  update(&p,&d,&t,0x18,sign*1.76f,2);CHECK(p.state==ROUTE_EXECUTING);
  update(&p,&d,&t,0xff,sign*1.76f,1);
  update(&p,&d,&t,0x18,sign*1.76f,2);CHECK(p.state==ROUTE_EXECUTING);
  t+=CONTROL_MAX_INTERVAL_MS+1;
  update(&p,&d,&t,0x18,sign*1.76f,1);
  CHECK(p.state==ROUTE_EXECUTING && p.component_recovery_active &&
        p.component_recovery_line_samples==0); /* gap breaks recovery confirmation */
  init(&p,s,2);d=0;t=0;junction(&p,&d,&t,0xff,0);
  update(&p,&d,&t,0x18,sign*JUNCTION_TURN_TARGET_RAD*.96f,4);
  CHECK(p.state==ROUTE_CLEARING); /* Ordinary turn: the line completes capture. */
 }
}
/* Once the exit is confirmed, lateral PID motion must not erase junction
 * clearance evidence. The physical junction remains locked until 50 ms/2 cm. */
static void test_clearance_lateral_motion(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  RouteStep s[]={{1,50,mirror?RIGHT:LEFT,200,0,false,0},
                 {4,50,LEFT,200,0,false,0},{1,50,STOP,200,0,false,0}};
  RoutePlanner p;float d=0;uint32_t t=0;float h=mirror?1.84f:-1.84f;
  init(&p,s,3);junction(&p,&d,&t,0xff,0);
  update(&p,&d,&t,0x08,h,3);
  CHECK(p.state==ROUTE_CLEARING && p.line_exit_confirmed && p.index==0);
  /* Sweep through center and both edges, as seen after the real first turn. */
  const uint8_t sweep[]={0x08,0x10,0x20,0x40,0x80,0x04,0x02,0x01};
  for(unsigned i=0;i<40;++i) {
   update(&p,&d,&t,sweep[i%8],h,1);
   CHECK(p.state!=ROUTE_FAILED);
   if(p.index==1) break;
  }
  CHECK(p.index==1 && p.state==ROUTE_FOLLOW && p.junctions_seen==0);
  /* The next non-target junction is passed, not turned or skipped. */
  junction(&p,&d,&t,0xff,h);
  CHECK(p.index==1 && p.junctions_seen==1 && p.passing_junction);
  update(&p,&d,&t,0x04,h,40);
  CHECK(p.index==1 && p.state==ROUTE_CLEARING && p.junctions_seen==1);
  clear(&p,&d,&t,h); /* A passed graphic needs a centered straight exit. */
  CHECK(p.index==1 && p.state==ROUTE_FOLLOW && p.junctions_seen==1);

  /* Loss, ambiguity and a missing sample still break clearance continuity. */
  const uint8_t invalid[]={0x00,0xff,0x81};
  for(unsigned i=0;i<3;++i) {
   init(&p,s,3);d=0;t=0;junction(&p,&d,&t,0xff,0);
   update(&p,&d,&t,0x08,h,3);
   update(&p,&d,&t,0x04,h,20);
   update(&p,&d,&t,invalid[i],h,1);
   CHECK(p.state==ROUTE_CLEARING && !p.exit_candidate_pending && p.exit_count==0);
   update(&p,&d,&t,0x04,h,20);
   CHECK(p.index==0); /* No carry-over of pre-interruption distance. */
   t+=CONTROL_MAX_INTERVAL_MS+1;
   update(&p,&d,&t,0x04,h,1);
   if(i>0) {
    CHECK(p.component_recovery_active && p.component_recovery_line_samples==0);
    update(&p,&d,&t,0x18,h,3); /* graph recovery requires a centered exit */
    CHECK(!p.component_recovery_active);
   }
   CHECK(p.exit_count==1 && p.exit_candidate_cm==d);
   update(&p,&d,&t,0x04,h,40);
   CHECK(p.index==1 && p.state==ROUTE_FOLLOW);
  }
  init(&p,s,3);d=0;t=0;junction(&p,&d,&t,0xff,0);
  update(&p,&d,&t,0x08,h,3);
  t+=JUNCTION_CLEAR_TIMEOUT_MS;
  route_update(&p,mask(0x04),d,h,t);
  CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT && p.index==0);
  init(&p,s,3);d=0;t=0;junction(&p,&d,&t,0xff,0);
  update(&p,&d,&t,0x08,h,3);
  d=p.clear_cm+JUNCTION_CLEAR_MAX_CM+.01f;
  update(&p,&d,&t,0x04,h,1);
  CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT && p.index==0);
 }
}
static void test_braking(void)
{
 BrakingPlan p=braking_plan(40,6,20,0,40,30,2,true);
 CHECK(p.valid);NEAR(p.allowed_speed,sqrtf(36+60*18));
 float previous=p.allowed_speed;
 for(unsigned d=1;d<25;++d) {p=braking_plan(40,6,20,(float)d,20,30,2,true);CHECK(p.valid && p.allowed_speed<=previous);previous=p.allowed_speed;}
 NEAR(p.allowed_speed,6);CHECK(isinf(p.required_decel));
 CHECK(!braking_plan(40,6,20,0,0,0,2,false).valid);
 CHECK(!braking_plan(40,6,NAN,0,0,30,2,true).valid);
 CHECK(braking_plan(40,6,0,1000,0,0,2,false).allowed_speed==40);
 RouteStep s[]={{1,6,LEFT,20,0,false,10},{1,6,STOP,20,0,false,0}};
 RoutePlanner r;route_init(&r,s,2,NULL,NULL);
#if !BRAKING_CALIBRATION_CONFIRMED
 CHECK(!r.valid); /* Never pretend the safe deceleration was measured. */
#endif
}
static void test_search_and_slew(void)
{
 BrakingSlew slew={0};
 float v=braking_slew(&slew,40,.01f,20,100);
 NEAR(v,.01f);NEAR(slew.acceleration,1);
 CHECK(braking_slew(&slew,0,.01f,20,100)==0 && slew.acceleration==0);
#if BRAKING_CALIBRATION_CONFIRMED
 RouteStep s[]={{2,6,LEFT,40,0,false,1},{1,6,STOP,40,0,false,0}};
 RoutePlanner p;init(&p,s,2);float d=0;uint32_t t=0;
 d=500;update(&p,&d,&t,0x18,0,4);CHECK(!p.final_segment_active && !p.search_pending);
 junction(&p,&d,&t,0xff,0);clear(&p,&d,&t,0);
 CHECK(p.final_segment_active && p.junctions_seen==1);
 d=p.final_segment_origin_cm+2;update(&p,&d,&t,0x18,0,1);
 CHECK(p.search_pending && p.state==ROUTE_FOLLOW && p.junctions_seen==1);
 t+=JUNCTION_SEARCH_TIMEOUT_MS;route_update(&p,mask(0x18),d,0,t);
 CHECK(p.failure==ROUTE_FAILURE_SEARCH && p.index==0);
 init(&p,s,2);d=0;t=0;junction(&p,&d,&t,0xff,0);clear(&p,&d,&t,0);
 d=p.final_segment_origin_cm+2;update(&p,&d,&t,0x18,0,1);
 d+=JUNCTION_SEARCH_MAX_CM;update(&p,&d,&t,0x18,0,1);
 CHECK(p.failure==ROUTE_FAILURE_SEARCH);
 #if TURN_APPROACH_SLOWDOWN_ENABLED
 /* Production integration: entering a measured final leg seeds the ramp from current targets. */
 s[0].junction_count=1;s[0].final_segment_length_cm=100;
 LineFollow f;EncoderCounts counts;uint32_t now;start(&f,&counts,&now,s,2);
 f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
 f.base=f.left_target_cm_s=f.right_target_cm_s=20;f.braking_active=false;
 tick(&f,&counts,&now,0,0,0x18,false);
 CHECK(f.base>=20 && f.braking_active);
 f.route.final_segment_origin_cm=f.odometry.distance_cm-101;
 tick(&f,&counts,&now,0,0,0x18,false);
 CHECK(f.base<=s[0].turn_speed && f.route.state==ROUTE_FOLLOW && f.route.junctions_seen==0);
 #endif
#endif
}
static void test_loops(void)
{
 RouteStep s[]={{1,6,STOP,20,0,false,0}};
 RoutePlanner p;init(&p,s,1);p.repeat_count=2;p.repeat_action=STRAIGHT;
 float d=0;uint32_t t=0;junction(&p,&d,&t,0xff,0);
 CHECK(p.completed_laps==1 && p.state==ROUTE_EXECUTING);
 update(&p,&d,&t,0xff,0,1);clear(&p,&d,&t,0);
 CHECK(p.index==0 && p.junctions_seen==0 && p.state==ROUTE_FOLLOW);
 junction(&p,&d,&t,0xff,0);CHECK(p.state==ROUTE_FINISHED && p.completed_laps==2);
 route_start(&p,d,t);CHECK(p.completed_laps==0 && p.junctions_seen==0);
}
static void test_control(void)
{
 RouteStep s[]={{1,6,RIGHT,40,0,false,0},{1,6,STOP,40,0,false,0}};
 LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
 for(unsigned i=0;i<20;++i) tick(&f,&c,&t,1,1,0x30,false);
 CHECK(f.left_target_cm_s>f.right_target_cm_s);
 for(unsigned i=0;i<100 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&t,1,1,0,false);
 CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_LOST);
 for(unsigned i=0;i<10;++i) tick(&f,&c,&t,0,0,0x18,false);
 CHECK(f.state==LINE_STOPPED);
 tick(&f,&c,&t,0,0,0x18,true);CHECK(f.state==LINE_TRACKING && f.route.junctions_seen==0);
 f.route.state=ROUTE_EXECUTING;f.route.phase_ms=t;f.route.entry_heading=f.odometry.heading_rad;
 tick(&f,&c,&t,0,0,0,false);CHECK(f.left_target_cm_s>f.right_target_cm_s);
}
/* Ideal wheel-target following validates controller/FSM cooperation, not real grip or PID tuning. */
static void test_closed_turns(void)
{
 for(unsigned arc=0;arc<2;++arc) for(unsigned mirror=0;mirror<2;++mirror) {
  RouteStep s[]={{1,16,mirror?LEFT:RIGHT,40,0,arc!=0,0},{1,12,STOP,40,0,false,0}};
  LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
  f.odometry.calibration.track_cm=WHEEL_TRACK_CM;
  f.route.state=ROUTE_EXECUTING;f.route.phase_ms=t;f.route.entry_cm=f.odometry.distance_cm;
  f.route.entry_heading=f.route.action_start_heading=f.odometry.heading_rad;
  f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
  float lc=0,rc=0;
  for(unsigned i=0;i<1100 && f.route.index==0;++i) {
   lc+=f.left_target_cm_s*.005f/.01f;rc+=f.right_target_cm_s*.005f/.01f;
   int dl=(int)lc,dr=(int)rc;lc-=dl;rc-=dr;
   float turn=fabsf(f.odometry.heading_rad-f.route.entry_heading);
   tick(&f,&c,&t,dl,dr,turn>=JUNCTION_TURN_TARGET_RAD*.9f?0x18:0,false);
    if(f.state==LINE_STOPPED)
     fprintf(stderr,"turn model stopped: arc=%u mirror=%u reason=%d failure=%d phase=%d turn=%f age=%u\n",
             arc,mirror,f.stop_reason,f.route.failure,f.stop_snapshot.route_state_before_update,
             f.stop_snapshot.turn_rad,f.stop_snapshot.route_phase_age_ms);
   CHECK(f.state!=LINE_STOPPED);
   if(f.route.state==ROUTE_EXECUTING)
    CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=s[0].turn_speed+.001f);
  }
  CHECK(f.route.index==1 && f.route.state==ROUTE_FOLLOW);
 }
}
/* 回归：最后路段的速度斜坡不得被“位置曲线后的实际目标”钳低。
 * 旧逻辑两者相扣，目标速度收敛到 rise/(曲线损失)：pos=1 约 15 cm/s，
 * 车辆无法加速、PWM 太低甚至触发堵转保护。修正后应能继续升到包络。 */
static void test_adjacent_graphic_guards(void)
{
 RouteStep s[]={{1,50,LEFT,50,0,false,0},{2,50,LEFT,200,0,false,0},
                {1,50,STOP,200,0,false,0}};
 /* No corridor, one separated sample, three isolated regions, a gap,
  * no forward progress and interrupted branch evidence cannot hand off. */
 for(unsigned scenario=0;scenario<6;++scenario) {
  RoutePlanner p;init(&p,s,3);float d=0;uint32_t t=0;float h=-1.65f;
  junction(&p,&d,&t,0xff,0);update(&p,&d,&t,0x18,h,3);
  CHECK(p.state==ROUTE_CLEARING && p.line_exit_confirmed);
  if(scenario!=0) update(&p,&d,&t,0x18,h,8);
  if(scenario==3) t+=CONTROL_MAX_INTERVAL_MS+1;
  if(scenario==4) {
   for(unsigned i=0;i<5;++i){t+=5;route_update(&p,mask(0x31),d,h,t);}
  } else if(scenario==5) {
   update(&p,&d,&t,0x31,h,2);update(&p,&d,&t,0,h,1);update(&p,&d,&t,0x31,h,2);
  } else update(&p,&d,&t,scenario==2?0x99:0x31,h,scenario==1?1:3);
  CHECK(p.index==0 && p.state==ROUTE_CLEARING && p.junctions_seen==1);
 }
 /* A stationary new branch expires instead of extending the old action. */
 RoutePlanner p;init(&p,s,3);float d=0;uint32_t t=0;
 junction(&p,&d,&t,0xff,0);update(&p,&d,&t,0x18,-1.65f,3);
 update(&p,&d,&t,0x18,-1.65f,8);
 for(unsigned i=0;i<22 && p.state!=ROUTE_FAILED;++i) {
  t+=5;route_update(&p,mask(0x31),d,-1.65f,t);
 }
 CHECK(p.index==0 && p.failure==ROUTE_FAILURE_CONFIRM_TIMEOUT);
}
static void test_line_exit_guards(void)
{
 const uint8_t rejected[]={0,0xff,0x31,0x01,0x3f};
 for(unsigned i=0;i<sizeof(rejected);++i) {
  RouteStep s[]={{1,50,LEFT,50,0,false,0},{1,50,STOP,50,0,false,0}};
  RoutePlanner p;init(&p,s,2);float d=0;uint32_t t=0;
  junction(&p,&d,&t,0xff,0);update(&p,&d,&t,rejected[i],-.736f,4);
  CHECK(p.state==ROUTE_EXECUTING && !p.line_exit_confirmed);
 }
 for(unsigned custom=0;custom<2;++custom) {
  RouteStep s[]={{1,50,custom?CUSTOM_TURN:LEFT,50,custom?-90.0f:0,false,0},
                 {1,50,STOP,50,0,false,0}};
  RoutePlanner p;init(&p,s,2);float d=0;uint32_t t=0;
  junction(&p,&d,&t,0xff,0);
  update(&p,&d,&t,0x08,custom?-.736f:-.6f,4);
  CHECK(p.state==ROUTE_EXECUTING); /* Custom angle / pre-capture angle stay gated. */
 }
 RouteStep s[]={{1,50,LEFT,50,0,false,0},{1,50,STOP,50,0,false,0}};
 RoutePlanner p;init(&p,s,2);float d=0;uint32_t t=0;
 junction(&p,&d,&t,0xff,0);update(&p,&d,&t,0x08,-.736f,2);
 update(&p,&d,&t,0,-.736f,1);update(&p,&d,&t,0x08,-.736f,2);
 CHECK(p.state==ROUTE_EXECUTING); /* Interrupted samples never add to three. */
 t+=JUNCTION_ACTION_TIMEOUT_MS;route_update(&p,mask(0x08),d,-.736f,t);
 CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT); /* No timeout exemption. */
}
static void test_final_leg_ramp(void)
{
 RouteStep s[]={{1,50,LEFT,200,0,false,1000},{1,50,STOP,200,0,false,0}};
 LineFollow f;EncoderCounts c;uint32_t t;
 start(&f,&c,&t,s,2);
 f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
 float lc=0,rc=0;
 for(unsigned i=0;i<400;++i) {
  lc+=f.left_target_cm_s*.005f/.01f;rc+=f.right_target_cm_s*.005f/.01f;
  int dl=(int)lc,dr=(int)rc;lc-=dl;rc-=dr;
  tick(&f,&c,&t,dl,dr,0x10,false); /* 位置 +1：旧逻辑会卡在约15 cm/s。 */
  CHECK(f.state!=LINE_STOPPED);
  CHECK(f.left_target_cm_s>=0 && f.right_target_cm_s>=0);
 }
 /* 仍须能摆脱旧的约15 cm/s衰减反馈，同时服从最后一段50 cm/s上限。 */
 CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)>40);
#if TURN_APPROACH_SLOWDOWN_ENABLED
 CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=JUNCTION_APPROACH_SPEED_CM_S+.001f);
#endif
 CHECK(f.odometry.distance_cm>50);
#if TURN_APPROACH_SLOWDOWN_ENABLED
 /* 包络下降仍要即时生效，不能因斜坡状态领先而超速冲向路口。 */
 f.route.final_segment_origin_cm=f.odometry.distance_cm-999;
 tick(&f,&c,&t,0,0,0x10,false);
 CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=s[0].turn_speed+.001f);
#endif
}
int main(void)
{
 test_counts();test_noise_and_straight();test_l_edges_and_confirmation();test_entry_brake();test_angles_and_failures();test_fast_exit();test_clearance_lateral_motion();test_braking();test_search_and_slew();test_loops();test_control();test_closed_turns();test_adjacent_graphic_guards();test_line_exit_guards();test_final_leg_ramp();
 printf("PASS: count navigation, short junctions, angle/entry/STOP safety, braking (%u checks)\n",checks);return 0;
}
