#define main previous_navigation_main
#include "test_navigation.c"
#undef main
#include "../src/route_table.c"
#include <string.h>

/* Real table, controller/odometry/speed loops; synthetic encoder increments
 * model feedback, not physical inertia. Observations are already classified. */
static void braking_entry(LineFollow *f,EncoderCounts *c,uint32_t *t)
{
 start(f,c,t,route,route_count);
 for(unsigned i=0;i<30;++i) tick(f,c,t,10,10,0x18,false);
 CHECK(f->odometry.left_speed_cm_s>route_entry_speed_limit(&f->route));
 tick(f,c,t,10,10,0x1f,false);
 CHECK(f->route.state==ROUTE_CANDIDATE && f->abnormal);
 /* The current table uses CUSTOM_TURN -90 for the first action, so the
  * two-frame LEFT/RIGHT early confirmation does not apply; use the normal
  * three-frame strong-candidate gate. */
 for(unsigned i=1;i<JUNCTION_CONFIRM_SAMPLES;++i) tick(f,c,t,10,10,0x1f,false);
 CHECK(f->route.state==ROUTE_BRAKING && f->route.junctions_seen==1);
 CHECK(f->left_target_cm_s==0 && f->right_target_cm_s==0 && !f->command.enabled);
}
static void test_budget(void)
{
 LineFollow f;EncoderCounts c;uint32_t t;braking_entry(&f,&c,&t);
 uint32_t entered=t;
 /* Coast slowly enough to stay inside 10cm, but above 16cm/s for >400ms. */
 for(unsigned i=0;i<90;++i) {
  tick(&f,&c,&t,10,10,0,false);
  CHECK(f.state!=LINE_STOPPED && f.route.state==ROUTE_BRAKING);
  CHECK(f.left_target_cm_s==0 && f.right_target_cm_s==0 && !f.command.enabled);
 }
 CHECK(t-entered>INTERFERENCE_TIMEOUT_MS);
 /* Original 10cm bound still aborts into recovery, without turning or recounting. */
 for(unsigned i=0;i<20 && f.route.state==ROUTE_BRAKING;++i) tick(&f,&c,&t,10,10,0,false);
 CHECK(f.route.state==ROUTE_RECOVERING && f.route.navigation_warning==ROUTE_FAILURE_ENTRY_SPEED);
 CHECK(f.route.index==0 && f.route.junctions_seen==1 && !f.route.line_exit_confirmed);
}
static void test_loss_ready(void)
{
 LineFollow f;EncoderCounts c;uint32_t t;braking_entry(&f,&c,&t);
 /* Once stopped beyond the visible branch, zero targets cannot reacquire it.
  * Abort the maneuver into existing bounded recovery; never blind-turn. */
 for(unsigned i=0;i<30 && f.route.state==ROUTE_BRAKING;++i) tick(&f,&c,&t,0,0,0,false);
 CHECK(f.route.state==ROUTE_RECOVERING && f.state!=LINE_STOPPED);
 CHECK(f.route.navigation_warning==ROUTE_FAILURE_ENTRY_SPEED);
 CHECK(f.route.index==0 && f.route.junctions_seen==1 && !f.route.line_exit_confirmed);
 uint32_t recovered=t;
 while(t-recovered<NAVIGATION_RECOVERY_MS-5U) {
  /* 2 counts / 5ms = 4cm/s in this synthetic calibration, below 12cm. */
  tick(&f,&c,&t,2,2,0,false);
  CHECK(f.state!=LINE_STOPPED && f.route.state==ROUTE_RECOVERING);
 }
 /* Deadline wins even against a late valid sample. */
 tick(&f,&c,&t,2,2,0x18,false);
 CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_ROUTE);
 CHECK(f.route.failure==ROUTE_FAILURE_EXIT_LOST);
 CHECK(!f.command.enabled);
}
static void test_recovery_bounds(void)
{
 LineFollow f;EncoderCounts c;uint32_t t;braking_entry(&f,&c,&t);
 for(unsigned i=0;i<30 && f.route.state==ROUTE_BRAKING;++i) tick(&f,&c,&t,0,0,0,false);
 CHECK(f.route.state==ROUTE_RECOVERING);
 uint32_t recovered=t;
 /* Forward feedback can reach the independent 12cm bound before 1200ms. */
 for(unsigned i=0;i<130 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&t,10,10,0,false);
 CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_ROUTE);
 CHECK(f.route.failure==ROUTE_FAILURE_EXIT_LOST && t-recovered<NAVIGATION_RECOVERY_MS);
 CHECK(!f.command.enabled);
 /* A stationary recovery is a stall, not permission to bypass feedback. */
 braking_entry(&f,&c,&t);
 for(unsigned i=0;i<30 && f.route.state==ROUTE_BRAKING;++i) tick(&f,&c,&t,0,0,0,false);
 CHECK(f.route.state==ROUTE_RECOVERING);
 for(unsigned i=0;i<150 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&t,0,0,0,false);
 CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_ENCODER);
 CHECK(f.stop_snapshot.speed_fault==SPEED_FAULT_STALL && !f.command.enabled);
 for(unsigned i=0;i<5;++i) tick(&f,&c,&t,0,0,0x18,i==4);
 CHECK(f.state==LINE_STOPPED && f.start_result==LINE_START_REJECTED_FAULT);
}
static void test_reacquire_and_guards(void)
{
 LineFollow f;EncoderCounts c;uint32_t t;braking_entry(&f,&c,&t);
 for(unsigned i=0;i<30 && f.route.state==ROUTE_BRAKING;++i) tick(&f,&c,&t,0,0,0,false);
 CHECK(f.route.state==ROUTE_RECOVERING);
 float origin=f.route.origin_cm;
 for(unsigned i=0;i<80 && f.route.state==ROUTE_RECOVERING;++i) tick(&f,&c,&t,5,5,0x18,false);
 CHECK(f.route.state==ROUTE_FOLLOW && f.route.index==0 && f.route.junctions_seen==1);
 NEAR(f.route.origin_cm,origin);
 for(unsigned i=0;i<5;++i) tick(&f,&c,&t,5,5,0x18,false);
 CHECK(f.route.state==ROUTE_FOLLOW); /* Reacquisition alone does not turn. */
 /* No regression to measured entry gate: visible branch, speed still high. */
 braking_entry(&f,&c,&t);
 for(unsigned i=0;i<10;++i) {
  tick(&f,&c,&t,10,10,0x1f,false);CHECK(f.route.state==ROUTE_BRAKING);
 }
 for(unsigned i=0;i<30 && f.route.state==ROUTE_BRAKING;++i) tick(&f,&c,&t,0,0,0x1f,false);
 CHECK(f.route.state==ROUTE_EXECUTING && f.route.junctions_seen==1);
 CHECK(f.left_target_cm_s<f.right_target_cm_s);
 /* Unconfirmed loss still uses ordinary 400ms budget and never counts. */
 start(&f,&c,&t,route,route_count);
 for(unsigned i=0;i<90 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&t,0,0,0,false);
 CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_LOST && f.route.junctions_seen==0);
 /* Feedback faults remain locked even inside entry braking. */
 braking_entry(&f,&c,&t);tick(&f,&c,&t,30000,0,0,false);
 CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_ENCODER);
}
static void test_filtered_entry(void)
{
 LineFollow f;EncoderCounts c={0};uint32_t t=0;GraySensor sensor;
 line_follow_init(&f,calibration(),route,route_count,NULL,NULL);
 f.route.repeat_count=1;gray_sensor_init(&sensor);
 for(unsigned i=0;i<35;++i) {
  c.left+=10;c.right+=10;t+=5;
  GrayObservation o=gray_sensor_update(&sensor,raw_for_line(0x18),true,0);
  CHECK(o.line_mask==0x18);
  line_follow_update(&f,o,c,t,.005f,i==2);
 }
 CHECK(f.state==LINE_TRACKING && f.route.state==ROUTE_FOLLOW);
 /* CUSTOM_TURN uses the normal three-frame strong gate; the sensor filter
  * also delays the first 0x1f by one period, so the entry needs one extra. */
 for(unsigned i=0;i<4;++i) {
  c.left+=10;c.right+=10;t+=5;
  GrayObservation o=gray_sensor_update(&sensor,raw_for_line(0x1f),true,0);
  CHECK(o.line_mask==(i==0?0x18:0x1f));
  line_follow_update(&f,o,c,t,.005f,false);
  CHECK(f.route.state==(i==0?ROUTE_FOLLOW:i<3?ROUTE_CANDIDATE:ROUTE_BRAKING));
 }
 CHECK(f.route.junctions_seen==1 && f.left_target_cm_s==0 && f.right_target_cm_s==0);
 /* Sensor majority also delays loss by one period; it does not renew node proof. */
 for(unsigned i=0;i<30 && f.route.state==ROUTE_BRAKING;++i) {
  t+=5;
  GrayObservation o=gray_sensor_update(&sensor,raw_for_line(0),true,0);
  line_follow_update(&f,o,c,t,.005f,false);
 }
 CHECK(f.route.state==ROUTE_RECOVERING && f.route.junctions_seen==1);
 CHECK(f.state!=LINE_STOPPED && f.route.index==0);
}
int main(int argc,char **argv)
{
 if(argc<2 || strcmp(argv[1],"budget")==0) test_budget();
 if(argc<2 || strcmp(argv[1],"loss")==0) test_loss_ready();
 if(argc<2 || strcmp(argv[1],"guards")==0) test_reacquire_and_guards();
 if(argc<2 || strcmp(argv[1],"bounds")==0) test_recovery_bounds();
 if(argc<2 || strcmp(argv[1],"filtered")==0) test_filtered_entry();
 printf("PASS: confirmed entry loss/budget/reacquisition/guards (%u checks)\n",checks);
 return 0;
}
