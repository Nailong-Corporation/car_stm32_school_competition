#define main historical_navigation_main
#include "test_navigation.c"
#undef main

static void test_distance_or_count(void)
{
 RouteStep s[]={{4,12,LEFT,200,0,false,70},{1,12,STOP,200,0,false,0}};
 RoutePlanner p;float d=0;uint32_t t=0;init(&p,s,2);
 junction(&p,&d,&t,0xff,0);CHECK(p.passing_junction && p.junctions_seen==1);
 clear(&p,&d,&t,0);NEAR(p.origin_cm,0);NEAR(p.final_segment_origin_cm,0);
 d=71;update(&p,&d,&t,0x18,0,5);
 CHECK(p.state==ROUTE_FOLLOW && p.search_pending && p.index==0);
 update(&p,&d,&t,0xff,0,1);CHECK(p.state==ROUTE_CANDIDATE);
 update(&p,&d,&t,0x18,0,4);CHECK(p.state==ROUTE_FOLLOW);
 junction(&p,&d,&t,0xff,0);
 CHECK(p.state==ROUTE_EXECUTING && p.junctions_seen==2 && !p.passing_junction);
 /* Count wins before distance, while distance alone never acts. */
 s[0].junction_count=1;s[0].segment_length_cm=200;
 init(&p,s,2);d=0;t=0;junction(&p,&d,&t,0xff,0);
 CHECK(p.state==ROUTE_EXECUTING && d<200);
 /* Zero ignores count: even many confirmed nodes cannot execute early. */
 s[0].junction_count=0;s[0].segment_length_cm=70;
 init(&p,s,2);d=0;t=0;
 for(unsigned i=0;i<4;++i) {
  update(&p,&d,&t,0x18,0,4);update(&p,&d,&t,0x99,0,4);
  CHECK(p.passing_junction && p.junctions_seen==i+1 && p.index==0);
  clear(&p,&d,&t,0);CHECK(p.state==ROUTE_FOLLOW);NEAR(p.origin_cm,0);
 }
 d=71;update(&p,&d,&t,0x18,0,4);CHECK(p.state==ROUTE_FOLLOW);
 junction(&p,&d,&t,0xff,0);CHECK(p.state==ROUTE_EXECUTING && !p.passing_junction);
 s[0].segment_length_cm=0;route_init(&p,s,2,NULL,NULL);CHECK(!p.valid);
 /* Distance-first STOP must still see a node and dispatch only once. */
 RouteStep stop[]={{0,12,STOP,200,0,false,65}};
 init(&p,stop,1);p.stop_callback=stop_callback;callbacks=0;d=66;t=0;
 update(&p,&d,&t,0x18,0,4);CHECK(p.state==ROUTE_FOLLOW && callbacks==0);
 junction(&p,&d,&t,0xff,0);CHECK(p.state==ROUTE_FINISHED);
 route_dispatch_stop_callback(&p);route_dispatch_stop_callback(&p);CHECK(callbacks==1);
}
static void test_speed_and_graphic(void)
{
 RouteStep s[]={{4,12,LEFT,200,0,false,100},{1,12,STOP,200,0,false,0}};
 RoutePlanner p;init(&p,s,2);NEAR(route_cruise_speed(&p),60);
 s[0].segment_length_cm=0;init(&p,s,2);NEAR(route_cruise_speed(&p),60);
 s[0].segment_length_cm=100.01f;init(&p,s,2);NEAR(route_cruise_speed(&p),200);
 s[0].segment_length_cm=1000;
 LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
 f.startup_elapsed_s=1;f.base=f.left_target_cm_s=f.right_target_cm_s=120;
 f.braking_active=false;
 for(unsigned i=0;i<4;++i) tick(&f,&c,&t,60,60,0x18,false);
 CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)>COMPONENT_RECOVERY_SPEED_CM_S);
 tick(&f,&c,&t,60,60,0x99,false);
 CHECK(f.route.state==ROUTE_CANDIDATE && f.state==LINE_JUNCTION);
 CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=COMPONENT_RECOVERY_SPEED_CM_S+.001f);
 for(unsigned i=0;i<3;++i) tick(&f,&c,&t,60,60,0x99,false);
 CHECK(f.route.junctions_seen==1 && f.route.passing_junction && f.route.state==ROUTE_CLEARING);
 CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=COMPONENT_RECOVERY_SPEED_CM_S+.001f);
 float origin=f.route.origin_cm;
 for(unsigned i=0;i<12;++i) tick(&f,&c,&t,60,60,0x18,false);
 CHECK(f.route.state==ROUTE_FOLLOW && f.route.index==0);NEAR(f.route.origin_cm,origin);
 /* The long-segment envelope still slows toward its measured end. */
 f.route.final_segment_origin_cm=f.odometry.distance_cm-1000;
 f.route.origin_cm=f.route.final_segment_origin_cm;
 tick(&f,&c,&t,1,1,0x18,false);
 CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=12.001f);
 CHECK(f.state!=LINE_STOPPED && f.route.state==ROUTE_FOLLOW);
}
static void test_soft_recovery_and_hard_stops(void)
{
 const RouteFailure soft[]={ROUTE_FAILURE_CONFIRM_TIMEOUT,ROUTE_FAILURE_ACTION_TIMEOUT,
  ROUTE_FAILURE_TURN_LIMIT,ROUTE_FAILURE_WRONG_WAY,ROUTE_FAILURE_SEARCH,
  ROUTE_FAILURE_WINDOW,ROUTE_FAILURE_ENTRY_SPEED};
 const RouteFailure hard[]={ROUTE_FAILURE_INVALID,ROUTE_FAILURE_EXIT_LOST,
  ROUTE_FAILURE_CLEAR_LINE_LOST,ROUTE_FAILURE_EXIT_RECAPTURE};
 RouteStep s[]={{4,12,LEFT,200,0,false,70},{1,12,STOP,200,0,false,0}};
 for(unsigned i=0;i<sizeof(soft)/sizeof(soft[0]);++i) {
  LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
  f.route.state=ROUTE_FAILED;f.route.failure=soft[i];f.route.junctions_seen=2;
  float origin=f.route.origin_cm;
  tick(&f,&c,&t,4,4,0x18,false);
  CHECK(f.state!=LINE_STOPPED && f.route.state==ROUTE_RECOVERING);
  CHECK(f.route.navigation_warning==soft[i] && f.route.navigation_recoveries==1);
  for(unsigned j=0;j<60 && f.route.state==ROUTE_RECOVERING;++j)
   tick(&f,&c,&t,4,4,0x18,false);
  CHECK(f.route.state==ROUTE_FOLLOW && f.route.index==0 && f.route.junctions_seen==2);
  NEAR(f.route.origin_cm,origin);
 }
 for(unsigned i=0;i<sizeof(hard)/sizeof(hard[0]);++i) {
  LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
  f.route.state=ROUTE_FAILED;f.route.failure=hard[i];tick(&f,&c,&t,0,0,0x18,false);
  CHECK(f.state==LINE_STOPPED && !f.command.enabled);
 }
 /* An otherwise sufficient line on the deadline cannot renew recovery. */
 for(unsigned boundary=0;boundary<2;++boundary) {
  RoutePlanner p;init(&p,s,2);p.state=ROUTE_FAILED;p.failure=ROUTE_FAILURE_ACTION_TIMEOUT;
  CHECK(route_recover_navigation(&p,0,0,0));
  p.exit_count=LINE_CONFIRM_SAMPLES;p.exit_candidate_pending=true;
  p.exit_candidate_ms=0;p.exit_candidate_cm=0;
  uint32_t expiry=boundary?JUNCTION_COUNT_CLEAR_MS:NAVIGATION_RECOVERY_MS;
  p.last_sample_ms=expiry-5;
  route_update(&p,mask(0x18),boundary?NAVIGATION_RECOVERY_CM:2.1f,0,expiry);
  CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_EXIT_LOST);
 }
 LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
 f.route.state=ROUTE_FAILED;f.route.failure=ROUTE_FAILURE_ACTION_TIMEOUT;
 for(unsigned i=0;i<300 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&t,1,1,0,false);
 CHECK(f.state==LINE_STOPPED && f.route.failure==ROUTE_FAILURE_EXIT_LOST);
 tick(&f,&c,&t,0,0,0x18,false);CHECK(f.state==LINE_STOPPED);
 /* Task timing remains locked even during a navigation retreat. */
 start(&f,&c,&t,s,2);f.route.state=ROUTE_FAILED;f.route.failure=ROUTE_FAILURE_TURN_LIMIT;
 tick(&f,&c,&t,1,1,0x18,false);
 line_follow_update(&f,mask(0x18),c,t+20,.020f,false);
 CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_FAULT);
}
int main(void)
{
 test_distance_or_count();test_speed_and_graphic();test_soft_recovery_and_hard_stops();
 printf("PASS: total distance/count/zero, short-long speed, graphic passage, soft recovery/hard stops (%u checks)\n",checks);
 return 0;
}
