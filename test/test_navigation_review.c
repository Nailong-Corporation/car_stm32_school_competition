#define main previous_navigation_main
#include "test_navigation.c"
#undef main
#include <string.h>

static void test_action_anchor(void)
{
 for(unsigned mode=0;mode<5;++mode) {
  RouteStep s[]={{1,12,mode==1?STRAIGHT:mode==2?HEXAGON:mode==3?RIGHT:mode==4?CUSTOM_TURN:LEFT,200,
                 mode==2?-300:mode==4?-120:0,false,0, 0},{0,12,STOP,200,0,false,5, 0}};
  RoutePlanner p;float d=0;uint32_t t=0;init(&p,s,2);
  junction_approach(&p,&d,&t,0xff,0);CHECK(p.state==ROUTE_EXECUTING);
  float heading=mode==1?0:mode==2?-6.0f:mode==3?1.6f:mode==4?-2.2f:-1.6f;
  if(mode==2) {
   /* The HEXAGON sub-FSM must earn its own exit before public clearance. */
   p.hexagon.state=HEXAGON_EXITING;p.hexagon.phase_ms=t;
   p.hexagon.phase_cm=d;p.hexagon.line_ms=t;p.hexagon.line_cm=d;
   for(unsigned i=0;i<30 && p.state==ROUTE_EXECUTING;++i) update(&p,&d,&t,0x18,heading,1);
  } else update(&p,&d,&t,0x18,heading,3);
  CHECK(p.state==ROUTE_CLEARING);
  float anchor=d;
  clear(&p,&d,&t,heading);CHECK(p.index==1 && p.state==ROUTE_FOLLOW);
  NEAR(p.origin_cm,anchor);NEAR(p.final_segment_origin_cm,anchor);
  /* Five cm after action completion includes the already traveled clearance. */
  d=anchor+5.05f;update(&p,&d,&t,0x18,heading,4);
  CHECK(p.search_pending && p.state==ROUTE_FOLLOW);
  junction_approach(&p,&d,&t,0xff,heading);CHECK(p.state==ROUTE_FINISHED);
 }
}
static void test_final_lap_preview(void)
{
 RouteStep stop[]={{1,50,STOP,200,0,false,101, 0}};
 RoutePlanner p;init(&p,stop,1);p.repeat_count=2;p.repeat_action=LEFT;
 float d=0;uint32_t t=0;junction_approach(&p,&d,&t,0xff,0);
 update(&p,&d,&t,0x18,-1.6f,3);CHECK(p.state==ROUTE_CLEARING && p.lap_boundary);
 NEAR(route_entry_speed_limit(&p),JUNCTION_SETTLE_SPEED_CM_S);NEAR(route_planning_entry_speed_limit(&p),50);
 /* The next final STOP is on the same row but uses the next lap's contract. */
 NEAR(route_planning_origin(&p),p.clear_cm);
}
static void test_clearance_envelope(void)
{
 RouteStep s[]={{5,12,LEFT,200,0,false,101, 0},{1,12,STOP,200,0,false,0, 0}};
 LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
 f.startup_elapsed_s=1;f.base=f.left_target_cm_s=f.right_target_cm_s=100;
 /* Seed a legitimate intermediate clearance close to the total distance end. */
 f.route.state=ROUTE_CLEARING;f.route.passing_junction=true;f.route.junctions_seen=1;
 f.route.phase_ms=f.route.capture_line_ms=t;
 f.route.clear_cm=f.route.capture_line_cm=f.odometry.distance_cm;
 f.route.origin_cm=f.route.final_segment_origin_cm=f.odometry.distance_cm-100.5f;
 f.braking_active=false;
 tick(&f,&c,&t,10,10,0x99,false);
 CHECK(f.route.component_recovery_active && f.braking_active);
 CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=12.001f);
 for(unsigned i=0;i<7;++i) {
  tick(&f,&c,&t,10,10,0x99,false);
  CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=12.001f);
 }
 CHECK(f.route.search_pending && f.route.state==ROUTE_CLEARING);
 for(unsigned i=0;i<30 && f.route.state==ROUTE_CLEARING;++i) {
  tick(&f,&c,&t,10,10,0x18,false);
  CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=12.001f);
 }
 CHECK(f.route.state==ROUTE_FOLLOW && f.route.search_pending && f.route.index==0);
 /* Target turn clearance previews the next segment from its exit point. */
 RouteStep turns[]={{1,12,LEFT,200,0,false,0, 0},{0,12,STOP,200,0,false,101, 0}};
 start(&f,&c,&t,turns,2);f.startup_elapsed_s=1;
 f.route.state=ROUTE_EXECUTING;f.route.phase_ms=t;
 f.route.entry_heading=f.route.action_start_heading=1.6f;
 for(unsigned i=0;i<3;++i) tick(&f,&c,&t,1,1,0x18,false);
 CHECK(f.route.state==ROUTE_CLEARING);
 NEAR(f.braking_origin_cm,f.route.clear_cm);CHECK(f.braking_active);
}
static void test_recovery_derivative(void)
{
 RouteStep s[]={{3,12,LEFT,200,0,false,70, 0},{1,12,STOP,200,0,false,0, 0}};
 LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
 f.route.state=ROUTE_FAILED;f.route.failure=ROUTE_FAILURE_ACTION_TIMEOUT;
 tick(&f,&c,&t,1,1,0x18,false);CHECK(f.route.state==ROUTE_RECOVERING);
 tick(&f,&c,&t,1,1,0x10,false);
 CHECK(f.pid.primed && f.pid.derivative>0);
 float first=f.pid.derivative;
 tick(&f,&c,&t,1,1,0x20,false);CHECK(f.pid.derivative>first);
 tick(&f,&c,&t,1,1,0,false);CHECK(!f.pid.primed);
}
static void test_reliable_recovery(void)
{
 RouteStep s[]={{3,12,LEFT,200,0,false,70, 0},{1,12,STOP,200,0,false,0, 0}};
 RoutePlanner p;init(&p,s,2);p.state=ROUTE_FAILED;p.failure=ROUTE_FAILURE_TURN_LIMIT;
 CHECK(route_recover_navigation(&p,0,0,0));
 /* No displacement: reliable line cannot be classified as sustained loss.
  * Production speed feedback independently detects an actual stalled wheel. */
 for(uint32_t t=5;t<=NAVIGATION_RECOVERY_MS+100;t+=5) {
  route_update(&p,mask(0x18),0,0,t);
  CHECK(p.state==ROUTE_RECOVERING && p.failure==ROUTE_FAILURE_NONE);
 }
 /* Continuous valid samples may travel >12 cm before the 50 ms unlock time,
  * e.g. residual high wheel speed; this is not a line-loss event. */
 init(&p,s,2);p.state=ROUTE_FAILED;p.failure=ROUTE_FAILURE_TURN_LIMIT;
 CHECK(route_recover_navigation(&p,0,0,0));
 for(unsigned i=1;i<=8;++i) {
  route_update(&p,mask(0x18),(float)i*2.5f,0,i*5);
  CHECK(p.state==ROUTE_RECOVERING && p.failure==ROUTE_FAILURE_NONE);
 }
 /* After genuine loss the budget is measured from last confirmed line,
  * not refreshed by isolated one/two-frame appearances. */
 uint32_t last=p.capture_line_ms;
 for(uint32_t t=45;t<last+NAVIGATION_RECOVERY_MS;t+=5) {
  uint8_t m=((t/5)%3)?0x18:0;
  route_update(&p,mask(m),20,0,t);
  CHECK(p.state!=ROUTE_FAILED);
 }
 route_update(&p,mask(0x18),20,0,last+NAVIGATION_RECOVERY_MS);
 CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_EXIT_LOST);

}
int main(int argc,char **argv)
{
 if(argc<2 || strcmp(argv[1],"anchor")==0) {test_action_anchor();test_final_lap_preview();}
 if(argc<2 || strcmp(argv[1],"braking")==0) test_clearance_envelope();
 if(argc<2 || strcmp(argv[1],"pid")==0) test_recovery_derivative();
 if(argc<2 || strcmp(argv[1],"reliable")==0) test_reliable_recovery();
 printf("PASS: review anchors/envelope/recovery PID/reliable-line semantics (%u checks)\n",checks);
 return 0;
}
