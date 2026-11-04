#define main navigation_regressions
#include "test_navigation.c"
#undef main

static RouteStep hex_steps[]={{1,12,HEXAGON,40,300,false,0},{1,12,STOP,40,0,false,0}};
static void ring(RoutePlanner *p,float *d,uint32_t *t,float sign)
{
 hex_steps[0].angle_deg=sign*300;*d=0;*t=0;init(p,hex_steps,2);
 route_set_speed(p,10,10);
 junction(p,d,t,sign>0?0xf8:0x1f,0);
 CHECK(p->state==ROUTE_EXECUTING && p->hexagon.state==HEXAGON_ENTERING);
 CHECK(p->junctions_seen==1);
 update(p,d,t,0x18,sign*.5f,4);
 CHECK(p->hexagon.state==HEXAGON_FOLLOWING);
}
static void exit_branch(RoutePlanner *p,float *d,uint32_t *t,float sign)
{
 float h=route_turn_target_rad(p)+sign*.01f;
 update(p,d,t,0x18,h,4);
 CHECK(p->state==ROUTE_EXECUTING && p->hexagon.state==HEXAGON_FOLLOWING);
 update(p,d,t,sign>0?0x1f:0xf8,h,3);
 CHECK(p->hexagon.state==HEXAGON_EXITING && p->state==ROUTE_EXECUTING);
 CHECK(p->junctions_seen==1 && p->index==0);
}
static void test_gates_and_mirror(void)
{
 for(unsigned side=0;side<2;++side) {
  float sign=side?1:-1;RoutePlanner p;float d;uint32_t t;ring(&p,&d,&t,sign);
  /* Center line for hundreds of ms is the ring, not an exit. */
  update(&p,&d,&t,0x18,sign*4.0f,80);
  CHECK(p.state==ROUTE_EXECUTING && p.hexagon.state==HEXAGON_FOLLOWING);
  float target=route_turn_target_rad(&p);
  update(&p,&d,&t,0x18,target+sign*.01f,30);
  CHECK(p.state==ROUTE_EXECUTING && !p.line_exit_confirmed);
  exit_branch(&p,&d,&t,sign);
  update(&p,&d,&t,0x18,target+sign*.01f,10);
  CHECK(p.state==ROUTE_EXECUTING); /* frames alone do not meet 100 ms */
  update(&p,&d,&t,0x80,target+sign*.01f,1);
  CHECK(!p.hexagon.center_pending);
  update(&p,&d,&t,0x18,target+sign*.01f,24);
  CHECK(p.state==ROUTE_CLEARING && p.hexagon.state==HEXAGON_DONE && p.line_exit_confirmed);
  clear(&p,&d,&t,target+sign*.01f);
  CHECK(p.index==1 && p.junctions_seen==0 && p.hexagon.state==HEXAGON_OFF);
  junction(&p,&d,&t,0xff,target+sign*.01f);
  CHECK(p.state==ROUTE_FINISHED);
 }
 /* A branch before the angle cannot be banked for later. */
 RoutePlanner p;float d;uint32_t t;ring(&p,&d,&t,1);
 update(&p,&d,&t,0x1f,5.1f,4);
 CHECK(p.hexagon.state==HEXAGON_FOLLOWING && !p.line_exit_confirmed);
 update(&p,&d,&t,0x1f,5.3f,4);
 CHECK(p.hexagon.state==HEXAGON_FOLLOWING && !p.hexagon.exit_armed);
 update(&p,&d,&t,0x18,5.3f,40);
 CHECK(p.state==ROUTE_EXECUTING && p.hexagon.state==HEXAGON_FOLLOWING);
 /* Three isolated regions cannot select an exit, even beyond target. */
 ring(&p,&d,&t,1);update(&p,&d,&t,0x99,5.3f,4);
 CHECK(p.hexagon.state==HEXAGON_FOLLOWING && p.component_recovery_active);
}
static void test_loss_graph_and_deadlines(void)
{
 RoutePlanner p;float d;uint32_t t;ring(&p,&d,&t,1);
 update(&p,&d,&t,0x3b,1,1);
 CHECK(p.component_recovery_active && p.component_recovery_used);
 uint32_t origin=p.component_recovery_ms;
 update(&p,&d,&t,0,1,90); /* beyond ordinary loss, within fixed graphic budget */
 CHECK(p.state==ROUTE_EXECUTING && p.component_recovery_ms==origin);
 update(&p,&d,&t,0x18,1,3);
 CHECK(!p.component_recovery_active && p.hexagon.state==HEXAGON_FOLLOWING);
 update(&p,&d,&t,0x3b,1,1);CHECK(!p.component_recovery_active);
 update(&p,&d,&t,0,1,80);
 CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_EXIT_LOST);
 ring(&p,&d,&t,1);update(&p,&d,&t,0xff,1,1);
 t=p.component_recovery_ms+COMPONENT_RECOVERY_TIMEOUT_MS;
 route_update(&p,mask(0x18),d,1,t);
 CHECK(p.failure==ROUTE_FAILURE_EXIT_RECAPTURE);
 /* Branch-shaped noise cannot bypass the active graphic recovery deadline. */
 ring(&p,&d,&t,1);update(&p,&d,&t,0x18,5.3f,4);
 update(&p,&d,&t,0x99,5.3f,1);CHECK(p.component_recovery_active);
 p.have_approach_center=true;p.approach_center_ms=t;p.approach_center_cm=d;
 t=p.component_recovery_ms+COMPONENT_RECOVERY_TIMEOUT_MS;
 p.last_sample_ms=t-5;
 route_update(&p,mask(0x1f),d,5.3f,t);
 CHECK(p.failure==ROUTE_FAILURE_EXIT_RECAPTURE);
 ring(&p,&d,&t,1);update(&p,&d,&t,0,1,80);
 CHECK(p.failure==ROUTE_FAILURE_EXIT_LOST && !p.component_recovery_used);
 ring(&p,&d,&t,1);update(&p,&d,&t,0,1,1);
 d=p.hexagon.line_cm+INTERFERENCE_MAX_DISTANCE_CM;
 route_update(&p,mask(0x18),d,1,t+5);CHECK(p.failure==ROUTE_FAILURE_EXIT_LOST);
 /* The regular 3/4.5 s turn budget must not terminate an on-line lap. */
 ring(&p,&d,&t,1);update(&p,&d,&t,0x18,1,1000);
 CHECK(p.state==ROUTE_EXECUTING && t>4500);
 t=p.hexagon.action_ms+HEXAGON_ACTION_TIMEOUT_MS;
 route_update(&p,mask(0x18),d,1,t);CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
 ring(&p,&d,&t,1);d=p.entry_cm+HEXAGON_ACTION_MAX_CM;
 route_update(&p,mask(0x18),d,1,t+5);CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
 ring(&p,&d,&t,1);exit_branch(&p,&d,&t,1);
 update(&p,&d,&t,0x80,5.3f,610);
 CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT); /* valid edge never proves exit */
 ring(&p,&d,&t,1);update(&p,&d,&t,0x18,-.36f,1);
 CHECK(p.failure==ROUTE_FAILURE_WRONG_WAY);
 ring(&p,&d,&t,-1);update(&p,&d,&t,0x18,-8,1);
 CHECK(p.failure==ROUTE_FAILURE_TURN_LIMIT);
 ring(&p,&d,&t,1);route_set_speed(&p,NAN,0);update(&p,&d,&t,0x18,1,1);
 CHECK(p.failure==ROUTE_FAILURE_INVALID);
}
static void test_validation_and_entry(void)
{
 RoutePlanner p;RouteStep s[]={{1,12,HEXAGON,40,300,false,0},{1,12,STOP,40,0,false,0}};
 const float invalid[]={0,59,721,NAN,INFINITY};
 for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);++i) {
  s[0].angle_deg=invalid[i];route_init(&p,s,2,NULL,NULL);CHECK(!p.valid);
 }
 s[0].angle_deg=-720;route_init(&p,s,2,NULL,NULL);CHECK(p.valid);
 s[0].corner_arc_enabled=true;route_init(&p,s,2,NULL,NULL);CHECK(!p.valid);
 s[0].corner_arc_enabled=false;s[0].angle_deg=300;
 init(&p,s,2);float d=0;uint32_t t=0;route_set_speed(&p,30,30);
 junction(&p,&d,&t,0xf8,0);CHECK(p.state==ROUTE_BRAKING);
 route_set_speed(&p,10,10);update(&p,&d,&t,0xf8,0,1);
 CHECK(p.hexagon.state==HEXAGON_ENTERING);
 update(&p,&d,&t,0,0,240);CHECK(p.failure==ROUTE_FAILURE_EXIT_LOST);
 /* Non-target physical junctions still pass and count exactly once. */
 s[0].junction_count=2;init(&p,s,2);d=0;t=0;
 junction(&p,&d,&t,0x1f,0);CHECK(p.passing_junction && p.hexagon.state==HEXAGON_OFF);
 clear(&p,&d,&t,0);junction(&p,&d,&t,0xf8,0);
 CHECK(p.junctions_seen==2 && p.hexagon.state==HEXAGON_ENTERING);
}
static void test_control_and_fallback(void)
{
 for(unsigned side=0;side<2;++side) {
  float sign=side?1:-1;hex_steps[0].angle_deg=sign*300;
  LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,hex_steps,2);
  CHECK(f.start_result==LINE_START_ACCEPTED);
  /* Exercise actual Route entry and both wheel speed controllers. */
  for(unsigned i=0;i<4;++i) tick(&f,&c,&t,5,5,0x18,false);
  for(unsigned i=0;i<4;++i) tick(&f,&c,&t,5,5,side?0xf8:0x1f,false);
  CHECK(f.route.hexagon.state==HEXAGON_ENTERING && f.state==LINE_JUNCTION);
  CHECK(sign*f.correction>0 && fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=12);
  f.route.entry_heading=f.odometry.heading_rad-sign*.5f;
  for(unsigned i=0;i<4;++i) tick(&f,&c,&t,5,5,0x18,false);
  CHECK(f.route.hexagon.state==HEXAGON_FOLLOWING && f.route.hexagon.use_line);
  tick(&f,&c,&t,5,5,side?0x20:0x04,false);
  CHECK(sign*f.correction>0); /* actual line position steers ring */
  tick(&f,&c,&t,5,5,0x3b,false);
  CHECK(f.route.component_recovery_active && !f.pid.primed);
  tick(&f,&c,&t,5,5,0,false);CHECK(f.command.enabled);
  /* Control timing faults still latch and reject START. */
  line_follow_update(&f,mask(0x18),c,t+20,.020f,false);
  CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_FAULT);
  tick(&f,&c,&t,5,5,0x18,true);CHECK(f.start_result==LINE_START_REJECTED_FAULT);
 }
#if MPU6050_ENABLED
 hex_steps[0].angle_deg=300;
 LineFollow f;EncoderCounts c={0};uint32_t t=0;
 line_follow_init(&f,calibration(),hex_steps,2,NULL,NULL);
 for(unsigned i=0;i<3;++i) {t+=5;line_follow_update_heading(&f,mask(0x18),c,t,.005f,i==2,true,0);}
 for(unsigned i=0;i<4;++i) {t+=5;c.left+=5;c.right+=5;line_follow_update_heading(&f,mask(0x18),c,t,.005f,false,true,0);}
 for(unsigned i=0;i<4;++i) {t+=5;c.left+=5;c.right+=5;line_follow_update_heading(&f,mask(0xf8),c,t,.005f,false,true,0);}
 CHECK(f.heading_source==HEADING_IMU && f.route.hexagon.state==HEXAGON_ENTERING);
 for(unsigned i=0;i<4;++i) {t+=5;c.left+=5;c.right+=5;line_follow_update_heading(&f,mask(0x18),c,t,.005f,false,true,.5f);}
 CHECK(f.route.hexagon.state==HEXAGON_FOLLOWING);
 float before=f.odometry.heading_rad;t+=5;c.left+=6;c.right+=4;
 line_follow_update_heading(&f,mask(0x18),c,t,.005f,false,false,NAN);
 CHECK(f.heading_source==HEADING_ENCODER && f.odometry.heading_rad>before);
 CHECK(f.route.hexagon.state==HEXAGON_FOLLOWING && f.route.junctions_seen==1);
#endif
}
int main(void)
{
 test_gates_and_mirror();test_loss_graph_and_deadlines();test_validation_and_entry();
 test_control_and_fallback();
 /* Existing ordinary LEFT/RIGHT/CUSTOM/STOP and loop regressions remain callable. */
 navigation_regressions();
 printf("HEXAGON FSM: %u checks PASS\n",checks);return 0;
}
