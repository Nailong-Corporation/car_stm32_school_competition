#define main navigation_regression_main
#include "test_navigation.c"
#undef main
static void test_observed_center_line_exit(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  RouteStep steps[]={{1,50,mirror?RIGHT:LEFT,50,0,false,0},{2,50,STOP,200,0,false,0}};
  LineFollow f;EncoderCounts c;uint32_t now;start(&f,&c,&now,steps,2);
  f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
  f.route.state=ROUTE_EXECUTING;f.route.phase_ms=now;
  f.route.entry_heading=f.route.action_start_heading=(mirror?-1:1)*.736127615f;
  for(unsigned i=0;i<2;++i) {
   tick(&f,&c,&now,8,8,mirror?0x10:0x08,false);
   CHECK(f.route.state==ROUTE_EXECUTING);
  }
  tick(&f,&c,&now,8,8,mirror?0x10:0x08,false);
  CHECK(f.route.state==ROUTE_CLEARING && f.state==LINE_TRACKING);
  CHECK(f.pid.primed && f.turn_exit[0].line_confirmed_exit);
  for(unsigned i=0;i<35 && f.route.index==0;++i) tick(&f,&c,&now,8,8,0x18,false);
  CHECK(f.route.state==ROUTE_FOLLOW && f.route.index==1);
 }
}
static void test_turn_into_adjacent_diode(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  RouteStep steps[]={{1,50,mirror?RIGHT:LEFT,50,0,false,0},
                     {2,50,LEFT,200,0,false,50},{1,50,STOP,200,0,false,0}};
  LineFollow f;EncoderCounts c;uint32_t now;start(&f,&c,&now,steps,3);
  f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
  f.route.state=ROUTE_EXECUTING;f.route.phase_ms=now;
  f.route.entry_heading=f.route.action_start_heading=(mirror?-1:1)*(JUNCTION_TURN_TARGET_RAD+.05f);
  uint8_t corridor=mirror?0xfc:0x3f, diode=mirror?0x8c:0x31;
  for(unsigned i=0;i<3;++i) tick(&f,&c,&now,8,8,corridor,false);
  CHECK(f.route.state==ROUTE_CLEARING && f.route.index==0);
  for(unsigned i=0;i<6;++i) tick(&f,&c,&now,8,8,corridor,false);
  CHECK(f.route.index==0); /* Not the ordinary 2 cm clearance yet. */
  for(unsigned i=0;i<3;++i) tick(&f,&c,&now,8,8,diode,false);
  CHECK(f.state!=LINE_STOPPED);
  CHECK(f.route.index==1 && f.route.junctions_seen==1 && f.route.passing_junction);
  CHECK(f.route.state==ROUTE_CLEARING);
  for(unsigned i=0;i<5;++i) tick(&f,&c,&now,8,8,diode,false);
  CHECK(f.route.index==1 && f.route.junctions_seen==1); /* no duplicate tail count */
  for(unsigned i=0;i<30;++i) tick(&f,&c,&now,8,8,0x18,false);
  CHECK(f.route.state==ROUTE_FOLLOW && f.route.final_segment_active);
  for(unsigned i=0;i<3;++i) tick(&f,&c,&now,8,8,0xff,false);
  CHECK(f.route.index==1 && f.route.junctions_seen==2 && f.route.state==ROUTE_EXECUTING);
 }
}
static void test_diode_straight_clearance(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  RouteStep steps[]={{2,50,LEFT,200,0,false,50},{1,50,STOP,200,0,false,0}};
  LineFollow f;EncoderCounts c;uint32_t now;start(&f,&c,&now,steps,2);
  f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
  for(unsigned i=0;i<3;++i) tick(&f,&c,&now,0,0,0x18,false);
  /* Captured 0x0f side of the diode is one region, not a straight exit.
   * Mirrored 0xf0 must also preserve the entry axis, not follow its edge. */
  for(unsigned i=0;i<3;++i) tick(&f,&c,&now,20,20,mirror?0xf0:0x0f,false);
  CHECK(f.route.state==ROUTE_CLEARING && f.route.junctions_seen==1);
  NEAR(f.left_target_cm_s,f.right_target_cm_s);
  CHECK(!f.pid.primed);
  for(unsigned i=0;i<12;++i) tick(&f,&c,&now,20,20,mirror?0xc0:0x03,false);
  CHECK(f.route.state==ROUTE_CLEARING && f.route.junctions_seen==1);
  for(unsigned i=0;i<4;++i) tick(&f,&c,&now,20,20,0x24,false); /* split sides, no centering */
  for(unsigned i=0;i<16;++i) tick(&f,&c,&now,20,20,0x18,false);
  CHECK(f.route.state==ROUTE_FOLLOW && f.route.final_segment_active);
  CHECK(f.route.junctions_seen==1 && f.route.index==0);
#if TURN_APPROACH_SLOWDOWN_ENABLED
  CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=50);
#endif
  for(unsigned i=0;i<3;++i) tick(&f,&c,&now,20,20,0xff,false);
  CHECK(f.route.junctions_seen==2 && f.route.state==ROUTE_EXECUTING);
  CHECK(route_current_action(&f.route)==LEFT);
 }
 /* Broad centered graphics, isolated center frames and side edges must not
  * unlock counting or renew the bounded straight-clearance loss budget. */
 for(unsigned condition=0;condition<3;++condition) {
  RouteStep steps[]={{2,50,LEFT,200,0,false,50},{1,50,STOP,200,0,false,0}};
  RoutePlanner p;init(&p,steps,2);float d=0;uint32_t t=0;
  junction(&p,&d,&t,0x0f,0);
  CHECK(p.state==ROUTE_CLEARING && p.junctions_seen==1);
  for(unsigned i=0;i<300 && p.state!=ROUTE_FAILED;++i) {
   uint8_t m=condition==0?0x3c:(condition==1 && i%20U==19U?0x18:0x03);
   update(&p,&d,&t,m,0,1);
   CHECK(p.junctions_seen==1);
  }
  CHECK(p.failure==(condition==0?ROUTE_FAILURE_EXIT_RECAPTURE:ROUTE_FAILURE_CLEAR_LINE_LOST) && p.index==0);
 }
 RouteStep steps[]={{2,50,LEFT,200,0,false,50},{1,50,STOP,200,0,false,0}};
 RoutePlanner p;init(&p,steps,2);float d=0;uint32_t t=0;
 junction(&p,&d,&t,0x0f,0);
 d=p.capture_line_cm+INTERFERENCE_MAX_DISTANCE_CM;
 route_update(&p,mask(0x18),d,0,t+5); /* A recovery at the distance deadline is too late. */
 CHECK(p.failure==ROUTE_FAILURE_CLEAR_LINE_LOST);
}
static void test_fast_penultimate_junction(void)
{
 RouteStep steps[]={{2,50,LEFT,200,0,false,50},{1,50,STOP,200,0,false,0}};
 LineFollow f;EncoderCounts c;uint32_t now;start(&f,&c,&now,steps,2);
 f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
 for(unsigned i=0;i<3;++i) tick(&f,&c,&now,0,0,0x18,false);
 /* 200 cm/s: each 5 ms frame advances 1 cm. The center history must
  * seed a continuous candidate, rather than expire before frame three. */
 for(unsigned i=0;i<3;++i) tick(&f,&c,&now,100,100,0xff,false);
 CHECK(f.route.junctions_seen==1 && f.route.state==ROUTE_CLEARING);
 CHECK(!f.route.final_segment_active); /* Distance origin still awaits clearance. */
#if TURN_APPROACH_SLOWDOWN_ENABLED
 CHECK(f.left_target_cm_s<=50 && f.right_target_cm_s<=50);
#endif
 /* Remaining over the same graphic must never increment the count. */
 for(unsigned i=0;i<4;++i) tick(&f,&c,&now,25,25,0xff,false);
 CHECK(f.route.junctions_seen==1 && f.route.state==ROUTE_CLEARING);
 for(unsigned i=0;i<16;++i) tick(&f,&c,&now,25,25,0x18,false);
 CHECK(f.route.state==ROUTE_FOLLOW && f.route.final_segment_active);
 CHECK(f.route.junctions_seen==1 && f.route.index==0);
#if TURN_APPROACH_SLOWDOWN_ENABLED
 CHECK(f.left_target_cm_s<=50 && f.right_target_cm_s<=50);
#endif
 /* After clearance the very next confirmed junction must execute LEFT. */
 for(unsigned i=0;i<3;++i) tick(&f,&c,&now,25,25,0xff,false);
 CHECK(f.route.junctions_seen==2 && f.route.state==ROUTE_BRAKING);
 /* Filtered feedback is still slightly above 50: retain the speed gate. */
 for(unsigned i=0;i<5 && f.route.state==ROUTE_BRAKING;++i)
  tick(&f,&c,&now,20,20,0xff,false);
 CHECK(f.route.junctions_seen==2 && f.route.state==ROUTE_EXECUTING);
 CHECK(route_current_action(&f.route)==LEFT);

 /* Latching center evidence cannot bridge loss or a sampling interruption. */
 for(unsigned gap=0;gap<2;++gap) {
  RoutePlanner p;init(&p,steps,2);float d=0;uint32_t t=0;
  update(&p,&d,&t,0x18,0,4);
  d+=1;t+=5;route_update(&p,mask(0xff),d,0,t);
  CHECK(p.state==ROUTE_CANDIDATE);
  d+=1;t+=gap?CONTROL_MAX_INTERVAL_MS+1:5;
  route_update(&p,mask(gap?0xff:0),d,0,t);
  CHECK(p.state==ROUTE_FOLLOW && p.junctions_seen==0);
  update(&p,&d,&t,0xff,0,4);
  CHECK(p.state==ROUTE_FOLLOW && p.junctions_seen==0);
 }
}
static void test_final_segment_speed_cap(void)
{
 /* Unknown length is slow throughout; measured length uses the distance envelope. */
 for(unsigned length=0;length<2;++length) {
  RouteStep steps[]={{1,50,LEFT,200,0,false,length?1000:0},
                     {1,50,STOP,200,0,false,0}};
  LineFollow f;EncoderCounts c;uint32_t now;start(&f,&c,&now,steps,2);
  f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
  f.base=f.left_target_cm_s=f.right_target_cm_s=200;
  f.braking_active=false;
  tick(&f,&c,&now,0,0,0x18,false);
#if TURN_APPROACH_SLOWDOWN_ENABLED
  NEAR(f.left_target_cm_s,length?200:50);
  NEAR(f.right_target_cm_s,length?200:50);
#else
  NEAR(f.left_target_cm_s,200);NEAR(f.right_target_cm_s,200);
#endif
  CHECK(f.route.final_segment_active && f.route.junctions_seen==0);
 }
 RouteStep steps[]={{3,50,LEFT,200,0,false,0},{1,50,STOP,200,0,false,0}};
 LineFollow f;EncoderCounts c;uint32_t now;start(&f,&c,&now,steps,2);
 f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
 f.base=f.left_target_cm_s=f.right_target_cm_s=200;
 tick(&f,&c,&now,0,0,0x18,false);NEAR(f.base,200);
 float d=0;
 for(unsigned i=0;i<2;++i) {
  junction(&f.route,&d,&now,0xff,0);clear(&f.route,&d,&now,0);
  CHECK(f.route.state==ROUTE_FOLLOW && f.route.junctions_seen==i+1);
  f.base=f.left_target_cm_s=f.right_target_cm_s=200;
  tick(&f,&c,&now,0,0,0x18,false);
#if TURN_APPROACH_SLOWDOWN_ENABLED
  NEAR(f.base,i==0?200:50);
#else
  NEAR(f.base,200);
#endif
 }
 /* A completed turn hands off directly to the next count=1 low-speed leg. */
 junction(&f.route,&d,&now,0xff,0);
 CHECK(f.route.state==ROUTE_EXECUTING);
 update(&f.route,&d,&now,0x18,-JUNCTION_TURN_TARGET_RAD,4);
 clear(&f.route,&d,&now,-JUNCTION_TURN_TARGET_RAD);
 CHECK(f.route.index==1 && f.route.final_segment_active);
 f.base=f.left_target_cm_s=f.right_target_cm_s=200;
 tick(&f,&c,&now,0,0,0x18,false);
#if TURN_APPROACH_SLOWDOWN_ENABLED
 NEAR(f.base,50);
#else
 NEAR(f.base,200);
#endif
}
int main(void)
{
 test_observed_center_line_exit();
 test_turn_into_adjacent_diode();
 test_diode_straight_clearance();
 test_fast_penultimate_junction();
 test_final_segment_speed_cap();
 SpeedControl s;MotorCommand out;speed_control_init(&s);
 CHECK(speed_control_update(&s,20,20,80,20,.005f,&out));
 CHECK(out.left>=0); /* Ordinary speed loop never requests reverse torque. */
 CHECK(speed_control_update_braking(&s,20,20,80,20,.005f,true,&out));
#if MOTOR_REVERSE_BRAKE_ENABLED
 CHECK(out.left<0 && out.left>=-REVERSE_BRAKE_MAX_PWM);
#else
 CHECK(out.left>=0);
#endif
 RouteStep steps[]={{1,6,LEFT,50,0,false,0},{1,6,STOP,80,0,false,0}};
 LineFollow f;EncoderCounts c;uint32_t now;start(&f,&c,&now,steps,2);
 f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
 f.base=f.left_target_cm_s=f.right_target_cm_s=50;
 f.route.origin_cm=-10000;
 tick(&f,&c,&now,0,0,0x18,false);NEAR(f.base,50); /* No distance trigger. */
 f.route.state=ROUTE_EXECUTING;f.route.phase_ms=now;f.route.entry_heading=f.odometry.heading_rad;
 tick(&f,&c,&now,0,0,0,false);
 CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=steps[0].turn_speed);
 f.route.state=ROUTE_CLEARING;f.route.phase_ms=now;f.route.clear_cm=f.odometry.distance_cm;
 f.route.entry_heading=f.odometry.heading_rad+JUNCTION_TURN_TARGET_RAD;
 f.base=f.left_target_cm_s=f.right_target_cm_s=6;
 tick(&f,&c,&now,0,0,0x18,false);
 CHECK(f.base<=6); /* 出口确认后先低速离口，不立即追下一段80。 */
 printf("PASS: approach=%d exit=%d reverse=%d\n",TURN_APPROACH_SLOWDOWN_ENABLED,TURN_EXIT_ACCEL_ENABLED,MOTOR_REVERSE_BRAKE_ENABLED);
 return 0;
}
