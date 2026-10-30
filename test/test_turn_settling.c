#define main navigation_regression_main
#include "test_navigation.c"
#undef main

static void prepare_turn(LineFollow *f, EncoderCounts *c, uint32_t *now,
                         const RouteStep *steps, float sign, float turn)
{
 start(f,c,now,steps,2);
 f->startup_elapsed_s=STARTUP_RAMP_TIME_S;
 f->base=25;f->left_target_cm_s=f->right_target_cm_s=50;
 f->route.state=ROUTE_EXECUTING;f->route.phase_ms=*now;
 f->route.entry_cm=f->odometry.distance_cm;
 f->route.entry_heading=f->route.action_start_heading=f->odometry.heading_rad-sign*turn;
}

static void test_settling(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) for(unsigned custom=0;custom<2;++custom) {
  float sign=mirror?1.0f:-1.0f;
  RouteStep steps[]={{1,50,custom?CUSTOM_TURN:(mirror?RIGHT:LEFT),200,
                     custom?sign*135:0,false,0},{1,50,STOP,200,0,false,0}};
  float target=custom?JUNCTION_TURN_TARGET_RAD*1.5f:JUNCTION_TURN_TARGET_RAD;
  LineFollow f;EncoderCounts c;uint32_t now;
  prepare_turn(&f,&c,&now,steps,sign,target);
  tick(&f,&c,&now,0,0,0,false);
  CHECK(f.route.state==ROUTE_EXECUTING && !f.route.line_exit_confirmed);
  /* No exit: hold angle, do not return to a full-speed pivot. */
  NEAR(f.left_target_cm_s,f.right_target_cm_s);
  CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=16.001f);

  prepare_turn(&f,&c,&now,steps,sign,target+.10f);
  tick(&f,&c,&now,0,0,0,false);
  CHECK(sign*f.correction<0 && f.route.state==ROUTE_EXECUTING);

  prepare_turn(&f,&c,&now,steps,sign,target-.20f);
  tick(&f,&c,&now,0,0,0,false);
  CHECK(sign*f.correction>0);
  CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<50);
  float slow_correction=sign*f.correction;
  /* Same remaining angle but measured positive yaw rate: demand less rotation. */
  prepare_turn(&f,&c,&now,steps,sign,target-.21f);
  tick(&f,&c,&now,mirror?10:0,mirror?0:10,0,false);
  CHECK(sign*f.correction<slow_correction);

  prepare_turn(&f,&c,&now,steps,sign,target+.01f);
  for(unsigned i=0;i<3;++i) tick(&f,&c,&now,0,0,0x18,false);
  CHECK(f.route.state==ROUTE_CLEARING && f.state==LINE_TRACKING);
  for(unsigned i=0;i<20;++i) {
   tick(&f,&c,&now,10,10,0x18,false);
   CHECK(f.route.index==0 && fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=16.001f);
  }
  for(unsigned i=0;i<40 && f.route.index==0;++i) tick(&f,&c,&now,10,10,0x18,false);
  CHECK(f.route.index==1 && f.route.state==ROUTE_FOLLOW && f.route.junctions_seen==0);
  CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)>16);
#if TURN_EXIT_ACCEL_ENABLED
  CHECK(f.exit_accelerating);
#else
  CHECK(!f.exit_accelerating);
#endif

  prepare_turn(&f,&c,&now,steps,sign,target);
  for(unsigned i=0;i<1000 && f.state!=LINE_STOPPED;++i) tick(&f,&c,&now,4,4,0,false);
  CHECK(f.state==LINE_STOPPED && f.route.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
  CHECK(!f.command.enabled && !f.route.line_exit_confirmed);
 }
}
static void test_line_controls_capture(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) for(unsigned mode=0;mode<3;++mode) {
  float sign=mirror?1.0f:-1.0f;
  float target=JUNCTION_TURN_TARGET_RAD*(mode==2?1.5f:1.0f);
  RouteStep steps[]={{1,50,mode==2?CUSTOM_TURN:(mirror?RIGHT:LEFT),200,
                     mode==2?sign*135.0f:0,mode==1,0},
                     {1,50,STOP,200,0,false,0}};
  LineFollow f;EncoderCounts c;uint32_t now;
  prepare_turn(&f,&c,&now,steps,sign,target-.30f);
  /* The exit has crossed center: line correction must oppose the original
   * turn even though the angle is short of target. Do not keep forcing yaw. */
  tick(&f,&c,&now,0,0,mirror?0x08:0x10,false);
  CHECK(f.route.turn_capturing && f.pid.primed);
  CHECK(sign*f.correction<0);
  CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=JUNCTION_SETTLE_SPEED_CM_S);
  CHECK(f.route.state==ROUTE_EXECUTING && !f.route.line_exit_confirmed);
  /* Centered early line permits only a small completion bias, never a
   * remaining-angle term large enough to overpower position steering. */
  pid_reset(&f.pid);
  tick(&f,&c,&now,0,0,0x18,false);
  CHECK(sign*f.correction>=0 && fabsf(f.correction)<=JUNCTION_CAPTURE_CENTER_BIAS_CM_S+.001f);
  CHECK(f.route.state==ROUTE_EXECUTING && !f.route.line_exit_confirmed);
  /* A lost captured exit is bounded independently of the action timeout. */
  for(unsigned i=0;i<INTERFERENCE_TIMEOUT_MS/5U+1U && f.state!=LINE_STOPPED;++i)
   tick(&f,&c,&now,8,8,i%20U==19U?0x18:0,false); /* isolated noise cannot renew loss budget */
  CHECK(f.state==LINE_STOPPED && f.route.failure==ROUTE_FAILURE_EXIT_LOST);
  CHECK(!f.command.enabled);
  prepare_turn(&f,&c,&now,steps,sign,target-.30f);
  tick(&f,&c,&now,0,0,0x18,false);
  float d=f.route.capture_line_cm+INTERFERENCE_MAX_DISTANCE_CM;
  route_update(&f.route,mask(0),d,f.odometry.heading_rad,now+5);
  CHECK(f.route.failure==ROUTE_FAILURE_EXIT_LOST);
  prepare_turn(&f,&c,&now,steps,sign,target-.30f);
  tick(&f,&c,&now,0,0,0x18,false);
  route_update(&f.route,mask(0x18),f.odometry.distance_cm,f.odometry.heading_rad,
               now+INTERFERENCE_TIMEOUT_MS);
  CHECK(f.route.failure==ROUTE_FAILURE_EXIT_LOST); /* Deadline precedes recovery. */
 }
}
#if MPU6050_ENABLED
static void test_imu_rate_and_fallback(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  float sign=mirror?1.0f:-1.0f;
  RouteStep steps[]={{1,50,mirror?RIGHT:LEFT,200,0,false,0},{1,50,STOP,200,0,false,0}};
  LineFollow f;EncoderCounts c={0};uint32_t now=0;
  line_follow_init(&f,calibration(),steps,2,NULL,NULL);
  for(unsigned i=0;i<3;++i) {
   now+=5;
   line_follow_update_heading(&f,mask(0x18),c,now,.005f,i==2,true,0);
  }
  CHECK(f.heading_source==HEADING_IMU && f.state==LINE_TRACKING);
  f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
  f.route.state=ROUTE_EXECUTING;f.route.phase_ms=now;
  f.route.entry_heading=-sign*(JUNCTION_TURN_TARGET_RAD-.2f);
  f.base=25;f.left_target_cm_s=f.right_target_cm_s=50;
  now+=5;
  line_follow_update_heading(&f,mask(0),c,now,.005f,false,true,0);
  float stationary=sign*f.correction;
  now+=5;
  /* IMU rotates while wheel counts stay unchanged: damping must use IMU yaw. */
  line_follow_update_heading(&f,mask(0),c,now,.005f,false,true,sign*.02f);
  CHECK(sign*f.turn_yaw_rate_rad_s>0 && sign*f.correction<stationary);
  float heading=f.odometry.heading_rad;
  now+=5;c.left+=mirror?5:0;c.right+=mirror?0:5;
  line_follow_update_heading(&f,mask(0),c,now,.005f,false,false,NAN);
  CHECK(f.heading_source==HEADING_ENCODER && f.state!=LINE_STOPPED);
  CHECK(sign*(f.odometry.heading_rad-heading)>0);
  CHECK(isfinite(f.turn_yaw_rate_rad_s) && isfinite(f.correction));
  CHECK(f.route.entry_heading==-sign*(JUNCTION_TURN_TARGET_RAD-.2f));

  /* A visible exit steers by position for both a healthy IMU and a
   * failure on that same captured line; fallback cannot restore forced yaw. */
  LineFollow captured;EncoderCounts capture_counts={0};uint32_t sample_ms=0;
  line_follow_init(&captured,calibration(),steps,2,NULL,NULL);
  for(unsigned i=0;i<3;++i) {
   sample_ms+=5;
   line_follow_update_heading(&captured,mask(0x18),capture_counts,sample_ms,.005f,i==2,true,0);
  }
  captured.startup_elapsed_s=STARTUP_RAMP_TIME_S;
  captured.route.state=ROUTE_EXECUTING;captured.route.phase_ms=sample_ms;
  captured.route.entry_heading=-sign*(JUNCTION_TURN_TARGET_RAD-.3f);
  sample_ms+=5;
  line_follow_update_heading(&captured,mask(mirror?0x08:0x10),capture_counts,sample_ms,.005f,false,true,0);
  CHECK(captured.heading_source==HEADING_IMU && sign*captured.correction<0);
  sample_ms+=5;
  line_follow_update_heading(&captured,mask(mirror?0x08:0x10),capture_counts,sample_ms,.005f,false,false,NAN);
  CHECK(captured.heading_source==HEADING_ENCODER && sign*captured.correction<0);
  CHECK(captured.route.state==ROUTE_EXECUTING && !captured.route.line_exit_confirmed);
 }
}
#endif
int main(void)
{
 test_line_controls_capture();
 test_settling();
#if MPU6050_ENABLED
 test_imu_rate_and_fallback();
#endif
 puts("PASS: mirrored angle settling, yaw damping, low-speed clearance, custom angles and no-exit timeout");
 return 0;
}
