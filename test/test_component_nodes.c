#define main existing_navigation_regressions
#include "test_navigation.c"
#undef main

static const RouteStep nodes[]={
 {3,50,LEFT,200,0,false,0},{1,50,STOP,200,0,false,0}
};
static uint8_t mirror_mask(uint8_t m)
{
 uint8_t r=0;
 for(unsigned i=0;i<8;++i) if(m&(1U<<i)) r|=(uint8_t)(1U<<(7U-i));
 return r;
}
static void test_graphics_both_directions(void)
{
 /* Simplified symbol feature sequences based on the supplied drawings:
  * resistor: bar -> separated sides -> bar; battery: short/long bars + gaps.
  * They are synthetic masks, not measured GPIO or a geometric simulation. */
 const uint8_t resistor[]={0xff,0xff,0xff,0xff,0x66,0x66,0x66,0x66,0x66,0x66,0xff,0xff,0xff,0xff};
 const uint8_t battery[]={0x3c,0x3c,0x3c,0x3c,0,0,0xff,0xff,0xff,0xff,0,0,0x3c,0x3c,0x3c,0x3c};
 for(unsigned reverse=0;reverse<2;++reverse) for(unsigned graphic=0;graphic<2;++graphic) {
  RoutePlanner p;float d=0;uint32_t t=0;init(&p,nodes,2);
  update(&p,&d,&t,0x18,0,4);
  const uint8_t *seq=graphic?battery:resistor;
  unsigned count=graphic?sizeof(battery):sizeof(resistor);
  for(unsigned i=0;i<count;++i) {
   uint8_t m=seq[reverse?count-1U-i:i];
   update(&p,&d,&t,reverse?mirror_mask(m):m,0,1);
  }
  CHECK(p.junctions_seen==1 && p.passing_junction && p.component_pass_active);
  CHECK(!p.component_recovery_active && p.index==0 && route_current_action(&p)==STRAIGHT);
  update(&p,&d,&t,0x03,0,4);CHECK(p.component_pass_active);
  clear(&p,&d,&t,0);CHECK(p.state==ROUTE_FOLLOW && p.junctions_seen==1);
  /* A second, partially visible resistor counts once in the same sequence. */
  update(&p,&d,&t,0x66,0,4);
  CHECK(p.junctions_seen==2 && p.last_node_kind==NODE_COMPONENT && p.component_pass_active);
  clear(&p,&d,&t,0);CHECK(p.final_segment_active && p.junctions_seen==2);
  update(&p,&d,&t,reverse?0xf8:0x1f,0,3);
  CHECK(p.junctions_seen==3 && p.state==ROUTE_EXECUTING && route_current_action(&p)==LEFT);
 }
 for(unsigned m=0;m<256;++m)
  CHECK(gray_has_component_evidence(mask((uint8_t)m))==
        gray_has_component_evidence(mask(mirror_mask((uint8_t)m))));
 CHECK(gray_has_component_evidence(mask(0x66)));
 CHECK(gray_has_component_evidence(mask(0xdb)));
 CHECK(!gray_has_component_evidence(mask(0x99)));
 CHECK(!gray_has_component_evidence(mask(0x9b)));
}
static void test_reject_noise_and_wrong_target(void)
{
 RoutePlanner p;float d=0;uint32_t t=0;init(&p,nodes,2);
 update(&p,&d,&t,0x66,0,4);CHECK(p.junctions_seen==0); /* no approach */
 update(&p,&d,&t,0x18,0,4);
 const uint8_t bad[]={0,0x99,0x9b,0x01,0x18};
 for(unsigned i=0;i<sizeof(bad);++i) update(&p,&d,&t,bad[i],0,2);
 CHECK(p.junctions_seen==0);
 update(&p,&d,&t,0x18,0,4);
 update(&p,&d,&t,0x66,0,1);update(&p,&d,&t,0x18,0,4);
 CHECK(p.junctions_seen==0);
 for(unsigned i=0;i<6;++i) {t+=5;route_update(&p,mask(0x66),d,0,t);}
 CHECK(p.junctions_seen==0); /* no movement */
 init(&p,nodes,2);d=0;t=0;update(&p,&d,&t,0x18,0,4);
 update(&p,&d,&t,0x66,0,2);t+=CONTROL_MAX_INTERVAL_MS+1U;
 update(&p,&d,&t,0x66,0,1);CHECK(p.junctions_seen==0);
 const RouteStep mismatch[]={{1,50,LEFT,50,0,false,0},{1,50,STOP,50,0,false,0}};
 init(&p,mismatch,2);d=0;t=0;update(&p,&d,&t,0x18,0,4);
 update(&p,&d,&t,0x66,0,4);
 CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_NODE_TYPE && p.index==0);
 const RouteStep straight[]={{1,50,STRAIGHT,50,0,false,0},{1,50,STOP,50,0,false,0}};
 init(&p,straight,2);d=0;t=0;update(&p,&d,&t,0x18,0,4);
 update(&p,&d,&t,0x66,0,4);update(&p,&d,&t,0x66,0,1);
 CHECK(p.component_pass_active && route_current_action(&p)==STRAIGHT);
 clear(&p,&d,&t,0);CHECK(p.index==1 && !p.component_pass_active);
}
static void test_pass_limits_and_close_turn(void)
{
 for(unsigned bound=0;bound<3;++bound) {
  RoutePlanner p;float d=0;uint32_t t=0;init(&p,nodes,2);
  junction(&p,&d,&t,0xff,0);CHECK(p.component_pass_active);
  uint32_t entry=p.component_pass_ms;float origin=p.component_pass_cm;
  update(&p,&d,&t,0xff,0,90);
  CHECK(p.component_pass_active && p.component_pass_ms==entry);
  CHECK(p.state==ROUTE_CLEARING && p.junctions_seen==1);
  route_update(&p,mask(0x18),bound==1?origin+COMPONENT_PASS_MAX_CM:d,
      bound==2?COMPONENT_PASS_HEADING_MAX_RAD:0,
      bound==0?entry+COMPONENT_PASS_TIMEOUT_MS:t+5);
  CHECK(p.failure==ROUTE_FAILURE_COMPONENT_PASS && p.index==0 && p.junctions_seen==1);
 }
 const RouteStep close[]={{2,50,LEFT,200,0,false,0},{1,50,STOP,200,0,false,0}};
 for(unsigned side=0;side<2;++side) {
  RoutePlanner p;float d=0;uint32_t t=0;init(&p,close,2);
  junction(&p,&d,&t,0xff,0);
  update(&p,&d,&t,0x18,0,8);CHECK(p.component_pass_active);
  update(&p,&d,&t,side?0xf8:0x1f,0,3);
  CHECK(p.junctions_seen==2 && p.state==ROUTE_EXECUTING && !p.component_pass_active);
 }
}
static void test_speed_heading_pid_and_faults(void)
{
 LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,nodes,2);
 for(unsigned i=0;i<4;++i) tick(&f,&c,&t,20,20,0x18,false);
 f.startup_elapsed_s=1;f.base=f.left_target_cm_s=f.right_target_cm_s=60;
 for(unsigned i=0;i<3;++i) tick(&f,&c,&t,20,20,0xff,false);
 CHECK(f.route.component_pass_active && !f.pid.primed && !f.abnormal);
 for(unsigned i=0;i<30;++i) tick(&f,&c,&t,20,20,i%2?0xff:0x66,false);
 CHECK(f.route.junctions_seen==1 && !f.route.component_recovery_active);
 CHECK(f.left_target_cm_s>COMPONENT_RECOVERY_SPEED_CM_S);
 CHECK(f.left_target_cm_s<=COMPONENT_PASS_SPEED_CM_S && !f.pid.primed);
 float anchor=f.route.component_pass_heading;
 tick(&f,&c,&t,20,21,0x03,false);
 NEAR(f.route.component_pass_heading,anchor);
 CHECK(f.left_target_cm_s>f.right_target_cm_s); /* encoder yaw negative -> right correction */
 for(unsigned i=0;i<16;++i) tick(&f,&c,&t,20,20,0x18,false);
 CHECK(!f.route.component_pass_active && f.state==LINE_TRACKING && f.pid.primed);
 CHECK(f.route.junctions_seen==1);
 start(&f,&c,&t,nodes,2);
 for(unsigned i=0;i<4;++i) tick(&f,&c,&t,20,20,0x18,false);
 for(unsigned i=0;i<3;++i) tick(&f,&c,&t,20,20,0xff,false);
 tick(&f,&c,&t,32768,0,0xff,false);
 CHECK(f.state==LINE_STOPPED && f.stop_reason==LINE_STOP_ENCODER);
 CHECK(f.stop_snapshot.component_pass_active);
 tick(&f,&c,&t,0,0,0x18,true);
 CHECK(f.start_result==LINE_START_REJECTED_FAULT);
 /* A graphic rejected for missing approach history still cannot steer PID. */
 start(&f,&c,&t,nodes,2);
 tick(&f,&c,&t,5,5,0x3c,false);
 CHECK(f.route.state==ROUTE_FOLLOW && f.route.junctions_seen==0);
 CHECK(f.state==LINE_GAP_HOLD && !f.pid.primed);
}
static void test_raw_filter_and_heading_source(void)
{
 for(unsigned reverse=0;reverse<2;++reverse) {
  RoutePlanner p;GraySensor sensor;float d=0;uint32_t t=0;
  init(&p,nodes,2);gray_sensor_init(&sensor);
  for(unsigned i=0;i<6;++i) {
   t+=5;d+=.06f;
   route_update(&p,gray_sensor_update(&sensor,raw_for_line(0x18),true,0),d,0,t);
  }
  for(unsigned i=0;i<8;++i) {
   t+=5;d+=.06f;
   uint8_t m=reverse?mirror_mask(0x36):0x36;
   route_update(&p,gray_sensor_update(&sensor,raw_for_line(m),true,0),d,0,t);
  }
  CHECK(p.junctions_seen==1 && p.component_pass_active);
 }
#if MPU6050_ENABLED
 LineFollow f;EncoderCounts c={0};uint32_t t=0;
 line_follow_init(&f,calibration(),nodes,2,NULL,NULL);
 for(unsigned i=0;i<7;++i) {
  t+=5;line_follow_update_heading(&f,mask(0x18),c,t,.005f,i==2,true,0);
 }
 f.startup_elapsed_s=1;f.base=f.left_target_cm_s=f.right_target_cm_s=60;
 for(unsigned i=0;i<3;++i) {
  t+=5;c.left+=20;c.right+=20;
  line_follow_update_heading(&f,mask(0xff),c,t,.005f,false,true,0);
 }
 CHECK(f.route.component_pass_active && f.heading_source==HEADING_IMU);
 float anchor=f.route.component_pass_heading;
 t+=5;c.left+=20;c.right+=20;
 line_follow_update_heading(&f,mask(0x66),c,t,.005f,false,true,.1f);
 CHECK(f.left_target_cm_s<f.right_target_cm_s && !f.pid.primed);
 t+=5;c.left+=20;c.right+=20;
 line_follow_update_heading(&f,mask(0),c,t,.005f,false,false,NAN);
 CHECK(f.heading_source==HEADING_ENCODER && f.route.component_pass_active);
 NEAR(f.route.component_pass_heading,anchor);
 NEAR(f.odometry.heading_rad,.1f);
#endif
}
int main(void)
{
 test_graphics_both_directions();test_reject_noise_and_wrong_target();
 test_pass_limits_and_close_turn();test_speed_heading_pid_and_faults();
 test_raw_filter_and_heading_source();
 printf("PASS: component nodes both directions, shared counts, bounded high-speed passage and close turn (%u checks)\n",checks);
 return 0;
}
