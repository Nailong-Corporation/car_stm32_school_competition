#define main existing_navigation_tests
#include "test_navigation.c"
#undef main

static uint8_t mirrored(uint8_t m)
{
 uint8_t r=0;
 for(unsigned i=0;i<8;++i) if(m&(1U<<i)) r|=(uint8_t)(1U<<(7U-i));
 return r;
}
static void test_counting_speed_and_short_nodes(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) for(unsigned phase=0;phase<5;++phase) {
  RouteStep s[]={{4,50,mirror?RIGHT:LEFT,200,0,false,70},{1,50,STOP,50,0,false,0}};
  LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
  f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
  /* A low-speed turn has just handed control to a count=4 leg whose configured
   * cruise is 200. It must not accelerate past the sampling speed while the
   * final-segment distance is still intentionally inactive. */
  f.base=f.left_target_cm_s=f.right_target_cm_s=16;
  GraySensor sensor;gray_sensor_init(&sensor);
  for(unsigned warm=0;warm<3;++warm) (void)gray_sensor_update(&sensor,raw_for_line(0x18),false,0);
  float fractional=0;
  for(unsigned i=0;i<2200 && f.route.state!=ROUTE_EXECUTING;++i) {
   float speed=fmaxf(f.left_target_cm_s,f.right_target_cm_s);
   fractional+=speed*.005f/.01f;
   unsigned delta=(unsigned)fractional;fractional-=delta;
   c.left=(uint16_t)(c.left+delta);c.right=(uint16_t)(c.right+delta);t+=5;
   float d=f.odometry.distance_cm+.01f*delta;
   uint8_t line=0x18;
   /* First three 1.2 cm features are 8 cm apart; the final turn is about
    * 70 cm after penultimate-node clearance. Five phases and both sides.
    * Shapes are synthetic; this does not calibrate real feature size. */
   const uint8_t shapes[]={0x66,0x1f,0xff,0x1f};
   for(unsigned n=0;n<4;++n) {
    float beginning=(n==3?102.0f:12+8.0f*n)+.05f*phase;
    if(d>=beginning && d<beginning+1.2f) line=mirror?mirrored(shapes[n]):shapes[n];
   }
   GrayObservation obs=gray_sensor_update(&sensor,raw_for_line(line),
      line_follow_have_recent_position(&f,t),f.reliable_position);
   line_follow_update(&f,obs,c,t,.005f,false);
   CHECK(f.state!=LINE_STOPPED);
   if(!f.route.final_segment_active)
    CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=50.001f);
   else if(f.odometry.distance_cm-f.route.final_segment_origin_cm>=65)
    CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=16.001f);
   CHECK(f.route.index==0);
  }
  CHECK(f.route.junctions_seen==4 && f.route.state==ROUTE_EXECUTING);
  CHECK(route_current_action(&f.route)==(mirror?RIGHT:LEFT));
 }
}
static void test_low_cruise_keeps_speed_longer(void)
{
 RouteStep s[]={{1,50,LEFT,50,0,false,65},{1,50,STOP,50,0,false,0}};
 LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
 f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
 f.base=f.left_target_cm_s=f.right_target_cm_s=50;
 f.odometry.left_speed_cm_s=f.odometry.right_speed_cm_s=50;
 f.route.final_segment_origin_cm=f.odometry.distance_cm-35;
 tick(&f,&c,&t,25,25,0x18,false);
 CHECK(f.state==LINE_TRACKING && f.route.junctions_seen==0);
 /* Old 40 profile has already dropped below 47 here. Low-cruise 50 profile
  * keeps 50 longer, with the SAME hard deadline and 16 cm/s entry gate. */
 CHECK(f.left_target_cm_s>49.9f && f.right_target_cm_s>49.9f);
 /* Higher configured cruise retains the conservative 40 profile, even when
  * the measured speed for this particular frame happens to be only 50. */
 s[0].cruise_speed=200;start(&f,&c,&t,s,2);
 f.startup_elapsed_s=STARTUP_RAMP_TIME_S;
 f.base=f.left_target_cm_s=f.right_target_cm_s=50;
 f.odometry.left_speed_cm_s=f.odometry.right_speed_cm_s=50;
 f.route.final_segment_origin_cm=f.odometry.distance_cm-35;
 tick(&f,&c,&t,25,25,0x18,false);
 CHECK(f.left_target_cm_s<47 && f.right_target_cm_s<47);
}
static void test_captured_two_frame_branch_still_rejected(void)
{
 RouteStep s[]={{4,50,LEFT,200,0,false,70},{1,50,STOP,50,0,false,0}};
 RoutePlanner p;float d=0;uint32_t t=0;init(&p,s,2);
 p.junctions_seen=2;route_set_speed(&p,119,113);
 update(&p,&d,&t,0x18,0,4);
 for(unsigned i=0;i<2;++i) {d+=.6f;t+=5;route_update(&p,mask(0x1f),d,0,t);}
 CHECK(p.state==ROUTE_CANDIDATE && p.candidate_count==2 && p.junctions_seen==2);
 d+=.6f;t+=5;route_update(&p,mask(0x18),d,0,t);
 CHECK(p.state==ROUTE_FOLLOW && p.junctions_seen==2);
}
int main(int argc,char **argv)
{
 (void)argv;
 if(argc==1) test_counting_speed_and_short_nodes();
 test_low_cruise_keeps_speed_longer();
 test_captured_two_frame_branch_still_rejected();
 printf("PASS: count-phase speed, filtered short nodes, mirrors/phases and low-cruise approach (%u checks)\n",checks);
 return 0;
}
