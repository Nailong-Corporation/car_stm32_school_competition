/* Standalone regression; production files deliberately remain untouched. */
#define main previous_navigation_main
#include "test_navigation.c"
#undef main
#include <string.h>

static float outer_speed(const LineFollow *f)
{return fmaxf(f->left_target_cm_s,f->right_target_cm_s);}
static void capped(const LineFollow *f)
{
 CHECK(f->state!=LINE_STOPPED);
 CHECK(outer_speed(f)<=JUNCTION_SETTLE_SPEED_CM_S+0.001f);
}
static void unchanged(const LineFollow *f,const RouteStep *s,const RouteStep *copy,
                      size_t n,uint16_t seen,RouteState state,float origin)
{
 CHECK(memcmp(s,copy,n*sizeof(*s))==0);
 CHECK(f->route.steps==s && f->route.count==n && f->route.index==0);
 CHECK(f->route.junctions_seen==seen && f->route.state==state);
 NEAR(f->route.origin_cm,origin);
 CHECK(f->route.failure==ROUTE_FAILURE_NONE);
}
static void direction(const LineFollow *f,float position)
{
 if(position>0) CHECK(f->left_target_cm_s>f->right_target_cm_s);
 else CHECK(f->right_target_cm_s>f->left_target_cm_s);
}
/* Authorized short-segment behavior: a <=100 cm segment cruises at
 * ROUTE_SHORT_SPEED_CM_S early; only the measured entry envelope and the
 * 16 cm/s turn gate slow it near the node. There is no whole-segment 16 cap. */
static void test_short_turn_cap(void)
{
 for(unsigned side=0;side<2;++side) for(unsigned pos=0;pos<4;++pos) {
  const uint8_t observations[]={0x10,0x20,0x08,0x04}; /* +1,+3,-1,-3 */
  RouteStep s[]={{1,50,side?RIGHT:LEFT,200,0,false,65, 0},{1,50,STOP,200,0,false,70, 0}};
  RouteStep copy[2];memcpy(copy,s,sizeof(s));
  LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
  float origin=f.route.origin_cm;
  f.startup_elapsed_s=1;
  f.braking_active=false;
  f.base=f.left_target_cm_s=f.right_target_cm_s=60;
  tick(&f,&c,&t,0,0,observations[pos],false);
  unchanged(&f,s,copy,2,0,ROUTE_FOLLOW,origin);
  direction(&f,mask(observations[pos]).position);
  CHECK(outer_speed(&f)>50); /* short segment is not globally capped at 16 */
  CHECK(outer_speed(&f)<=ROUTE_SHORT_SPEED_CM_S+.001f);
  /* Mid-segment still plans the short-segment cruise speed. */
  f.odometry.distance_cm=origin+15;
  tick(&f,&c,&t,0,0,observations[pos],false);
  unchanged(&f,s,copy,2,0,ROUTE_FOLLOW,origin);
  direction(&f,mask(observations[pos]).position);
  CHECK(outer_speed(&f)>50);
  /* Near the measured node the speed envelope reaches the 16 cm/s turn gate. */
  f.odometry.distance_cm=origin+65-GRAY_FORWARD_OFFSET_CM;
  tick(&f,&c,&t,0,0,observations[pos],false);
  unchanged(&f,s,copy,2,0,ROUTE_FOLLOW,origin);
  direction(&f,mask(observations[pos]).position);
  CHECK(f.base<=JUNCTION_SETTLE_SPEED_CM_S+.001f);
  CHECK(outer_speed(&f)<=JUNCTION_SETTLE_SPEED_CM_S+.001f);
 }
}
static void test_long_turn_cap(void)
{
 for(unsigned side=0;side<2;++side) for(unsigned pos=0;pos<4;++pos) {
  const uint8_t observations[]={0x10,0x20,0x08,0x04};
  RouteStep s[]={{1,50,side?RIGHT:LEFT,200,0,false,219, 0},{1,50,STOP,200,0,false,70, 0}};
  RouteStep copy[2];memcpy(copy,s,sizeof(s));
  LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
  for(unsigned i=0;i<220;++i) tick(&f,&c,&t,40,40,0x18,false);
  CHECK(outer_speed(&f)>16 && f.route.state==ROUTE_FOLLOW && !f.route.search_pending);
  /* Exhaust distance without a junction: must never execute a blind turn. */
  f.route.origin_cm=f.route.final_segment_origin_cm=f.odometry.distance_cm-220;
  float origin=f.route.origin_cm;
  for(unsigned i=0;i<4;++i) {
   tick(&f,&c,&t,40,40,observations[pos],false);
   unchanged(&f,s,copy,2,0,ROUTE_FOLLOW,origin);
   CHECK(f.route.search_pending);
   direction(&f,mask(observations[pos]).position);capped(&f);
  }
 }
}
/* Forty counts/5ms gives positive feedback (~80cm/s), avoiding accidental
 * STALL-based success with intentionally high pre-existing commands. */
static void seed_pass(LineFollow *f,EncoderCounts *c,uint32_t *t,const RouteStep *s)
{
 start(f,c,t,s,2);f->startup_elapsed_s=1;
 f->base=f->left_target_cm_s=f->right_target_cm_s=80;f->braking_active=false;
 f->route.state=ROUTE_CLEARING;f->route.passing_junction=true;
 f->route.junctions_seen=1;f->route.phase_ms=f->route.capture_line_ms=*t;
 f->route.clear_cm=f->route.capture_line_cm=f->odometry.distance_cm;
 f->route.entry_heading=f->odometry.heading_rad;
}
static void test_pass_cap(bool recovery)
{
 const uint8_t observations[]={0,0xff,0x99};
 for(unsigned side=0;side<2;++side) for(unsigned obs=0;obs<3;++obs) {
  RouteStep s[]={{4,50,side?RIGHT:LEFT,200,0,false,219, 0},{1,50,STOP,200,0,false,70, 0}};
  RouteStep copy[2];memcpy(copy,s,sizeof(s));
  LineFollow f;EncoderCounts c;uint32_t t;seed_pass(&f,&c,&t,s);
  float origin=f.route.origin_cm;
  if(recovery) {
   f.route.component_recovery_active=f.route.component_recovery_used=true;
   f.route.component_recovery_ms=t;f.route.component_recovery_cm=f.odometry.distance_cm;
   f.route.component_recovery_heading=f.odometry.heading_rad;
  }
  for(unsigned i=0;i<4;++i) {
   tick(&f,&c,&t,40,40,observations[obs],false);
   unchanged(&f,s,copy,2,1,ROUTE_CLEARING,origin);
   CHECK(f.route.passing_junction);
   if(recovery || obs>0) CHECK(f.route.component_recovery_active);
   NEAR(f.left_target_cm_s,f.right_target_cm_s);
   capped(&f);
  }
 }
}
static void test_reliable_pass_speed(void)
{
 for(unsigned side=0;side<2;++side) {
  RouteStep s[]={{4,50,side?RIGHT:LEFT,200,0,false,219, 0},{1,50,STOP,200,0,false,70, 0}};
  RouteStep copy[2];memcpy(copy,s,sizeof(s));
  LineFollow f;EncoderCounts c;uint32_t t;seed_pass(&f,&c,&t,s);
  float origin=f.route.origin_cm;
  for(unsigned i=0;i<4;++i) {
   tick(&f,&c,&t,40,40,0x18,false);
   unchanged(&f,s,copy,2,1,ROUTE_CLEARING,origin);
   CHECK(!f.route.component_recovery_active && !f.route.search_pending);
   if(i>=LINE_CONFIRM_SAMPLES-1U)
    CHECK(outer_speed(&f)>16); /* Confirmed center retains the long-segment plan. */
  }
 }
}
static void test_gap_and_entry(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  RouteStep s[]={{1,50,mirror?RIGHT:LEFT,200,0,false,219, 0},
                 {1,50,STOP,200,0,false,70, 0}};
  LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
  f.startup_elapsed_s=1;f.base=f.left_target_cm_s=f.right_target_cm_s=16;
  f.route.origin_cm=f.route.final_segment_origin_cm=f.odometry.distance_cm-220;
  f.abnormal=true;f.abnormal_origin_cm=f.odometry.distance_cm;
  f.last_reliable_ms=t;f.hold_heading_rad=f.odometry.heading_rad+(mirror?.5f:-.5f);
  tick(&f,&c,&t,4,4,0,false);
  CHECK(f.route.state==ROUTE_FOLLOW && f.route.search_pending);
  CHECK(f.state==LINE_GAP_HOLD);capped(&f);
  direction(&f,mirror?1:-1);
  s[0].segment_length_cm=65;start(&f,&c,&t,s,2);
  for(unsigned i=0;i<220;++i) tick(&f,&c,&t,4,4,0x18,false);
  tick(&f,&c,&t,6,6,mirror?0xf8:0x1f,false);
  CHECK(f.route.state==ROUTE_CANDIDATE);
  tick(&f,&c,&t,6,6,mirror?0xf8:0x1f,false);
  CHECK(f.route.state==ROUTE_EXECUTING && f.route.junctions_seen==1);
  CHECK(!f.route.navigation_recoveries && f.state!=LINE_STOPPED);
  /* Front detection precedes the pivot by the sensor-offset track turn-in. */
  CHECK(f.route.turn_approaching);
  CHECK(fabsf(f.left_target_cm_s-f.right_target_cm_s)<.01f);
  f.route.turn_approaching=false;
  tick(&f,&c,&t,6,6,mirror?0xf8:0x1f,false);
  if(mirror) CHECK(f.left_target_cm_s>f.right_target_cm_s);
  else CHECK(f.right_target_cm_s>f.left_target_cm_s);
 }
}
int main(int argc,char **argv)
{
 if(argc<2 || strcmp(argv[1],"gap")==0) test_gap_and_entry();
 if(argc<2 || strcmp(argv[1],"short")==0) test_short_turn_cap();
 if(argc<2 || strcmp(argv[1],"long")==0) test_long_turn_cap();
 if(argc<2 || strcmp(argv[1],"clear")==0) test_pass_cap(false);
 if(argc<2 || strcmp(argv[1],"recovery")==0) test_pass_cap(true);
 if(argc<2 || strcmp(argv[1],"center")==0) test_reliable_pass_speed();
 printf("PASS: low-speed outside-wheel regression (%u checks)\n",checks);
 return 0;
}
