#define main previous_navigation_main
#include "test_navigation.c"
#undef main

/* Synthetic controller snapshots isolate the distance envelope from motor
 * inertia. Full route sequences below use only normal route_update calls. */
static void test_known_length_envelope(void)
{
 const float lengths[]={5,50,65,70,100,100.01f,101,200};
 const unsigned counts[]={0,1,4};
 const Action actions[]={LEFT,RIGHT,STRAIGHT,STOP};
 for(unsigned l=0;l<sizeof(lengths)/sizeof(lengths[0]);++l)
 for(unsigned n=0;n<sizeof(counts)/sizeof(counts[0]);++n)
 for(unsigned a=0;a<sizeof(actions)/sizeof(actions[0]);++a) {
  RouteStep s[]={{counts[n],12,actions[a],200,0,false,lengths[l]},
                 {1,12,STOP,200,0,false,0}};
  size_t size=actions[a]==STOP?1:2;
  LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,size);
  f.startup_elapsed_s=1;
  f.base=f.left_target_cm_s=f.right_target_cm_s=60;
  /* Move the route origin, not the encoders: no artificial jump in feedback. */
  f.route.origin_cm=f.route.final_segment_origin_cm=f.odometry.distance_cm-lengths[l];
  tick(&f,&c,&t,1,1,0x18,false);
  CHECK(f.state!=LINE_STOPPED && f.route.state==ROUTE_FOLLOW);
  CHECK(f.route.index==0 && f.route.junctions_seen==0 && f.route.search_pending);
  CHECK(f.braking_active);
  CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=12.001f);
 }
}
static void test_short_handoffs(void)
{
 for(unsigned mode=0;mode<3;++mode) {
  RouteStep s[]={{4,12,LEFT,200,0,false,200},{1,12,STOP,200,0,false,5}};
  LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,s,2);
  f.startup_elapsed_s=1;f.base=f.left_target_cm_s=f.right_target_cm_s=60;
  f.route.state=ROUTE_CLEARING;
  f.route.phase_ms=t;f.route.clear_cm=f.odometry.distance_cm;
  f.route.passing_junction=mode!=0;
  f.route.junctions_seen=1;
  if(mode==0) {
   /* A long action's clearance must already use the short NEXT envelope. */
   f.route.next_segment_origin_valid=true;
   f.route.next_segment_origin_cm=f.odometry.distance_cm-5;
  } else {
   /* Intermediate clearance and its graphic hold keep the SAME short origin. */
   s[0].segment_length_cm=5;
   f.route.origin_cm=f.route.final_segment_origin_cm=f.odometry.distance_cm-5;
  }
  tick(&f,&c,&t,1,1,mode==2?0x99:0x18,false);
  CHECK(f.state!=LINE_STOPPED && f.route.state==ROUTE_CLEARING);
  CHECK(f.route.index==0 && f.route.junctions_seen==1);
  CHECK(f.braking_active && f.route.search_pending);
  CHECK(fmaxf(f.left_target_cm_s,f.right_target_cm_s)<=12.001f);
  CHECK(f.route.component_recovery_active==(mode==2));
 }
}
static void test_spacing_and_node_matrix(void)
{
 const float lengths[]={0,50,65,100,101,200};
 const float spacings[]={3,10,30};
 const unsigned counts[]={1,2,4};
 for(unsigned l=0;l<sizeof(lengths)/sizeof(lengths[0]);++l)
 for(unsigned g=0;g<sizeof(spacings)/sizeof(spacings[0]);++g)
 for(unsigned n=0;n<sizeof(counts)/sizeof(counts[0]);++n) {
  RouteStep s[]={{counts[n],12,STRAIGHT,200,0,false,lengths[l]},
                 {1,12,STOP,200,0,false,0}};
  RoutePlanner p;init(&p,s,2);float d=0;uint32_t t=0;
  for(unsigned node=1;node<=counts[n];++node) {
   float location=spacings[g]*(float)node;
   while(d<location) update(&p,&d,&t,0x18,0,1);
   junction(&p,&d,&t,node%2?0x1f:0xf8,0);
   CHECK(p.junctions_seen==node && p.index==0);
   /* Segment length is a speed-planning bound, not a node authorization.
    * Only the configured junction count may turn this step into a target. */
   bool target=node>=counts[n];
   CHECK(p.passing_junction==!target);
   CHECK(p.state==(target?ROUTE_EXECUTING:ROUTE_CLEARING));
   float origin=p.origin_cm;
   clear(&p,&d,&t,0);
   CHECK(p.state==ROUTE_FOLLOW && p.index==(target?1U:0U));
   if(target) {
    CHECK(p.junctions_seen==0 && p.origin_cm>origin);
    junction(&p,&d,&t,0xff,0);CHECK(p.state==ROUTE_FINISHED);
    break;
   }
   NEAR(p.origin_cm,origin);
  }
 }
}
int main(void)
{
 test_known_length_envelope();test_short_handoffs();test_spacing_and_node_matrix();
 printf("PASS: race review known-length envelope and spacing/count matrix (%u checks)\n",checks);
 return 0;
}
