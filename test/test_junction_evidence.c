#define main existing_navigation_regressions
#include "test_navigation.c"
#undef main

static const RouteStep evidence_steps[]={
 {1,50,LEFT,200,0,false,0},{1,50,STOP,200,0,false,0}
};

static void test_branch_identity(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  RoutePlanner p;float d=0;uint32_t t=0;init(&p,evidence_steps,2);
  update(&p,&d,&t,0x18,0,4);
  for(unsigned i=0;i<8;++i) update(&p,&d,&t,(i+mirror)%2?0x1f:0xf8,0,1);
  CHECK(p.junctions_seen==0 && p.candidate_restarts>0);
  /* A bilateral middle frame must not bridge unrelated left/right evidence. */
  init(&p,evidence_steps,2);d=0;t=0;update(&p,&d,&t,0x18,0,4);
  update(&p,&d,&t,mirror?0xf8:0x1f,0,1);update(&p,&d,&t,0xff,0,1);
  update(&p,&d,&t,mirror?0x1f:0xf8,0,1);
  CHECK(p.junctions_seen==0 && p.candidate_count==1);
  update(&p,&d,&t,mirror?0x1f:0xf8,0,2);
  CHECK(p.junctions_seen==1 && p.last_confirmed_kind==JUNCTION_EVIDENCE_CENTER_BRANCH);
 }
 RoutePlanner p;float d=0;uint32_t t=0;init(&p,evidence_steps,2);
 update(&p,&d,&t,0x18,0,4);update(&p,&d,&t,0x9b,0,5);
 CHECK(p.junctions_seen==0); /* three regions including a center and side pair */
 init(&p,evidence_steps,2);d=0;t=0;update(&p,&d,&t,0x18,0,4);
 d+=1;t+=5;route_update(&p,mask(0x1f),d,0,t);
 d+=2;t+=5;route_update(&p,mask(0xf8),d,0,t);
 CHECK(p.state==ROUTE_FOLLOW && p.junctions_seen==0);
 update(&p,&d,&t,0xf8,0,5);CHECK(p.junctions_seen==0);
}

static void test_curve_and_weak_edges(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  RoutePlanner p;float d=0;uint32_t t=0;init(&p,evidence_steps,2);
  update(&p,&d,&t,0x18,0,4);
  update(&p,&d,&t,mirror?0x0c:0x30,0,1);
  update(&p,&d,&t,mirror?0x06:0x60,0,1);
  update(&p,&d,&t,mirror?0x03:0xc0,0,8);
  CHECK(p.junctions_seen==0 && p.smooth_edge_rejections>0);
  update(&p,&d,&t,0,0,1);update(&p,&d,&t,mirror?0x03:0xc0,0,4);
  CHECK(p.junctions_seen==0); /* noise does not unlock the same drifting edge */
  update(&p,&d,&t,0x18,0,4);
  update(&p,&d,&t,mirror?0x03:0xc0,0,3);
  CHECK(p.junctions_seen==0 && p.candidate_kind==JUNCTION_EVIDENCE_EDGE);
  update(&p,&d,&t,mirror?0x03:0xc0,0,1);
  CHECK(p.junctions_seen==1 && p.last_confirmed_kind==JUNCTION_EVIDENCE_EDGE);

  init(&p,evidence_steps,2);d=0;t=0;update(&p,&d,&t,0x18,0,4);
  update(&p,&d,&t,mirror?0x8c:0x31,0,3);
  CHECK(p.junctions_seen==0 && p.candidate_kind==JUNCTION_EVIDENCE_SEPARATED);
  update(&p,&d,&t,mirror?0x8c:0x31,0,1);
  CHECK(p.junctions_seen==1);
  init(&p,evidence_steps,2);d=0;t=0;update(&p,&d,&t,0x18,0,4);
  update(&p,&d,&t,mirror?0xd8:0x1b,0,3);
  CHECK(p.junctions_seen==0 && p.candidate_kind==JUNCTION_EVIDENCE_SEPARATED);
  update(&p,&d,&t,mirror?0xd8:0x1b,0,1);CHECK(p.junctions_seen==1);
 }
 /* Neither samples alone nor motion alone confirms a weak branch. */
 RoutePlanner p;float d=0;uint32_t t=0;init(&p,evidence_steps,2);
 update(&p,&d,&t,0x18,0,4);
 for(unsigned i=0;i<6;++i) {t+=5;route_update(&p,mask(0x03),d,0,t);}
 CHECK(p.junctions_seen==0);
 t+=CONTROL_MAX_INTERVAL_MS+1;d+=.5f;route_update(&p,mask(0x03),d,0,t);
 CHECK(p.junctions_seen==0 && p.candidate_reject==JUNCTION_REJECT_SAMPLE_GAP);
}

static void test_raw_filter_replay(void)
{
 /* Replay actual raw GPIO through polarity/majority filtering at two speeds.
  * These are synthetic labeled inputs, not invented 5ms hardware history. */
 for(unsigned fast=0;fast<2;++fast) for(unsigned weak=0;weak<2;++weak) {
  RoutePlanner p;GraySensor sensor;gray_sensor_init(&sensor);init(&p,evidence_steps,2);
  float d=0;uint32_t t=0;
  for(unsigned i=0;i<4;++i) {
   t+=5;d+=fast?1.0f:.25f;
   route_update(&p,gray_sensor_update(&sensor,raw_for_line(0x18),true,0),d,0,t);
  }
  unsigned frames=weak?5:4;
  for(unsigned i=0;i<frames;++i) {
   t+=5;d+=fast?1.0f:.25f;
   route_set_speed(&p,fast?200:50,fast?200:50);
   route_update(&p,gray_sensor_update(&sensor,raw_for_line(weak?0x03:0xff),true,0),d,0,t);
   if(i+1<frames) CHECK(p.junctions_seen==0);
  }
  CHECK(p.junctions_seen==1);
  CHECK(p.state==(fast?ROUTE_BRAKING:ROUTE_EXECUTING));
 }
 for(unsigned mirror=0;mirror<2;++mirror) {
  RoutePlanner p;GraySensor sensor;gray_sensor_init(&sensor);init(&p,evidence_steps,2);
  float d=0;uint32_t t=0;
  const uint8_t curve[]={0x18,0x30,0x60,0xc0,0xc0};
  for(unsigned group=0;group<sizeof(curve);++group) for(unsigned i=0;i<4;++i) {
   uint8_t m=curve[group];
   if(mirror) {uint8_t reverse=0;for(unsigned bit=0;bit<8;++bit) if(m&(1U<<bit)) reverse|=(uint8_t)(1U<<(7U-bit));m=reverse;}
   t+=5;d+=.06f;
   route_update(&p,gray_sensor_update(&sensor,raw_for_line(m),true,0),d,0,t);
  }
  CHECK(p.junctions_seen==0 && p.smooth_edge_rejections>0);
 }
}

static void test_adjacent_branch_identity(void)
{
 RouteStep steps[]={{1,50,LEFT,50,0,false,0},{2,50,LEFT,200,0,false,0},{1,50,STOP,200,0,false,0}};
 for(unsigned mirror=0;mirror<2;++mirror) {
  RoutePlanner p;float d=0;uint32_t t=0;init(&p,steps,3);
  junction(&p,&d,&t,0xff,0);update(&p,&d,&t,0x18,-1.65f,3);
  update(&p,&d,&t,0x18,-1.65f,8);
  for(unsigned i=0;i<8;++i) update(&p,&d,&t,(i+mirror)%2?0x31:0x8c,-1.65f,1);
  CHECK(p.index==0 && p.state==ROUTE_CLEARING && p.candidate_restarts>0);
 }
}

static void test_trace_freeze(void)
{
 LineFollow f;EncoderCounts c;uint32_t t;start(&f,&c,&t,evidence_steps,2);
 for(unsigned i=0;i<140;++i) tick(&f,&c,&t,5,5,0x18,false);
 CHECK(f.navigation_trace.count==NAVIGATION_TRACE_CAPACITY);
 unsigned last=(f.navigation_trace.next+NAVIGATION_TRACE_CAPACITY-1U)%NAVIGATION_TRACE_CAPACITY;
 CHECK(f.navigation_trace.samples[last].ms==t);
 CHECK(f.navigation_trace.samples[last].counts.left==c.left);
 CHECK(f.navigation_trace.samples[last].route_index==f.route.index);
 CHECK(f.navigation_trace.samples[last].junctions_seen==f.route.junctions_seen);
 t+=20;line_follow_update(&f,mask(0x18),c,t,.020f,false);
 CHECK(f.state==LINE_STOPPED);
 last=(f.navigation_trace.next+NAVIGATION_TRACE_CAPACITY-1U)%NAVIGATION_TRACE_CAPACITY;
 CHECK(f.navigation_trace.samples[last].ms==t);
 CHECK(f.navigation_trace.samples[last].stop_reason==LINE_STOP_FAULT);
 unsigned next=f.navigation_trace.next;
 for(unsigned i=0;i<10;++i) tick(&f,&c,&t,0,0,0x18,i==9);
 CHECK(f.navigation_trace.next==next && f.start_result==LINE_START_REJECTED_FAULT);
 start(&f,&c,&t,evidence_steps,2);
 for(unsigned i=0;i<6;++i) tick(&f,&c,&t,5,5,0x18,false);
 line_follow_stop(&f,LINE_STOP_REQUEST);
 for(unsigned i=0;i<LINE_CONFIRM_SAMPLES;++i) tick(&f,&c,&t,0,0,0x18,i+1==LINE_CONFIRM_SAMPLES);
 CHECK(f.start_result==LINE_START_ACCEPTED && f.navigation_trace.count==1);
}

static void test_captured_component_stop(const char *path)
{
 FILE *file=fopen(path,"r");CHECK(file!=NULL);
 RoutePlanner p;init(&p,evidence_steps,2);
 p.state=ROUTE_EXECUTING;p.phase_ms=8493;p.last_sample_ms=9358;
 p.entry_heading=-1.58434951f;p.capture_line_seen=true;
 p.capture_line_ms=9358;p.capture_line_cm=227.019699f;
 char line[512];CHECK(fgets(line,sizeof(line),file)!=NULL);unsigned used=0;
 while(fgets(line,sizeof(line),file)) {
  unsigned ms,m,kind,phase,capturing;float d,turn,pos,lt,rt,lm,rm;
  CHECK(sscanf(line,"%u,%f,%f,%f,%f,%f,%f,%f,%u,%u,%u,%u",
   &ms,&d,&turn,&pos,&lt,&rt,&lm,&rm,&m,&kind,&phase,&capturing)==12);
  if(ms<9373) continue;
  route_set_speed(&p,lm,rm);route_update(&p,mask((uint8_t)m),d,p.entry_heading+turn,ms);
  CHECK(p.state==ROUTE_EXECUTING && !p.line_exit_confirmed);++used;
 }
 fclose(file);CHECK(used==20);
 route_update(&p,mask(0x3b),232.329620f,p.entry_heading-1.71601951f,9758);
 CHECK(p.failure==ROUTE_FAILURE_NONE && p.state==ROUTE_EXECUTING && p.component_recovery_active);
 CHECK(p.index==0 && p.junctions_seen==0 && !p.line_exit_confirmed);
 route_update(&p,mask(0x18),232.4f,p.entry_heading-1.716f,
              p.component_recovery_ms+COMPONENT_RECOVERY_TIMEOUT_MS);
 CHECK(p.failure==ROUTE_FAILURE_EXIT_RECAPTURE && p.state==ROUTE_FAILED);
 /* The user now authorizes a one-shot straight recovery through the graphic.
  * It still cannot complete a turn without line evidence or run forever. */
}

int main(int argc,char **argv)
{
 CHECK(argc==2);
 test_branch_identity();test_curve_and_weak_edges();test_raw_filter_replay();
 test_adjacent_branch_identity();test_trace_freeze();
 test_captured_component_stop(argv[1]);
 printf("PASS: junction identity, mirrored curve/L/separated evidence, raw filter replay, frozen trace and captured EXIT_LOST (%u checks)\n",checks);
 return 0;
}
