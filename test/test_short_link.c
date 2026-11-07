/* Explicit short geometry is authorized by the preceding real turn exit only. */
#include "route.h"
#include "motion_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static unsigned checks;
#define CHECK(x) do { ++checks; assert(x); } while(0)
static RouteStep steps[3];
static float d;static uint32_t now;
static void sample(RoutePlanner *p,uint8_t mask,float ds,float heading)
{d+=ds;now+=5;route_update(p,gray_classify(mask,true,0),d,heading,now);}
static void setup(RoutePlanner *p,int sign,int next_sign)
{
 steps[0]=(RouteStep){1,16,CUSTOM_TURN,60,sign*90,false,40,0};
 steps[1]=(RouteStep){1,16,CUSTOM_TURN,60,next_sign*90,false,25,30};
 steps[2]=(RouteStep){1,16,STOP,60,0,false,50,0};
 route_init(p,steps,3,NULL,NULL);p->repeat_count=1;route_start(p,0,0);
 d=0;now=0;CHECK(p->valid);
}
static void arm_radius(RoutePlanner *p,int sign,int next_sign,float radius)
{
 setup(p,sign,next_sign);
 for(int i=0;i<4;i++)sample(p,0x18,.08f,0);
 for(int i=0;i<4;i++)sample(p,0xff,.08f,0);
 CHECK(p->state==ROUTE_EXECUTING && p->turn_approaching);
 sample(p,0,TURN_PIVOT_APPROACH_CM+.01f,0);
 CHECK(p->short_link_tracking && !p->short_link_active);
 /* Integrate a real axle arc, not its scalar length. No first exit => no arm. */
 for(int i=1;i<=100;i++) {
  sample(p,i>=98?0x18:0,radius*JUNCTION_TURN_TARGET_RAD/100,sign*JUNCTION_TURN_TARGET_RAD*i/100);
  if(i<100)CHECK(p->index==0 && !p->short_link_active);
 }
 CHECK(p->index==1 && p->state==ROUTE_EXECUTING && p->turn_approaching && p->short_link_active);
 CHECK(p->junctions_seen==0); /* No synthetic node count. */
 CHECK(fabsf(p->short_link_forward_cm-radius)<.01f);
 CHECK(fabsf(fabsf(p->short_link_lateral_cm)-radius)<.01f);
}
static void arm(RoutePlanner *p,int sign,int next_sign)
{arm_radius(p,sign,next_sign,WHEEL_TRACK_CM*.5f);}
static void test_mirrors(void)
{
 for(int sign=-1;sign<=1;sign+=2)for(int next_sign=-1;next_sign<=1;next_sign+=2) {
  RoutePlanner p;arm(&p,sign,next_sign);
  const float axis=sign*JUNCTION_TURN_TARGET_RAD;
  for(int i=0;i<190;i++)sample(&p,0,.08f,axis);
  CHECK(p.turn_approaching && p.short_link_active && p.index==1);
  while(p.turn_approaching)sample(&p,0,.08f,axis);
  CHECK(p.state==ROUTE_EXECUTING && p.index==1 && !p.short_link_active);
  CHECK(p.short_link_forward_cm>=23 && p.short_link_forward_cm<23.1f);
  /* Arriving at the next angle with no sensor line never claims success. */
  float final=axis+next_sign*JUNCTION_TURN_TARGET_RAD;
  for(int i=0;i<35;i++)sample(&p,0,.01f,final);
  CHECK(p.index==1 && p.state==ROUTE_EXECUTING && p.turn_rewind_active);
  for(int i=0;i<3;i++)sample(&p,0x18,.08f,final-next_sign*JUNCTION_EXIT_REWIND_RAD);
  CHECK(p.state==ROUTE_CLEARING && p.index==1 && p.line_exit_confirmed);
  for(int i=0;i<40;i++)sample(&p,0x18,.08f,final-next_sign*JUNCTION_EXIT_REWIND_RAD);
  CHECK(p.index==2 && p.state==ROUTE_FOLLOW && !p.stop_pending);
 }
}
static void test_gates(void)
{
 RoutePlanner p;arm_radius(&p,1,1,9.0f);float axis=JUNCTION_TURN_TARGET_RAD;
 CHECK(fabsf(p.short_link_pivot_lead_cm-9.0f)<.01f);
 while(p.turn_approaching)sample(&p,0,.08f,axis);
 CHECK(p.short_link_forward_cm>=21.0f && p.short_link_forward_cm<21.1f);
 arm(&p,1,1);
 route_set_speed(&p,25,25);
 while(p.state==ROUTE_EXECUTING)sample(&p,0,.08f,axis);
 CHECK(p.state==ROUTE_BRAKING && p.short_link_active && p.turn_approaching);
 for(int i=0;i<4;i++)sample(&p,0,0,axis);
 CHECK(p.state==ROUTE_BRAKING);route_set_speed(&p,0,0);sample(&p,0,0,axis);
 CHECK(p.state==ROUTE_EXECUTING && !p.turn_approaching && p.index==1);
 /* No exit after the pivot still uses the ordinary action deadline. */
 for(int i=0;i<700 && p.state==ROUTE_EXECUTING;i++)sample(&p,0,0,axis+JUNCTION_TURN_TARGET_RAD);
 CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_ACTION_TIMEOUT && p.index==1);
 arm(&p,1,1);sample(&p,0,.08f,axis+.31f);
 CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_EXIT_RECAPTURE);
 arm(&p,1,1);now+=CONTROL_MAX_INTERVAL_MS;sample(&p,0,.08f,axis);
 CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_EXIT_RECAPTURE);
 arm(&p,1,1);sample(&p,0,-.02f,axis);
 CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_EXIT_RECAPTURE);
 arm(&p,1,1);sample(&p,0,30,axis);
 CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_EXIT_RECAPTURE);
 arm(&p,1,1);
 /* The lateral budget is 14 cm (raised with the faster pivots), so drive a
  * clearly larger lateral projection before expecting the fault. */
 for(int i=0;i<400 && p.state==ROUTE_EXECUTING;i++)sample(&p,0,.08f,axis-.5f);
 CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_EXIT_RECAPTURE);
 arm(&p,1,1);p.short_link_ms=now-6000U;sample(&p,0,.08f,axis);
 CHECK(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_EXIT_RECAPTURE);
 arm(&p,1,1);route_start(&p,d,now);CHECK(!p.short_link_active && !p.short_link_tracking);
 setup(&p,1,1);steps[1].short_link_cm=0;route_init(&p,steps,3,NULL,NULL);route_start(&p,0,0);
 for(int i=0;i<100;i++)sample(&p,0,.5f,0);
 CHECK(p.index==0 && p.state==ROUTE_FOLLOW && !p.short_link_active);
}
static void test_validation(void)
{
 RoutePlanner p;setup(&p,1,1);
 steps[0].short_link_cm=30;route_init(&p,steps,3,NULL,NULL);CHECK(!p.valid);steps[0].short_link_cm=0;
 steps[2].short_link_cm=30;route_init(&p,steps,3,NULL,NULL);CHECK(!p.valid);steps[2].short_link_cm=0;
 float invalid[]={-1,19,41,NAN,INFINITY};
 for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++) {steps[1].short_link_cm=invalid[i];route_init(&p,steps,3,NULL,NULL);CHECK(!p.valid);}
 steps[1].short_link_cm=30;steps[1].angle_deg=45;route_init(&p,steps,3,NULL,NULL);CHECK(!p.valid);
 steps[1].angle_deg=90;steps[0].action=STRAIGHT;route_init(&p,steps,3,NULL,NULL);CHECK(!p.valid);
}
int main(void)
{test_mirrors();test_gates();test_validation();printf("PASS: explicit short links, projection, mirrored turns, entry speed, rewind, deadlines and STOP exclusion (%u checks)\n",checks);}
