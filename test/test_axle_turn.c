#include "gray_sensor.h"
#include "route.h"
#include "motion_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
 for(unsigned mirror=0;mirror<2;++mirror) {
  RouteStep steps[]={{1,16,CUSTOM_TURN,60,mirror?90:-90,false,60},
                     {1,16,STOP,60,0,false,100}};
  RoutePlanner p;route_init(&p,steps,2,NULL,NULL);p.repeat_count=1;route_start(&p,0,0);
  float d=0;uint32_t t=0;float sign=mirror?1:-1;
  for(unsigned i=0;i<4;++i){d+=.06f;t+=5;route_update(&p,gray_classify(0x18,true,0),d,0,t);}
  for(unsigned i=0;i<3;++i){d+=.06f;t+=5;route_update(&p,gray_classify(0xff,true,0),d,0,t);}
  assert(p.state==ROUTE_EXECUTING && p.turn_approaching && p.junctions_seen==1);
  float origin=p.turn_approach_origin_cm;
  for(unsigned i=0;i<100;++i){d+=.1f;t+=5;route_update(&p,gray_classify(0,true,0),d,0,t);}
  assert(p.turn_approaching && p.state==ROUTE_EXECUTING);
  d=origin+GRAY_FORWARD_OFFSET_CM-WHEEL_TRACK_CM*.5f+.01f;t+=5;
  route_update(&p,gray_classify(0,true,0),d,0,t);
  assert(!p.turn_approaching && p.state==ROUTE_EXECUTING);
  for(unsigned i=0;i<3;++i){d+=.06f;t+=5;route_update(&p,gray_classify(0x18,true,0),d,sign*.8f,t);}
  assert(p.state==ROUTE_EXECUTING && !p.line_exit_confirmed);
  for(unsigned i=0;i<3;++i){d+=.06f;t+=5;route_update(&p,gray_classify(0x18,true,0),d,sign*1.55f,t);}
  assert(p.state==ROUTE_CLEARING && p.line_exit_confirmed);
 }
 puts("PASS: mirrored axle approach, no-line transit and rejection of 46-degree completion");
 return 0;
}
