#include "route.h"
#include <assert.h>
int main(void) {
 const RouteStep steps[]={{3,50,LEFT,200,0,0,0},{1,50,STOP,200,0,0,0}};
 RoutePlanner p;route_init(&p,steps,2,0,0);route_start(&p,0,0);
 for(unsigned i=1;i<=8;++i)
  route_update(&p,gray_classify(i<=4?0x18:0x66,1,0),.06f*i,0,5U*i);
 assert(p.junctions_seen==1);
 return 0;
}
