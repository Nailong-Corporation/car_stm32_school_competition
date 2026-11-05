#define main nav_main
#include "test_navigation.c"
#undef main
int main(void){
 /* Real table step 2: count 4, length 180 from previous action completion. */
 RouteStep s[]={{4,50,CUSTOM_TURN,200,-90,false,180},{1,50,STOP,200,0,false,0}};
 RoutePlanner p;init(&p,s,2);float d=0;uint32_t t=0;
 d=200; /* total-segment distance already exceeded, only 1 of 4 junctions */
 junction(&p,&d,&t,0xff,0);
 printf("seen=%u passing=%d state=%d (1=should pass straight)\n",
   p.junctions_seen,p.passing_junction,(int)p.state);
 return 0;
}
