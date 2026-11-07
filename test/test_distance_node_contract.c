/* Latest user contract: maximum node count OR distance; always confirm a node. */
#define main historical_navigation_main
#include "test_navigation.c"
#undef main

int main(void)
{
 RouteStep steps[]={{4,12,LEFT,30,0,false,105, 0},{1,12,STOP,30,0,false,0, 0}};
 RoutePlanner p;float d=0;uint32_t t=0;
 init(&p,steps,2);
 CHECK(!route_target_ready(&p,104.99f,true));
 CHECK(route_target_ready(&p,105,true));
 /* Reaching 105 cm (110 cm geometry minus 5) cannot turn on plain line. */
 d=105;update(&p,&d,&t,0x18,0,8);
 CHECK(p.state==ROUTE_FOLLOW && p.junctions_seen==0);
 update(&p,&d,&t,0,0,1);
 CHECK(p.state==ROUTE_FOLLOW && p.junctions_seen==0);
 /* A following confirmed node acts even though the maximum count is not met. */
 junction(&p,&d,&t,0xff,0);
 CHECK(p.state==ROUTE_EXECUTING && p.junctions_seen==1 && !p.passing_junction);
 /* Count includes the candidate node and can win before distance. */
 init(&p,steps,2);p.junctions_seen=3;
 CHECK(route_target_ready(&p,10,true));CHECK(!route_target_ready(&p,10,false));
 d=10;t=0;junction(&p,&d,&t,0xff,0);
 CHECK(p.state==ROUTE_EXECUTING && p.junctions_seen==4);
 /* Zero disables that individual condition. */
 steps[0].junction_count=0;init(&p,steps,2);p.junctions_seen=40;
 CHECK(!route_target_ready(&p,20,true));CHECK(route_target_ready(&p,105,true));
 steps[0].junction_count=4;steps[0].segment_length_cm=0;init(&p,steps,2);
 CHECK(!route_target_ready(&p,10000,true));p.junctions_seen=3;
 CHECK(route_target_ready(&p,0,true));
 /* A resistor's centre cap and hollow region form one confirmed node. */
 steps[0].junction_count=4;steps[0].segment_length_cm=1000;init(&p,steps,2);d=0;t=0;
 update(&p,&d,&t,0x18,0,4);
 update(&p,&d,&t,0x3c,0,1);
 update(&p,&d,&t,0x24,0,5);
 CHECK(p.junctions_seen==1 && p.state==ROUTE_CLEARING && p.passing_junction);
 update(&p,&d,&t,0x24,0,8);
 CHECK(p.junctions_seen==1);
 printf("PASS: distance OR maximum count, confirmed node required, resistor counted once (%u checks)\n",checks);
 return 0;
}
