/* HEXAGON entry: pose trigger, straight approach to the vertex and the bounded
 * entry deadline (2026-11-07 pose maneuver).  The ring action is handed over by
 * the encoder/IMU pose (the 24 cm look-ahead bar cannot see a 35 cm vertex in
 * time), then the axle drives straight to one turn-lead short of V0. */
#include "route.h"
#include "motion_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static unsigned checks;
#define CHECK(x) do{++checks;assert(x);}while(0)

static uint32_t now_ms;
static float dist_cm;

static void feed(RoutePlanner *p,uint8_t m,float h,float x,float y,float ds)
{
    dist_cm+=ds;now_ms+=5;
    route_update_pose(p,gray_classify(m,true,0),dist_cm,h,now_ms,x,y);
}

static void begin(RoutePlanner *p,int sign,float *h0)
{
    static RouteStep s[2];
    s[0]=(RouteStep){1,16.0f,HEXAGON,60,(float)sign*360.0f,false,50.0f,0};
    s[1]=(RouteStep){1,16.0f,STOP,60,0.0f,false,50.0f,0};
    route_init(p,s,2,NULL,NULL);
    CHECK(p->valid);
    p->repeat_count=1;
    route_start(p,0,0);
    /* route_start() rebuilt the waypoint chain; re-anchor the ring corner. */
    p->waypoint_x_cm[0]=HEXAGON_CENTER_X_CM+HEXAGON_SIDE_CM;
    p->waypoint_y_cm[0]=HEXAGON_CENTER_Y_CM;
    route_set_speed(p,12.0f,12.0f);
    dist_cm=0;now_ms=0;
    *h0=sign>0?(float)M_PI:0.0f;
}

int main(void)
{
    for(int sign=-1;sign<=1;sign+=2) {
        RoutePlanner p;float h0;begin(&p,sign,&h0);
        float diry=sign>0?1.0f:-1.0f;
        float ax=HEXAGON_CENTER_X_CM+HEXAGON_SIDE_CM;
        float ay=HEXAGON_CENTER_Y_CM-diry*(GRAY_FORWARD_OFFSET_CM+2.0f);
        /* Pose trigger confirms the row exactly like a grey node would. */
        feed(&p,0x18,h0,ax,ay,0.0f);
        CHECK(p.state==ROUTE_EXECUTING);
        CHECK(p.hexagon.state==HEXAGON_ENTERING);
        CHECK(p.junctions_seen==1);
        /* Straight approach holds the entry heading and does not pivot. */
        float ay2=ay-diry*1.0f;
        feed(&p,0x18,h0,ax,ay2,1.0f);
        CHECK(p.hexagon.state==HEXAGON_ENTERING);
        CHECK(!p.hexagon.turning);
        /* One turn-lead short of V0 starts the first 30 deg pivot. */
        float lead=WHEEL_TRACK_CM*0.5f*tanf(0.5f*HEXAGON_ENTRY_TURN_RAD);
        float ay3=HEXAGON_CENTER_Y_CM+diry*(lead-0.2f);
        feed(&p,0x18,h0,ax,ay3,2.0f);
        CHECK(p.hexagon.state==HEXAGON_FOLLOWING && p.hexagon.turning);
        /* A never-reached vertex must hit the bounded entry approach. */
        RoutePlanner q;begin(&q,sign,&h0);
        float qy=HEXAGON_CENTER_Y_CM-diry*(GRAY_FORWARD_OFFSET_CM+2.0f);
        for(int i=0;i<3;i++) feed(&q,0x18,h0,ax,qy,0.0f);
        CHECK(q.state==ROUTE_EXECUTING);
        for(int i=0;i<200 && q.failure==ROUTE_FAILURE_NONE;i++)
            feed(&q,0x18,h0,ax,qy-=diry*0.4f,0.4f);
        CHECK(q.failure==ROUTE_FAILURE_ENTRY_SPEED);
        /* A blank bar is tolerated in pose mode for a few frames. */
        RoutePlanner r;begin(&r,sign,&h0);
        float ry=HEXAGON_CENTER_Y_CM-diry*(GRAY_FORWARD_OFFSET_CM+2.0f);
        feed(&r,0x18,h0,ax,ry,0.0f);
        CHECK(r.state==ROUTE_EXECUTING);
        for(int i=0;i<4;i++) feed(&r,0, h0,ax,ry,0.0f);
        CHECK(r.state==ROUTE_EXECUTING && r.failure==ROUTE_FAILURE_NONE);
    }
    printf("PASS: pose hexagon entry, approach and bounded deadline (%u checks)\n",checks);
    return 0;
}
