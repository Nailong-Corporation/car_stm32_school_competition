/* HEXAGON ring: pose-driven polygon FSM (2026-11-07).
 *
 * The ring is a lollipop with one external connection at the right vertex V0
 * (345,165).  The maneuver is driven by the known 35 cm edges / 60 deg vertices
 * with odometry + IMU: entry turn 30 deg, five 60 deg vertices, exit turn
 * 30 deg, then the grey line confirms the outgoing line.  This test feeds the
 * axle pose along that polygon and checks the FSM transitions and the guards.
 * The legacy grey-only entry heuristics were retired with the pose maneuver. */
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

/* Ring vertex i along the traversal: clockwise (+360) leaves V0 via the
 * bottom-right edge, counter-clockwise (-360) via the top-right edge. */
static void vertex(int sign,int i,float *x,float *y)
{
    int k=(sign>0)?(6-(i%6))%6:(i%6);
    float a=-(float)k*(float)M_PI/3.0f;
    *x=HEXAGON_CENTER_X_CM+HEXAGON_SIDE_CM*cosf(a);
    *y=HEXAGON_CENTER_Y_CM+HEXAGON_SIDE_CM*sinf(a);
}

static void setup(RoutePlanner *p,int sign,float *h0)
{
    static RouteStep s[2];
    s[0]=(RouteStep){1,16.0f,HEXAGON,60,(float)sign*360.0f,false,50.0f,0};
    s[1]=(RouteStep){1,16.0f,STOP,60,0.0f,false,50.0f,0};
    route_init(p,s,2,NULL,NULL);
    CHECK(p->valid);
    p->repeat_count=1;
    route_start(p,0,0);
    /* route_start() rebuilt the waypoint chain; anchor the ring action's corner
     * on the real V0 afterwards (the ring FSM uses the fixed geometry anyway). */
    p->waypoint_x_cm[0]=HEXAGON_CENTER_X_CM+HEXAGON_SIDE_CM;
    p->waypoint_y_cm[0]=HEXAGON_CENTER_Y_CM;
    route_set_speed(p,12.0f,12.0f);
    dist_cm=0;now_ms=0;
    *h0=sign>0?(float)M_PI:0.0f; /* controller heading: south from the north side, north from the south */
}

/* Feed the approach until the pose trigger hands the row over. */
static void enter_ring(RoutePlanner *p,int sign,float h0)
{
    float vx,vy;vertex(sign,0,&vx,&vy);
    float dirx=sign>0?0.0f:0.0f,diry=sign>0?1.0f:-1.0f;
    float ax=vx-dirx*(GRAY_FORWARD_OFFSET_CM+2.0f);
    float ay=vy-diry*(GRAY_FORWARD_OFFSET_CM+2.0f);
    for(int i=0;i<6 && p->state!=ROUTE_EXECUTING;i++) feed(p,0x18,h0,ax,ay,0.0f);
    CHECK(p->state==ROUTE_EXECUTING);
    CHECK(p->hexagon.state==HEXAGON_ENTERING);
    /* Hold straight until the axle is one turn-lead short of V0. */
    int guard=0;
    while(p->hexagon.state==HEXAGON_ENTERING && ++guard<120)
        feed(p,0x18,h0,ax+=dirx*0.4f,ay+=diry*0.4f,0.4f);
    CHECK(p->hexagon.state==HEXAGON_FOLLOWING);
}

static void drive_ring(RoutePlanner *p,int sign,float h0)
{
    float cum[7]={0};
    for(int k=0;k<7;++k) {
        cum[k]=(k>0?cum[k-1]:0.0f)+
            (k==0||k==6?HEXAGON_ENTRY_TURN_RAD:HEXAGON_TURN_RAD);
    }
    float vx,vy;vertex(sign,0,&vx,&vy);
    float ax=vx,ay=vy;
    float h=h0;
    for(int k=0;k<7;++k) {
        float target=h0+(float)sign*cum[k];
        int guard=0;
        while(p->hexagon.turning && ++guard<200) {
            h=(fabsf(target-h)<0.05f)?target:h+(target-h)*0.5f;
            feed(p,0x18,h,ax,ay,0.0f);
        }
        CHECK(!p->hexagon.turning);
        if(k==6) break;
        vertex(sign,k+1,&vx,&vy);
        guard=0;
        while(!p->hexagon.turning && hypotf(vx-ax,vy-ay)>0.6f && ++guard<200) {
            float dx=vx-ax,dy=vy-ay;
            float len=hypotf(dx,dy);
            ax+=dx/len*0.8f;ay+=dy/len*0.8f;
            feed(p,0x18,h,ax,ay,0.8f);
        }
        CHECK(p->hexagon.turning);
    }
    CHECK(p->hexagon.state==HEXAGON_DONE);
    CHECK(p->state==ROUTE_CLEARING);
    CHECK(p->line_exit_confirmed);
}

static void test_ring_completion(void)
{
    for(int side=0;side<2;++side) {
        int sign=side?1:-1;
        RoutePlanner p;float h0;setup(&p,sign,&h0);
        enter_ring(&p,sign,h0);
        drive_ring(&p,sign,h0);
        /* Pose clearance hands the next row over after 2 cm. */
        float vx,vy;vertex(sign,0,&vx,&vy);
        for(int i=0;i<8;i++) feed(&p,0x18,h0,vx,vy,0.5f);
        CHECK(p.index==1 && p.hexagon.state==HEXAGON_OFF);
    }
}

static void test_guards(void)
{
    /* Wrong way: net turn opposite to the configured direction. */
    RoutePlanner p;float h0;setup(&p,1,&h0);enter_ring(&p,1,h0);
    feed(&p,0x18,h0-0.5f,345,165,0.0f);
    CHECK(p.failure==ROUTE_FAILURE_WRONG_WAY);
    /* Over-rotation beyond the configured minimum angle + 120 deg. */
    setup(&p,1,&h0);enter_ring(&p,1,h0);
    feed(&p,0x18,h0+6.283f+HEXAGON_OVERANGLE_RAD+0.2f,345,165,0.0f);
    CHECK(p.failure==ROUTE_FAILURE_TURN_LIMIT);
    /* Action deadline and distance bound. */
    setup(&p,1,&h0);enter_ring(&p,1,h0);
    now_ms=p.hexagon.action_ms+HEXAGON_ACTION_TIMEOUT_MS;
    route_update_pose(&p,gray_classify(0x18,true,0),dist_cm,h0,now_ms,345,165);
    CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
    setup(&p,1,&h0);enter_ring(&p,1,h0);
    dist_cm=p.entry_cm+HEXAGON_ACTION_MAX_CM;
    feed(&p,0x18,h0,345,165,0.0f);
    CHECK(p.failure==ROUTE_FAILURE_ACTION_TIMEOUT);
    /* Blind ring loss is bounded by the pose cross-track, not the grey timer. */
    setup(&p,1,&h0);enter_ring(&p,1,h0);
    float vx,vy;vertex(1,0,&vx,&vy);
    float nx,ny;vertex(1,1,&nx,&ny);
    float ex=nx-vx,ey=ny-vy,el=hypotf(ex,ey);
    float hturn=h0+HEXAGON_ENTRY_TURN_RAD;
    for(int i=0;i<20 && p.hexagon.turning;i++) feed(&p,0x18,hturn,vx,vy,0.0f);
    CHECK(!p.hexagon.turning);
    /* 15 cm along the leg, then 13 cm to the side. */
    float px=vx+ex/el*15.0f-ey/el*13.0f;
    float py=vy+ey/el*15.0f+ex/el*13.0f;
    for(int i=0;i<80 && p.failure==ROUTE_FAILURE_NONE;i++)
        feed(&p,0,hturn,px,py,0.2f);
    CHECK(p.failure==ROUTE_FAILURE_EXIT_LOST);
}

static void test_entry_deadline(void)
{
    RoutePlanner p;float h0;setup(&p,1,&h0);
    float vx,vy;vertex(1,0,&vx,&vy);
    float ay=vy-(GRAY_FORWARD_OFFSET_CM+2.0f);
    for(int i=0;i<4;i++) feed(&p,0x18,h0,345,ay,0.0f);
    CHECK(p.state==ROUTE_EXECUTING);
    /* Never reaching V0 must hit the bounded entry approach. */
    for(int i=0;i<120 && p.failure==ROUTE_FAILURE_NONE;i++) feed(&p,0x18,h0,345,ay-=0.4f,0.4f);
    CHECK(p.failure==ROUTE_FAILURE_ENTRY_SPEED);
}

int main(void)
{
    test_ring_completion();
    test_guards();
    test_entry_deadline();
    printf("PASS: pose hexagon ring polygon, guards and bounded entry (%u checks)\n",checks);
    return 0;
}
