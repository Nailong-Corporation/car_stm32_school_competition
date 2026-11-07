#include "line_follow.h"
#include "control_config.h"
#include "motion_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static unsigned checks;
#define CHECK(x) do { ++checks; assert(x); } while(0)
#define NEAR(a,b) CHECK(fabsf((a)-(b))<.001f)

typedef struct {
    LineFollow f;
    RouteStep steps[3];
    EncoderCounts counts;
    uint32_t now;
    float heading;
    int mirror;
} Fixture;

static uint8_t flipped(uint8_t m)
{
    uint8_t r=0;
    for(unsigned i=0;i<8;++i) if(m&(1U<<i)) r|=(uint8_t)(1U<<(7U-i));
    return r;
}
static void sample(Fixture *x,uint8_t mask,int left,int right)
{
    x->counts.left=(uint16_t)(x->counts.left+left);
    x->counts.right=(uint16_t)(x->counts.right+right);
    x->now+=5;
    uint8_t line=x->mirror<0?flipped(mask):mask;
    GrayObservation o=gray_classify(line,true,0);
    line_follow_update_heading(&x->f,o,x->counts,x->now,.005f,false,true,x->heading);
    CHECK(x->f.state!=LINE_STOPPED);
    CHECK(x->f.left_target_cm_s>=0 && x->f.right_target_cm_s>=0);
    CHECK(x->f.left_target_cm_s<=WHEEL_TARGET_MAX_CM_S && x->f.right_target_cm_s<=WHEEL_TARGET_MAX_CM_S);
}
/* A real confirmed target turn -> early center capture -> clearance -> next
 * FOLLOW. The axle still approaches the geometric exit at about 27 degrees. */
static void handoff(Fixture *x,int mirror)
{
    *x=(Fixture){.mirror=mirror};
    x->steps[0]=(RouteStep){1,16,mirror>0?LEFT:RIGHT,60,0,false,0, 0};
    x->steps[1]=(RouteStep){2,16,RIGHT,60,0,false,0, 0};
    x->steps[2]=(RouteStep){1,16,STOP,60,0,false,0, 0};
    OdometryCalibration c={1/(3.14159265359f*ODOMETRY_DISTANCE_SCALE),
        1/(3.14159265359f*ODOMETRY_DISTANCE_SCALE),100,100,10,1,1,true};
    line_follow_init(&x->f,c,x->steps,3,NULL,NULL);
    x->f.route.repeat_count=1;
    x->heading=mirror*JUNCTION_TURN_TARGET_RAD;
    for(unsigned i=0;i<3;++i) {
        x->now+=5;
        line_follow_update_heading(&x->f,gray_classify(0x18,true,0),x->counts,
            x->now,.005f,i==2,true,x->heading);
    }
    CHECK(x->f.state==LINE_TRACKING);
    x->f.startup_elapsed_s=1;
    for(unsigned i=0;i<4;++i) sample(x,0x18,6,6); /* establish centered history */
    for(unsigned i=0;i<6;++i) sample(x,0xff,mirror>0?10:6,mirror>0?6:10);
    CHECK(x->f.route.state==ROUTE_EXECUTING);
    /* Consume the 24 cm − 7 cm wheel-axle approach stage before the pivot. */
    /* The approach stage rebaselines entry_heading at the car's real heading,
     * so the fixture must hold heading straight (0) while it is consumed. */
    for(unsigned i=0;i<400 && x->f.route.turn_approaching;++i) { x->heading=0; sample(x,0x18,60,60); }
    CHECK(!x->f.route.turn_approaching);
    /* Short pivot sweep: 20 loss samples stay inside the one-shot graphic
     * budget that the 17 cm approach stage now consumes part of. */
    for(unsigned i=0;i<20;++i) {
        x->heading=mirror*(JUNCTION_TURN_TARGET_RAD-
            (JUNCTION_TURN_TARGET_RAD-.55f)*(float)(i+1)/20);
        sample(x,0,mirror>0?0:12,mirror>0?12:0);
    }
    for(unsigned i=0;i<8;++i) sample(x,0x18,6,6);
    CHECK(x->f.route.state==ROUTE_CLEARING);
    for(unsigned i=0;i<40 && x->f.route.index==0;++i) {
        x->heading-=mirror*.002f;
        sample(x,0x18,6,6);
    }
    CHECK(x->f.route.index==1 && x->f.route.state==ROUTE_FOLLOW);
    CHECK(x->f.route.junctions_seen==0);
}

static void test_adjacent_pass(int mirror)
{
    Fixture x;handoff(&x,mirror);
    float origin=x.f.route.origin_cm;
    x.heading-=mirror*.002f;sample(&x,0x0c,6,6);
    x.heading-=mirror*.002f;sample(&x,0x0e,6,6);
    CHECK(x.f.route.junctions_seen==0); /* three-channel drift still rejected */
    x.heading-=mirror*.002f;sample(&x,0x0f,6,6);
    CHECK(x.f.route.state==ROUTE_CANDIDATE);
    CHECK(mirror*(x.f.left_target_cm_s-x.f.right_target_cm_s)<0);
    CHECK(x.f.exit_heading_pending);NEAR(x.f.exit_heading_rad,0);
    for(unsigned i=0;i<2;++i) {
        x.heading-=mirror*.002f;sample(&x,0x0f,6,6);
    }
    CHECK(x.f.route.junctions_seen==1 && x.f.route.passing_junction);
    CHECK(x.f.route.state==ROUTE_CLEARING && x.f.route.index==1);
    x.heading-=mirror*.002f;sample(&x,0x0f,6,6);
    CHECK(x.f.route.component_recovery_active);
    CHECK(mirror*(x.f.left_target_cm_s-x.f.right_target_cm_s)<0);
    uint32_t recovery_ms=x.f.route.component_recovery_ms;
    /* An edge inside the graphic is still not a confirmed straight exit. */
    for(unsigned i=0;i<3;++i) sample(&x,0x07,6,6);
    CHECK(x.f.route.component_recovery_active);
    CHECK(x.f.route.component_recovery_ms==recovery_ms);
    NEAR(x.f.route.origin_cm,origin);
    CHECK(x.f.route.index==1 && x.f.route.junctions_seen==1);
    for(unsigned i=0;i<3;++i) sample(&x,0x18,6,6);
    CHECK(!x.f.route.component_recovery_active);
    NEAR(x.f.route.origin_cm,origin);
    CHECK(x.f.route.index==1 && x.f.route.junctions_seen==1);
}

static void test_release_and_bounds(void)
{
    Fixture x;
    handoff(&x,1);CHECK(x.f.exit_heading_pending);
    x.f.odometry.distance_cm=x.f.exit_heading_origin_cm+GRAY_FORWARD_OFFSET_CM+.1f;
    sample(&x,0x0f,6,6);CHECK(!x.f.exit_heading_pending);

    handoff(&x,1);CHECK(x.f.exit_heading_pending);
    x.heading=0;x.f.odometry.heading_rad=0;x.f.turn_yaw_rate_rad_s=0;
    for(unsigned i=0;i<3;++i) sample(&x,0x18,6,6);
    CHECK(!x.f.exit_heading_pending);

    handoff(&x,1);CHECK(x.f.exit_heading_pending);
    x.f.turn_yaw_rate_rad_s=0;x.f.odometry.heading_rad=x.heading;
    for(unsigned i=0;i<3;++i) sample(&x,0x18,6,6);
    CHECK(!x.f.exit_heading_pending); /* established line can differ from planned yaw */

    handoff(&x,1);x.heading=.8f;x.f.odometry.heading_rad=.8f;x.f.turn_yaw_rate_rad_s=0;
    sample(&x,0x0f,6,6);
    CHECK(x.f.route.state==ROUTE_CANDIDATE);
    NEAR(x.f.left_target_cm_s,x.f.right_target_cm_s); /* >0.6 rad: retain local heading */

    handoff(&x,1);x.f.route.state=ROUTE_BRAKING;
    x.f.route.passing_junction=false;x.f.route.entry_heading=x.heading;
    x.f.route.phase_ms=x.now;x.f.route.entry_cm=x.f.odometry.distance_cm;
    sample(&x,0xff,6,6);
    CHECK(x.f.route.state==ROUTE_EXECUTING && !x.f.exit_heading_pending);
}
int main(void)
{
    test_adjacent_pass(1);test_adjacent_pass(-1);test_release_and_bounds();
    printf("PASS: mirrored early-exit passage heading, center-only graphic recovery, reference release and bounds (%u checks)\n",checks);
    return 0;
}
