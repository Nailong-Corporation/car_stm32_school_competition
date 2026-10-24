#include "route.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    RoutePlanner p;
    route_init(&p,route,route_count,0,0);
    assert(p.valid);
    route_start(&p,0,0);
    p.index=1; p.state=ROUTE_CLEARING;
    p.entry_heading=1.85737765f;
    p.clear_cm=174.682388f;
    p.outgoing_cm=19.9506721f;
    p.projection_previous_cm=177.766159f;
    p.projection_previous_heading=.0397814997f;
    p.phase_ms=9325; p.exit_count=2;
    GrayObservation obs=gray_classify(0x18,1,0);
    route_update(&p,obs,177.766159f,.0397814997f,9565);
    printf("segment start index=%zu state=%d origin=%.6f progress=%.6f upper=%.6f\n",p.index,p.state,p.origin_cm,177.766159f-p.origin_cm,p.steps[p.index].distance+p.steps[p.index].tolerance);
    assert(p.index==2 && p.state==ROUTE_FOLLOW);
    route_update(&p,obs,177.862289f,.04080153f,9570);
    printf("next sample state=%d failure=%d progress=%.6f\n",p.state,p.failure,177.862289f-p.origin_cm);
    assert(p.state==ROUTE_FAILED && p.failure==ROUTE_FAILURE_WINDOW);
}
