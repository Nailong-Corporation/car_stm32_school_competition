#include "braking_planner.h"
#include <math.h>
BrakingPlan braking_plan(float cruise,float entry,float length,float progress,
                         float speed,float decel,float margin,bool calibrated)
{
    BrakingPlan p={0};
    if(!isfinite(cruise) || !isfinite(entry) || !isfinite(length) || !isfinite(progress) ||
       !isfinite(speed) || !isfinite(decel) || !isfinite(margin) || cruise<=0 ||
       entry<0 || entry>cruise || length<0 || margin<0) return p;
    p.allowed_speed=cruise;
    if(length==0) {p.valid=true;return p;}
    if(!calibrated || decel<=0) return p;
    p.remaining_cm=fmaxf(0,length-fmaxf(0,progress)-margin);
    p.allowed_speed=fminf(cruise,sqrtf(entry*entry+2*decel*p.remaining_cm));
    float excess=fmaxf(0,speed*speed-entry*entry);
    p.required_decel=p.remaining_cm>0?excess/(2*p.remaining_cm):(excess>0?INFINITY:0);
    p.valid=isfinite(p.allowed_speed);
    return p;
}

float braking_slew(BrakingSlew *s,float cap,float dt,float acceleration,float jerk)
{
    if(!s || !isfinite(cap) || cap<0 || !isfinite(dt) || dt<=0 ||
       !isfinite(acceleration) || acceleration<=0 || !isfinite(jerk) || jerk<=0) return 0;
    if(cap<=s->speed) {s->speed=cap;s->acceleration=0;return cap;}
    s->acceleration=fminf(acceleration,s->acceleration+jerk*dt);
    s->speed=fminf(cap,s->speed+s->acceleration*dt);
    return s->speed;
}
