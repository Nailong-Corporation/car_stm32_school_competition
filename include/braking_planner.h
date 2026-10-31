#ifndef BRAKING_PLANNER_H
#define BRAKING_PLANNER_H
#include <stdbool.h>
typedef struct { float allowed_speed, required_decel, remaining_cm; bool valid; } BrakingPlan;
/* Pure speed envelope. No route state, junction event, or motor output. */
BrakingPlan braking_plan(float cruise, float entry, float length, float progress,
                         float measured_speed, float decel, float margin, bool calibrated);
/* Decision uses conservative calibrated deceleration and observed braking response;
 * sensor acceleration can only reduce the estimate, never raise the safe bound. */
typedef struct { bool reverse_needed, distance_exhausted; float required_decel; } BrakingDecision;
BrakingDecision braking_decide(float speed, float entry, float remaining,
                               float observed_decel, float safe_decel, float guard);
typedef struct { float speed, acceleration; } BrakingSlew;
/* Smooth acceleration, with an immediate hard safety cap when the envelope drops. */
float braking_slew(BrakingSlew *state, float cap, float dt, float acceleration, float jerk);
#endif
