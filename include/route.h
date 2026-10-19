#ifndef ROUTE_H
#define ROUTE_H
#include "gray_sensor.h"
#include <stddef.h>

typedef enum { LEFT, RIGHT, STRAIGHT, STOP } Action;
typedef struct {
    float distance;   /* cm from start / previous action completion */
    float tolerance;  /* inclusive +/- cm intersection window */
    Action action;
    int speed;        /* target forward speed, cm/s, NOT PWM */
} RouteStep;
typedef enum {
    ROUTE_FOLLOW, ROUTE_CANDIDATE, ROUTE_EXECUTING, ROUTE_CLEARING,
    ROUTE_FINISHED, ROUTE_FAILED, ROUTE_DISABLED
} RouteState;
typedef void (*RouteStopCallback)(size_t step_index, void *context);
typedef struct {
    const RouteStep *steps;
    size_t count, index;
    RouteState state;
    float origin_cm, entry_cm, entry_heading, clear_cm;
    uint32_t segment_ms, phase_ms;
    uint8_t candidate_count, exit_count;
    bool valid, stop_pending, callback_dispatched;
    RouteStopCallback stop_callback;
    void *callback_context;
} RoutePlanner;

extern const RouteStep route[];
extern const size_t route_count;
void route_init(RoutePlanner *planner, const RouteStep *steps, size_t count,
                 RouteStopCallback callback, void *context);
void route_start(RoutePlanner *planner, float distance_cm, uint32_t now_ms);
void route_update(RoutePlanner *planner, GrayObservation obs, float distance_cm,
                   float heading_rad, uint32_t now_ms);
bool route_is_narrow_line(GrayObservation obs);
/* Caller MUST physically issue motor_stop / disabled output before dispatch.
 * Called in the main task, never an ISR. Callback must be short/nonblocking. */
void route_dispatch_stop_callback(RoutePlanner *planner);
#endif
