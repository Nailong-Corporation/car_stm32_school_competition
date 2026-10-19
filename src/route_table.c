#include "route.h"

/* EXAMPLE ONLY: replace distances/speeds after course and odometry calibration.
 * Each distance begins at completion of the previous junction action.
 * STOP also needs a confirmed intersection within its distance window. */
const RouteStep route[] = {
    {30.0f, 5.0f, LEFT, 12},
    {20.0f, 4.0f, STOP, 8},
};
const size_t route_count = sizeof(route) / sizeof(route[0]);
