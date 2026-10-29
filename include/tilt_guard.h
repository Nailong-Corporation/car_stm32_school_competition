#ifndef TILT_GUARD_H
#define TILT_GUARD_H

#include <stdbool.h>

typedef struct {
    bool primed, tripped;
    float x_g, y_g, z_g;
    float pitch_deg; /* Sensor X axis pitch, magnitude, 0..90 degrees. */
    float above_threshold_s;
} TiltGuard;

/* Returns false for invalid dt or nonfinite data. Does not clear a latched trip. */
bool tilt_guard_update(TiltGuard *guard, float ax_g, float ay_g, float az_g, float dt);

#endif
