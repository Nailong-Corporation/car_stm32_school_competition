#include "tilt_guard.h"
#include "motion_config.h"
#include <math.h>

bool tilt_guard_update(TiltGuard *g, float ax, float ay, float az, float dt)
{
    if (!g || !isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
        !isfinite(dt) || dt <= 0.0f || dt > 0.015f) return false;
    if (g->tripped) return true;
    float magnitude = sqrtf(ax * ax + ay * ay + az * az);
    if (!isfinite(magnitude)) return false;
    /* Dynamic acceleration is not a reliable measure of gravity. Require a
     * near-1g interval; reject implausible frames without extending evidence. */
    if (magnitude < MPU6050_PITCH_MIN_G || magnitude > MPU6050_PITCH_MAX_G) {
        g->above_threshold_s = 0.0f;
        return true;
    }
    if (!g->primed) {
        g->x_g = ax;
        g->y_g = ay;
        g->z_g = az;
        g->primed = true;
    } else {
        float alpha = dt / (MPU6050_PITCH_FILTER_TAU_S + dt);
        g->x_g += alpha * (ax - g->x_g);
        g->y_g += alpha * (ay - g->y_g);
        g->z_g += alpha * (az - g->z_g);
    }
    /* Upright/inverted board: Z sign differs, but the pitch magnitude is
     * unchanged. X must be aligned with the vehicle's forward axis. */
    g->pitch_deg = atan2f(fabsf(g->x_g), hypotf(g->y_g, g->z_g)) * 57.2957795131f;
    if (!isfinite(g->pitch_deg)) return false;
    if (g->pitch_deg > MPU6050_PITCH_STOP_DEG) {
        g->above_threshold_s += dt;
        if (g->above_threshold_s >= MPU6050_PITCH_CONFIRM_MS / 1000.0f)
            g->tripped = true;
    } else {
        g->above_threshold_s = 0.0f;
    }
    return true;
}
