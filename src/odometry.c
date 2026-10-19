#include "odometry.h"
#include "control_config.h"
#include "motion_config.h"
#include <math.h>
#include <string.h>

static bool positive(float v) { return isfinite(v) && v > 0.0f; }

void odometry_init(Odometry *o, OdometryCalibration c)
{
    memset(o, 0, sizeof(*o));
    o->calibration = c;
    o->ready = c.confirmed && positive(c.left_diameter_cm) && positive(c.right_diameter_cm) &&
        positive(c.left_counts_per_rev) && positive(c.right_counts_per_rev) && positive(c.track_cm) &&
        (c.left_sign == 1 || c.left_sign == -1) && (c.right_sign == 1 || c.right_sign == -1);
    if (!o->ready) return;
    o->left_cm_per_count = 3.14159265359f * c.left_diameter_cm / c.left_counts_per_rev;
    o->right_cm_per_count = 3.14159265359f * c.right_diameter_cm / c.right_counts_per_rev;
    float max_dt = (float)CONTROL_MAX_INTERVAL_MS / 1000.0f;
    /* At most half a counter revolution between samples, including quantization margin. */
    o->ready = positive(o->left_cm_per_count) && positive(o->right_cm_per_count) &&
        ODOMETRY_MAX_SPEED_CM_S * max_dt / o->left_cm_per_count + 4.0f < 32768.0f &&
        ODOMETRY_MAX_SPEED_CM_S * max_dt / o->right_cm_per_count + 4.0f < 32768.0f;
}

static int32_t delta(uint16_t now, uint16_t before)
{
    uint32_t d = (uint16_t)(now - before);
    return d < 32768U ? (int32_t)d : (int32_t)d - 65536;
}

bool odometry_update(Odometry *o, EncoderCounts counts, float dt)
{
    if (!o->ready || !isfinite(dt) || dt <= 0.0f ||
        dt > (float)CONTROL_MAX_INTERVAL_MS / 1000.0f) return false;
    if (!o->primed) {
        o->previous = counts;
        o->primed = true;
        return true;
    }
    int32_t dl = delta(counts.left, o->previous.left);
    int32_t dr = delta(counts.right, o->previous.right);
    if (dl == -32768 || dr == -32768) return false;
    dl *= o->calibration.left_sign;
    dr *= o->calibration.right_sign;
    float l = (float)dl * o->left_cm_per_count;
    float r = (float)dr * o->right_cm_per_count;
    if (fabsf(l) > ODOMETRY_MAX_SPEED_CM_S * dt + 4.0f * o->left_cm_per_count ||
        fabsf(r) > ODOMETRY_MAX_SPEED_CM_S * dt + 4.0f * o->right_cm_per_count) return false;
    o->previous = counts;
    o->left_delta = dl;
    o->right_delta = dr;
    o->distance_cm += (l + r) * 0.5f;
    /* Clockwise/right is positive, matching positive line error and wheel mixing. */
    o->heading_rad += (l - r) / o->calibration.track_cm;
    float alpha = dt / (SPEED_FILTER_TAU_S + dt);
    o->left_speed_cm_s += alpha * (l / dt - o->left_speed_cm_s);
    o->right_speed_cm_s += alpha * (r / dt - o->right_speed_cm_s);
    return isfinite(o->distance_cm) && isfinite(o->heading_rad) &&
        isfinite(o->left_speed_cm_s) && isfinite(o->right_speed_cm_s);
}
