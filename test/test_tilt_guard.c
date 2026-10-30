#include "tilt_guard.h"
#include "motion_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void pitch_frame(TiltGuard *g, float degrees, bool upside_down)
{
    const float rad = degrees * 0.0174532925199f;
    const float z_sign = upside_down ? -1.0f : 1.0f;
    assert(tilt_guard_update(g, sinf(rad), 0.0f, z_sign * cosf(rad), .005f));
}

int main(void)
{
    assert(MPU6050_PITCH_STOP_ENABLED == 1);
    assert(fabsf(MPU6050_PITCH_STOP_DEG - 40.0f) < .0001f);
    for (int inverted = 0; inverted < 2; ++inverted) {
        for (int direction = -1; direction <= 1; direction += 2) {
            TiltGuard g = {0};
            for (int i = 0; i < 100; ++i) pitch_frame(&g, direction * 39.0f, inverted);
            assert(fabsf(g.pitch_deg - 39.0f) < .01f && !g.tripped);

            /* One-frame 90° shock must not trigger the pickup latch. */
            pitch_frame(&g, direction * 90.0f, inverted);
            for (int i = 0; i < 50; ++i) pitch_frame(&g, 0.0f, inverted);
            assert(!g.tripped && g.above_threshold_s == 0.0f);

            /* Brief tilt above 40° is ignored; continuous tilt trips. */
            for (int i = 0; i < 100 && g.pitch_deg < 41.0f; ++i)
                pitch_frame(&g, direction * 55.0f, inverted);
            assert(g.pitch_deg > 40.0f);
            if (!g.tripped) {
                pitch_frame(&g, direction * 55.0f, inverted);
                assert(!g.tripped);
            }
            for (int i = 0; i < 30; ++i) pitch_frame(&g, direction * 55.0f, inverted);
            assert(g.tripped);
            assert(!tilt_guard_acknowledge(&g)); /* Still lifted cannot START. */
            for (int i = 0; i < 100; ++i) pitch_frame(&g, 0, inverted);
            assert(g.tripped && g.can_restart && g.pitch_deg < 1);
            assert(tilt_guard_acknowledge(&g) && !g.tripped);
            for (int i = 0; i < 100; ++i) pitch_frame(&g, 55, inverted);
            assert(g.tripped); /* Repeated pickup still stops. */
            assert(tilt_guard_update(&g, 1.5f, 0, 0, .005f));
            assert(!tilt_guard_acknowledge(&g)); /* Invalid gravity cannot release. */
        }
    }
    TiltGuard invalid = {0};
    assert(!tilt_guard_update(&invalid, NAN, 0, 1, .005f));
    assert(!tilt_guard_acknowledge(&invalid));
    assert(!tilt_guard_update(&invalid, 0, 0, 1, 0.0f));
    assert(!tilt_guard_update(&invalid, 0, 0, 1, .020f));
    assert(tilt_guard_update(&invalid, 0, 0, -1, .005f));
    assert(tilt_guard_update(&invalid, 1.5f, 0, 0, .005f));
    assert(invalid.above_threshold_s == 0.0f && !invalid.tripped);
    puts("PASS: inverted/normal pitch 40deg, spike filtering, latched stop, level acknowledgement and repeat pickup");
    return 0;
}
