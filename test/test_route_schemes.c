/* Validate the four generated route tables through the real route_table.c
 * source, not a hand-copied fixture. Each scheme is compiled under its own
 * ROUTE_SCHEME_SELECT and renamed, so no stale duplicate array can drift. */
#include "route.h"
#include "motion_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define ROUTE_SCHEME_SELECT 0
#define route route_scheme_0
#define route_count route_count_0
#include "../src/route_table.c"
#undef route
#undef route_count
#undef ROUTE_SCHEME_SELECT

#define ROUTE_SCHEME_SELECT 1
#define route route_scheme_1
#define route_count route_count_1
#include "../src/route_table.c"
#undef route
#undef route_count
#undef ROUTE_SCHEME_SELECT

#define ROUTE_SCHEME_SELECT 2
#define route route_scheme_2
#define route_count route_count_2
#include "../src/route_table.c"
#undef route
#undef route_count
#undef ROUTE_SCHEME_SELECT

#define ROUTE_SCHEME_SELECT 4
#define route route_scheme_4
#define route_count route_count_4
#include "../src/route_table.c"
#undef route
#undef route_count
#undef ROUTE_SCHEME_SELECT

#define ROUTE_SCHEME_SELECT 5
#define route route_scheme_5
#define route_count route_count_5
#include "../src/route_table.c"
#undef route
#undef route_count
#undef ROUTE_SCHEME_SELECT

#define ROUTE_SCHEME_SELECT 6
#define route route_scheme_6
#define route_count route_count_6
#include "../src/route_table.c"
#undef route
#undef route_count
#undef ROUTE_SCHEME_SELECT

#define ROUTE_SCHEME_SELECT 7
#define route route_scheme_7
#define route_count route_count_7
#include "../src/route_table.c"
#undef route
#undef route_count
#undef ROUTE_SCHEME_SELECT

#define ROUTE_SCHEME_SELECT 3
#define route route_scheme_3
#define route_count route_count_3
#include "../src/route_table.c"
#undef route
#undef route_count
#undef ROUTE_SCHEME_SELECT

static unsigned checks;

#define CHECK(c) do { ++checks; assert(c); } while (0)

static bool ordinary_right_angle(const RouteStep *s)
{
    return !s->corner_arc_enabled && (s->action == LEFT || s->action == RIGHT ||
        (s->action == CUSTOM_TURN && fabsf(fabsf(s->angle_deg) - 90.0f) < 0.001f));
}

static void verify_scheme(const char *name, const RouteStep *steps, size_t count,
                          size_t expected_steps, float expected_len,
                          bool expect_short_links)
{
    RoutePlanner p;
    route_init(&p, steps, count, NULL, NULL);
    CHECK(p.valid);
    CHECK(p.state == ROUTE_FOLLOW);
    CHECK(p.failure == ROUTE_FAILURE_NONE);
    CHECK(count == expected_steps);
    CHECK(steps[count - 1].action == STOP);

    size_t short_links = 0;
    float total_len = 0.0f;
    for (size_t i = 0; i < count; ++i) {
        const RouteStep *s = &steps[i];
        CHECK(s->junction_count > 0);
        CHECK(s->cruise_speed > 0);
        CHECK(s->cruise_speed <= (int)WHEEL_TARGET_MAX_CM_S);
        CHECK(isfinite(s->turn_speed) && s->turn_speed > 0.0f);
        CHECK(s->turn_speed <= (float)s->cruise_speed);
        CHECK(isfinite(s->segment_length_cm) && s->segment_length_cm > 0.0f);
        CHECK(isfinite(s->short_link_cm) && s->short_link_cm >= 0.0f);
        if (s->short_link_cm > 0.0f) {
            /* Optional metadata: full corner-to-corner link geometry only on a
             * right angle following another right angle, else route_init fails. */
            CHECK(i > 0);
            CHECK(s->short_link_cm >= 20.0f && s->short_link_cm <= 40.0f);
            CHECK(ordinary_right_angle(s));
            CHECK(ordinary_right_angle(&steps[i - 1]));
            /* Generated table rows subtract the 5 cm distance margin once. */
            CHECK(fabsf(s->short_link_cm - (s->segment_length_cm + 5.0f)) < 0.1f);
            ++short_links;
        }
        CHECK(s->action != STOP || i == count - 1);
        if (s->action == HEXAGON) {
            CHECK(fabsf(s->angle_deg) >= 60.0f);
            CHECK(!s->corner_arc_enabled);
        }
        total_len += s->segment_length_cm;
    }
    CHECK(fabsf(total_len - expected_len) < 0.1f);
    if (expect_short_links) CHECK(short_links > 0);
    else CHECK(short_links == 0);
    printf("PASS: %s (steps=%zu, total_len=%.1f cm, short_links=%zu)\n",
           name, count, total_len, short_links);
}

int main(void)
{
    /* Scheme 0: four-step debug route, table cruise/turn values retained. */
    verify_scheme("Scheme 0 (debug)", route_scheme_0, route_count_0, 4, 480.0f, false);
    for (size_t i = 0; i < route_count_0; ++i) {
        CHECK(route_scheme_0[i].cruise_speed == 50 || route_scheme_0[i].cruise_speed == 200);
        CHECK(route_scheme_0[i].turn_speed == 50.0f);
    }
    /* Official schemes use the authorized cruise 60 / turn 16 pair. */
    /* 2026-11-07: the ring now exits south and the tail continues along the
     * bottom; the 5 cm margin is already subtracted in the table. */
    verify_scheme("Scheme 1 (dashed reference route)", route_scheme_1, route_count_1,
                  30, 2680.0f, true);
    verify_scheme("Scheme 2 (no dashed reference route)", route_scheme_2, route_count_2,
                  28, 2600.0f, true);
    /* Generated tables share one cruise/turn pair; compare against row 0 so the
     * sweep value does not need a code change (see test/new_track_routes.json). */
    for (size_t i = 1; i < route_count_1; ++i) {
        CHECK(route_scheme_1[i].cruise_speed == route_scheme_1[0].cruise_speed);
        CHECK(route_scheme_1[i].turn_speed == route_scheme_1[0].turn_speed);
    }
    CHECK(route_scheme_1[0].turn_speed == 28.0f);
    for (size_t i = 1; i < route_count_2; ++i) {
        CHECK(route_scheme_2[i].cruise_speed == route_scheme_2[0].cruise_speed);
        CHECK(route_scheme_2[i].turn_speed == route_scheme_2[0].turn_speed);
    }
    CHECK(route_scheme_2[0].turn_speed == 28.0f);
    verify_scheme("Scheme 3 (outer test loop)", route_scheme_3, route_count_3,
                  5, 1165.0f, false);
    for (size_t i = 0; i < route_count_3; ++i) {
        CHECK(route_scheme_3[i].cruise_speed == route_scheme_3[0].cruise_speed);
        CHECK(route_scheme_3[i].turn_speed == route_scheme_3[0].turn_speed);
    }
    /* 2026-11-09: four more tables - dashed without the hexagon ring (normal),
     * and the fast twins of schemes 1/2/3. */
    verify_scheme("Scheme 4 (dashed, no hexagon ring)", route_scheme_4, route_count_4,
                  30, 2680.0f, true);
    for (size_t i = 0; i < route_count_4; ++i) {
        CHECK(route_scheme_4[i].cruise_speed == route_scheme_4[0].cruise_speed);
        CHECK(route_scheme_4[i].turn_speed == route_scheme_4[0].turn_speed);
    }
    verify_scheme("Scheme 5 (fast dashed with ring)", route_scheme_5, route_count_5,
                  30, 2680.0f, true);
    verify_scheme("Scheme 6 (fast no-dashed)", route_scheme_6, route_count_6,
                  28, 2600.0f, true);
    verify_scheme("Scheme 7 (fast outer test loop)", route_scheme_7, route_count_7,
                  5, 1165.0f, false);
    for (size_t n = 5; n <= 7; ++n) {
        const RouteStep *steps = n == 5 ? route_scheme_5 : n == 6 ? route_scheme_6 : route_scheme_7;
        size_t count = n == 5 ? route_count_5 : n == 6 ? route_count_6 : route_count_7;
        for (size_t i = 0; i < count; ++i) {
            CHECK(steps[i].cruise_speed == steps[0].cruise_speed);
            CHECK(steps[i].turn_speed == steps[0].turn_speed);
        }
        CHECK(steps[0].cruise_speed >= route_scheme_1[0].cruise_speed); /* fast twins are faster */
    }
    printf("PASS: all eight generated route tables valid (%u checks)\n", checks);
    return 0;
}
