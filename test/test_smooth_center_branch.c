#include "gray_sensor.h"
#include "motion_config.h"
#include "route.h"
#include <assert.h>
#include <stdio.h>

static unsigned checks;
#define CHECK(c) do { ++checks; assert(c); } while (0)

typedef struct {
    RoutePlanner planner;
    RouteStep steps[3];
    float distance;
    float heading;
    uint32_t now;
    unsigned mirror;
} Fixture;

static uint8_t mirrored(uint8_t mask, unsigned mirror)
{
    if (!mirror) return mask;
    uint8_t result = 0;
    for (unsigned bit = 0; bit < 8; ++bit)
        if (mask & (1U << bit)) result |= (uint8_t)(1U << (7U - bit));
    return result;
}

static void observe(Fixture *f, GrayObservation observation)
{
    f->distance += .06f;
    f->now += 5;
    route_update(&f->planner, observation, f->distance, f->heading, f->now);
    CHECK(f->planner.failure == ROUTE_FAILURE_NONE);
}

static void ticks(Fixture *f, uint8_t mask, unsigned count)
{
    for (unsigned i = 0; i < count; ++i)
        observe(f, gray_classify(mirrored(mask, f->mirror), true, 0));
}

/* Reach the next FOLLOW through a real turn and clearance, not by writing FSM
 * internals. The incoming centered corridor is established by the exit itself. */
static void after_turn(Fixture *f, unsigned mirror, unsigned next_count)
{
    *f = (Fixture){.mirror = mirror};
    Action action = mirror ? LEFT : RIGHT;
    f->steps[0] = (RouteStep){1, 16, action, 60, 0, false, 0, 0};
    f->steps[1] = (RouteStep){next_count, 16, action, 60, 0, false, 0, 0};
    f->steps[2] = (RouteStep){1, 16, STOP, 60, 0, false, 0, 0};
    route_init(&f->planner, f->steps, 3, NULL, NULL);
    f->planner.repeat_count = 1;
    route_start(&f->planner, 0, 0);
    CHECK(f->planner.valid);
    ticks(f, 0x18, 4);
    ticks(f, 0xff, 3);
    CHECK(f->planner.state == ROUTE_EXECUTING);
    /* 轮轴接近阶段：前伸 24 cm − 半轮距 7 cm 走完后才开始枢轴转弯。 */
    while (f->planner.turn_approaching) ticks(f, 0x18, 1);
    /* 出口须接近目标角（±90°）；半程 0.8 rad 仍属 EXECUTING。 */
    f->heading = mirror ? -1.55f : 1.55f;
    ticks(f, 0x18, 3);
    CHECK(f->planner.state == ROUTE_CLEARING);
    ticks(f, 0x18, 40);
    CHECK(f->planner.index == 1 && f->planner.state == ROUTE_FOLLOW);
    CHECK(f->planner.junctions_seen == 0 && f->planner.have_approach_center);
}

static void drift(Fixture *f)
{
    /* Actual simulator sequence: position 1 -> 2 -> 3. 0x70 contains center
     * bit S4 and two outer channels, so its shape is CENTER_BRANCH, not EDGE. */
    ticks(f, 0x38, 1);
    ticks(f, 0x30, 1);
    ticks(f, 0x70, 6);
    if (f->planner.junctions_seen != 0) {
        fprintf(stderr, "drift mirror=%u: index=%zu count=%u state=%u\n",
                f->mirror, f->planner.index, (unsigned)f->planner.junctions_seen,
                (unsigned)f->planner.state);
    }
    CHECK(f->planner.junctions_seen == 0);
    CHECK(f->planner.index == 1 && f->planner.state == ROUTE_FOLLOW);
    CHECK(f->planner.candidate_count == 0);
    CHECK(f->planner.candidate_reject == JUNCTION_REJECT_SMOOTH_EDGE);
    CHECK(f->planner.smooth_edge_sides == (f->mirror ? 1U : 2U));
}

static void test_no_extra_count_or_action(void)
{
    for (unsigned mirror = 0; mirror < 2; ++mirror) {
        for (unsigned target = 1; target <= 4; target += 3) {
            Fixture f;
            after_turn(&f, mirror, target);
            drift(&f);
            /* A brief missing observation must not relabel the same line. */
            ticks(&f, 0, 1);
            ticks(&f, 0x70, 4);
            ticks(&f, 0xc0, 4);
            CHECK(f.planner.junctions_seen == 0 && f.planner.state == ROUTE_FOLLOW);
            CHECK(f.planner.index == 1);
        }
    }
}

static void test_wider_center_branch_overrides_narrow_latch(void)
{
    /* ticks() already mirrors each input. These are respectively a centered
     * four-channel branch, the edge-biased four-channel Q4 mirror and five
     * channels. The left-mirror masks are 0x1e, 0x0f and 0x1f. */
    const uint8_t masks[] = {0x78, 0xf0, 0xf8};
    const unsigned widths[] = {4, 4, 5};
    for (unsigned mirror = 0; mirror < 2; ++mirror) {
        for (unsigned branch = 0; branch < sizeof(masks); ++branch) {
            Fixture f;
            after_turn(&f, mirror, 1);
            drift(&f);
            GrayObservation observation = gray_classify(mirrored(masks[branch], mirror), true, 0);
            CHECK(observation.white_count == widths[branch]);
            CHECK(observation.region_count == 1);
            /* The old narrow-latch blanket would hide these same-side
             * connected branches. Original strong confirmation must win. */
            ticks(&f, masks[branch], 2);
            CHECK(f.planner.junctions_seen == 1);
            CHECK(f.planner.state == ROUTE_EXECUTING);
            CHECK(f.planner.last_confirmed_kind == JUNCTION_EVIDENCE_CENTER_BRANCH);
            CHECK(f.planner.smooth_edge_rejections > 0);
        }
    }
}

static void test_center_rearms_real_branches(void)
{
    for (unsigned mirror = 0; mirror < 2; ++mirror) {
        for (unsigned edge = 0; edge < 2; ++edge) {
            Fixture f;
            after_turn(&f, mirror, 1);
            drift(&f);
            ticks(&f, 0x18, LINE_CONFIRM_SAMPLES - 1U);
            CHECK(f.planner.smooth_edge_sides != 0);
            ticks(&f, 0x18, 1);
            CHECK(f.planner.smooth_edge_sides == 0);
            /* A strong same-side L (center + branch) still counts. The weak
             * outer-only edge was retired on the 2026-11-06 track because
             * printed transistor V arms produce exactly that shape. */
            if (edge) {
                ticks(&f, 0xc0, 4);
                CHECK(f.planner.junctions_seen == 0);
            } else {
                ticks(&f, 0xf8, 2);
                CHECK(f.planner.junctions_seen == 1 && f.planner.state == ROUTE_EXECUTING);
                CHECK(f.planner.last_confirmed_kind == JUNCTION_EVIDENCE_CENTER_BRANCH);
            }
        }
    }
}

static void test_other_side_and_separated_still_count(void)
{
    for (unsigned mirror = 0; mirror < 2; ++mirror) {
        for (unsigned separated = 0; separated < 2; ++separated) {
            Fixture f;
            after_turn(&f, mirror, 1);
            drift(&f);
            /* The latch applies only to its side and to connected shapes.
             * 0xd8 has a separate same-side outer component; it must count. */
            ticks(&f, separated ? 0xd8 : 0x1f, separated ? 4 : 3);
            CHECK(f.planner.junctions_seen == 1 && f.planner.state == ROUTE_EXECUTING);
            CHECK(f.planner.last_confirmed_kind ==
                  (separated ? JUNCTION_EVIDENCE_SEPARATED : JUNCTION_EVIDENCE_CENTER_BRANCH));
        }
    }
}

static void test_raw_filter_drift(void)
{
    for (unsigned mirror = 0; mirror < 2; ++mirror) {
        Fixture f;
        GraySensor sensor;
        after_turn(&f, mirror, 1);
        gray_sensor_init(&sensor);
        const uint8_t sequence[] = {0x18, 0x38, 0x30, 0x70};
        for (unsigned group = 0; group < sizeof(sequence); ++group) {
            for (unsigned i = 0; i < 4; ++i) {
                uint8_t line = mirrored(sequence[group], mirror);
                uint8_t raw = GRAY_INVERT_ENABLED ? (uint8_t)~line : line;
                observe(&f, gray_sensor_update(&sensor, raw, true, 0));
                CHECK(f.planner.junctions_seen == 0 && f.planner.state == ROUTE_FOLLOW);
            }
        }
        CHECK(f.planner.smooth_edge_rejections > 0);
    }
}

int main(void)
{
    test_no_extra_count_or_action();
    test_wider_center_branch_overrides_narrow_latch();
    test_center_rearms_real_branches();
    test_other_side_and_separated_still_count();
    test_raw_filter_drift();
    printf("PASS: mirrored post-turn narrow drift, wider real center branches, rearming, side/separated evidence and raw polarity %u (%u checks)\n",
           (unsigned)GRAY_INVERT_ENABLED, checks);
    return 0;
}
