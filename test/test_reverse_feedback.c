#include "line_follow.h"
#include "motion_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void disabled(MotorCommand command)
{
    assert(!command.enabled && command.left == 0 && command.right == 0);
}

static void test_reverse_confirmation(void)
{
    for (unsigned side = 0; side < 2; ++side) {
        for (unsigned zero_target = 0; zero_target < 2; ++zero_target) {
            SpeedControl s;
            MotorCommand out;
            float lt = side == 0 && zero_target ? 0 : 10;
            float rt = side == 1 && zero_target ? 0 : 10;
            speed_control_init(&s);
            /* Both captured small reverse samples must inhibit, not lock. */
            assert(speed_control_update(&s, lt, rt, side == 0 ? -.593797f : 0,
                                        side == 1 ? -.593797f : 0, .005f, &out));
            disabled(out);
            assert(s.fault == SPEED_FAULT_NONE && s.reverse_pending);
            assert(!s.left.primed && !s.right.primed);
            /* Transient disappears; no stale PID kick or accumulating evidence. */
            assert(speed_control_update(&s, lt, rt, 0, 0, .005f, &out));
            assert(out.enabled && !s.reverse_pending);
            for (unsigned i = 0; i < 20; ++i) {
                assert(speed_control_update(&s, lt, rt, side == 0 ? -.594f : 0,
                                            side == 1 ? -.594f : 0, .005f, &out));
                disabled(out);
                assert(speed_control_update(&s, lt, rt, 0, 0, .005f, &out));
            }
            speed_control_reset(&s);
            for (unsigned i = 1; i <= 10; ++i) {
                bool ok = speed_control_update(&s, lt, rt, side == 0 ? -.6f : 0,
                                               side == 1 ? -.6f : 0, .005f, &out);
                disabled(out);
                assert(ok == (i < 10));
            }
            assert(s.fault == SPEED_FAULT_REVERSE && s.fault_wheel == side + 1);
            /* Actual dt, not frame count: 5+15+15+15 = 50 ms. */
            speed_control_reset(&s);
            for (unsigned i = 0; i < 4; ++i) {
                bool ok = speed_control_update(&s, lt, rt, side == 0 ? -.6f : 0,
                                               side == 1 ? -.6f : 0,
                                               i == 0 ? .005f : .015f, &out);
                assert(ok == (i < 3));
                disabled(out);
            }
            /* Distance gate precedes time gate: 4 cm/s * 25 ms = 0.1 cm. */
            speed_control_reset(&s);
            for (unsigned i = 1; i <= 5; ++i) {
                bool ok = speed_control_update(&s, lt, rt, side == 0 ? -4 : 0,
                                               side == 1 ? -4 : 0, .005f, &out);
                assert(ok == (i < 5));
                disabled(out);
            }
            speed_control_reset(&s);
            assert(!speed_control_update(&s, lt, rt, side == 0 ? -5 : 0,
                                         side == 1 ? -5 : 0, .005f, &out));
            assert(s.fault == SPEED_FAULT_REVERSE && s.fault_wheel == side + 1);
            disabled(out);
        }
    }
    SpeedControl s;
    MotorCommand out;
    speed_control_init(&s);
    /* A mild left sample must not mask a severe right fault or invalid input. */
    assert(!speed_control_update(&s, 10, 10, -.6f, -5, .005f, &out));
    assert(s.fault == SPEED_FAULT_REVERSE && s.fault_wheel == 2);
    speed_control_reset(&s);
    assert(!speed_control_update(&s, 10, NAN, -.6f, 0, .005f, &out));
    assert(s.fault == SPEED_FAULT_INPUT && s.fault_wheel == 2);
    disabled(out);
    assert(!speed_control_update(&s, 10, 10, 0, 0, .020f, &out));
    assert(s.fault == SPEED_FAULT_INTERVAL);
    disabled(out);
    speed_control_reset(&s);
    for (unsigned i = 0; i < 100; ++i) {
        if (!speed_control_update(&s, 10, 10, 0, 0, .005f, &out)) break;
    }
    assert(s.fault == SPEED_FAULT_STALL && s.fault_wheel == 1);
    disabled(out);
    speed_control_reset(&s);
    /* Evidence is independent: switch sides without transferring time. */
    for (unsigned i = 0; i < 6; ++i)
        assert(speed_control_update(&s, 10, 10, -.6f, 0, .005f, &out));
    assert(speed_control_update(&s, 10, 10, 0, -.6f, .005f, &out));
    assert(s.left_reverse_s == 0 && fabsf(s.right_reverse_s - .005f) < .00001f);
    speed_control_reset(&s);
    assert(!s.reverse_pending && s.left_reverse_s == 0 && s.right_reverse_s == 0);
    assert(s.left_reverse_cm == 0 && s.right_reverse_cm == 0);
}

static void test_one_count_start(void)
{
    OdometryCalibration c = {
        LEFT_WHEEL_DIAMETER_CM, RIGHT_WHEEL_DIAMETER_CM,
        LEFT_COUNTS_PER_WHEEL_REV, RIGHT_COUNTS_PER_WHEEL_REV, WHEEL_TRACK_CM,
        ENCODER_LEFT_SIGN, ENCODER_RIGHT_SIGN, ODOMETRY_CALIBRATION_CONFIRMED
    };
    LineFollow f;
    line_follow_init(&f, c, route, route_count, NULL, NULL);
    EncoderCounts counts = {0};
    GrayObservation obs = gray_classify(0x10, true, 0);
    uint32_t now = 0;
    for (unsigned i = 0; i < LINE_CONFIRM_SAMPLES; ++i) {
        now += 5;
        line_follow_update_heading(&f, obs, counts, now, .005f,
                                   i == LINE_CONFIRM_SAMPLES - 1, true, 0);
    }
    assert(f.start_result == LINE_START_ACCEPTED);
    /* Right sign -1: one positive raw count represents reverse wheel motion. */
    counts.right = 1;
    now += 5;
    line_follow_update_heading(&f, obs, counts, now, .005f, false, true, 0);
    assert(fabsf(f.odometry.right_speed_cm_s + .5937956f) < .0001f);
    assert(f.state == LINE_TRACKING && f.stop_reason == LINE_STOP_NONE);
    disabled(f.command);
    now += 5;
    line_follow_update_heading(&f, obs, counts, now, .005f, false, true, 0);
    /* One-count impulse decays .594 -> .509 -> .436 with the 30 ms filter. */
    disabled(f.command);
    now += 5;
    line_follow_update_heading(&f, obs, counts, now, .005f, false, true, 0);
    assert(f.command.enabled && !f.stop_snapshot.valid);
    /* Repeated actual reverse counts must still lock, even at zero target. */
    for (unsigned i = 0; i < 100 && f.state != LINE_STOPPED; ++i) {
        ++counts.right;
        now += 5;
        line_follow_update_heading(&f, obs, counts, now, .005f, false, true, 0);
    }
    assert(f.state == LINE_STOPPED && f.stop_reason == LINE_STOP_ENCODER);
    assert(f.stop_snapshot.speed_fault == SPEED_FAULT_REVERSE);
    assert(f.stop_snapshot.speed_fault_wheel == 2);
    disabled(f.command);
    now += 5;
    line_follow_update_heading(&f, obs, counts, now, .005f, true, true, 0);
    assert(f.start_result == LINE_START_REJECTED_FAULT);
    assert(f.state == LINE_STOPPED);
}

int main(void)
{
    test_reverse_confirmation();
    test_one_count_start();
    puts("PASS: single-count START, transient inhibit, mirrored reverse time/distance/severe gates and lock");
    return 0;
}