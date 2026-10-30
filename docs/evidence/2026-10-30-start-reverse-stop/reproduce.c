#include "line_follow.h"
#include "motion_config.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    SpeedControl speed;
    MotorCommand command;
    speed_control_init(&speed);
    assert(!speed_control_update(&speed, 0.0f, 2.60788321f,
                                0.648663521f, -0.593795598f, .005f, &command));
    assert(speed.fault == SPEED_FAULT_REVERSE && speed.fault_wheel == 2);
    assert(!command.enabled && command.left == 0 && command.right == 0);

    OdometryCalibration calibration = {
        LEFT_WHEEL_DIAMETER_CM, RIGHT_WHEEL_DIAMETER_CM,
        LEFT_COUNTS_PER_WHEEL_REV, RIGHT_COUNTS_PER_WHEEL_REV, WHEEL_TRACK_CM,
        ENCODER_LEFT_SIGN, ENCODER_RIGHT_SIGN, ODOMETRY_CALIBRATION_CONFIRMED
    };
    LineFollow follow;
    line_follow_init(&follow, calibration, route, route_count, NULL, NULL);
    line_follow_stop(&follow, LINE_STOP_ENCODER);
    for (unsigned i = 1; i <= LINE_CONFIRM_SAMPLES; ++i)
        line_follow_update_heading(&follow, gray_classify(0x18, true, 0),
                                   (EncoderCounts){0}, i * 5U, .005f,
                                   i == LINE_CONFIRM_SAMPLES, true, 0.0f);
    assert(follow.start_result == LINE_START_REJECTED_FAULT);
    assert(follow.state == LINE_STOPPED && !follow.command.enabled);
    assert(follow.stop_reason == LINE_STOP_ENCODER);
    puts("PASS: captured right reverse feedback trips protection; explicit START cannot clear encoder lock");
    return 0;
}