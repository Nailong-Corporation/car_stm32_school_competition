#include "gray_test_input.h"
#include "line_follow.h"
#include <assert.h>
#include <stdio.h>

/* 使用实际生产几何、路径表与颜色转换，验证静止在线时 START 可接受。 */
static void request_start(LineFollow *f)
{
    GraySensor sensor;
    gray_sensor_init(&sensor);
    for (unsigned i = 1; i <= 10; ++i) {
        GrayObservation obs = gray_sensor_update(&sensor, raw_for_line(0x18), false, 0);
        line_follow_update(f, obs, (EncoderCounts){0}, i * CONTROL_PERIOD_MS,
                           .005f, i == 10);
    }
}

int main(void)
{
    OdometryCalibration calibration = {
        LEFT_WHEEL_DIAMETER_CM, RIGHT_WHEEL_DIAMETER_CM,
        LEFT_COUNTS_PER_WHEEL_REV, RIGHT_COUNTS_PER_WHEEL_REV, WHEEL_TRACK_CM,
        ENCODER_LEFT_SIGN, ENCODER_RIGHT_SIGN, ODOMETRY_CALIBRATION_CONFIRMED
    };
    LineFollow f;
    line_follow_init(&f, calibration, route, route_count, NULL, NULL);
    assert(f.route.valid && f.route.failure == ROUTE_FAILURE_NONE);
    request_start(&f);
    assert(f.start_result == LINE_START_ACCEPTED);
    assert(f.state == LINE_TRACKING && f.command.enabled);
    RouteStep invalid[] = {{75, 10, STOP, (int)WHEEL_TARGET_MAX_CM_S + 1, 0.0f, true}};
    line_follow_init(&f, calibration, invalid, 1, NULL, NULL);
    assert(!f.route.valid && f.route.failure == ROUTE_FAILURE_INVALID);
    request_start(&f);
    assert(f.start_result == LINE_START_REJECTED_ROUTE);
    assert(f.state == LINE_STOPPED && !f.command.enabled);
    puts("PASS: production route/raw grayscale START accepted; excessive speed rejected");
    return 0;
}
