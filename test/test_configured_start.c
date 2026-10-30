#include "gray_test_input.h"
#include "line_follow.h"
#include <assert.h>
#include <stdio.h>

/* 使用实际生产几何、路径表与颜色转换，验证静止在线时 START 可接受。 */
static void request_start(LineFollow *f, uint8_t line_mask)
{
    GraySensor sensor;
    gray_sensor_init(&sensor);
    for (unsigned i = 1; i <= 10; ++i) {
        GrayObservation obs = gray_sensor_update(&sensor, raw_for_line(line_mask), false, 0);
        line_follow_update_heading(f, obs, (EncoderCounts){0}, i * CONTROL_PERIOD_MS,
                                   .005f, i == 10, true, 0.0f);
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
    request_start(&f, 0x18);
    assert(f.start_result == LINE_START_ACCEPTED);
    assert(f.state == LINE_TRACKING && f.command.enabled);
    /* 板上复现：START 已接受，左计数 0→1；生产符号必须使反馈为正。 */
    line_follow_update_heading(&f, gray_classify(0x10, true, 0),
                               (EncoderCounts){1, 0}, 11 * CONTROL_PERIOD_MS,
                               .005f, false, true, 0.0f);
    assert(f.odometry.left_speed_cm_s > 0.0f);
    assert(f.state == LINE_TRACKING && f.command.enabled);
    /* 最新板上快照：左正计数、右 0→65535；右侧应按测距工程符号产生正反馈。
     * 不关闭反向保护，继续使用真实生产标定与灰度启动门禁。 */
    line_follow_update_heading(&f, gray_classify(0x10, true, 0),
                               (EncoderCounts){16, 65535}, 12 * CONTROL_PERIOD_MS,
                               .005f, false, true, 0.0f);
    assert(f.odometry.left_speed_cm_s > 0.0f);
    assert(f.odometry.right_speed_cm_s > 0.0f);
    assert(f.speed.fault == SPEED_FAULT_NONE);
    assert(f.state == LINE_TRACKING && f.command.enabled);
    line_follow_init(&f, calibration, route, route_count, NULL, NULL);
    request_start(&f, 0);
    assert(f.start_result == LINE_START_REJECTED_LINE && !f.command.enabled);
#if MPU6050_ENABLED
    line_follow_init(&f, calibration, route, route_count, NULL, NULL);
    line_follow_update_heading(&f, gray_classify(0x18, true, 0), (EncoderCounts){0},
                               CONTROL_PERIOD_MS, .005f, true, false, 0.0f);
    assert(f.start_result == LINE_START_REJECTED_FAULT);
    assert(f.stop_reason == LINE_STOP_IMU && !f.command.enabled);
#endif
    RouteStep invalid[] = {{75, 10, STOP, (int)WHEEL_TARGET_MAX_CM_S + 1, 0.0f, true}};
    line_follow_init(&f, calibration, invalid, 1, NULL, NULL);
    assert(!f.route.valid && f.route.failure == ROUTE_FAILURE_INVALID);
    request_start(&f, 0x18);
    assert(f.start_result == LINE_START_REJECTED_ROUTE);
    assert(f.state == LINE_STOPPED && !f.command.enabled);
    puts("PASS: production route/raw grayscale START accepted; no-line, excessive speed and enabled IMU fault gates retained");
    return 0;
}
