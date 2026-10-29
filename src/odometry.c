#include "odometry.h"
#include "control_config.h"
#include "motion_config.h"
#include <math.h>
#include <string.h>

/**
 * @brief 判断标定量是否为有限正数，拒绝零值、负值和非有限数。
 */
static bool positive(float v) { return isfinite(v) && v > 0.0f; }

/**
 * @brief 校验尺寸、每输出轮四倍频计数、方向和标定确认，建立厘米换算比例。
 * 同时检查最长采样间隔的半圈计数约束；未通过时 ready 为 false，禁止编码器模式启动。
 */
void odometry_init(Odometry *o, OdometryCalibration c)
{
    memset(o, 0, sizeof(*o));
    o->calibration = c;
    o->sensor_x_cm = GRAY_FORWARD_OFFSET_CM;
    o->ready = c.confirmed && positive(ODOMETRY_DISTANCE_SCALE) && positive(ODOMETRY_HEADING_SCALE) &&
        positive(c.left_diameter_cm) && positive(c.right_diameter_cm) &&
        positive(c.left_counts_per_rev) && positive(c.right_counts_per_rev) && positive(c.track_cm) &&
        (c.left_sign == 1 || c.left_sign == -1) && (c.right_sign == 1 || c.right_sign == -1);
    if (!o->ready) return;
    /* 轮程与轮速共享距离换算；轮差转航向另乘独立转角比例。 */
    o->left_cm_per_count = 3.14159265359f * c.left_diameter_cm / c.left_counts_per_rev *
        ODOMETRY_DISTANCE_SCALE;
    o->right_cm_per_count = 3.14159265359f * c.right_diameter_cm / c.right_counts_per_rev *
        ODOMETRY_DISTANCE_SCALE;
    float max_dt = (float)CONTROL_MAX_INTERVAL_MS / 1000.0f;
    /* At most half a counter revolution between samples, including quantization margin. */
    o->ready = positive(o->left_cm_per_count) && positive(o->right_cm_per_count) &&
        ODOMETRY_MAX_SPEED_CM_S * max_dt / o->left_cm_per_count + 4.0f < 32768.0f &&
        ODOMETRY_MAX_SPEED_CM_S * max_dt / o->right_cm_per_count + 4.0f < 32768.0f;
}

/**
 * @brief 将两个 16 位计数快照的模差转换为有符号增量，兼容正常正反向回绕。
 * 恰好半圈的差值返回 -32768，由调用者拒绝，因为此时无法判定方向。
 */
static int32_t delta(uint16_t now, uint16_t before)
{
    uint32_t d = (uint16_t)(now - before);
    return d < 32768U ? (int32_t)d : (int32_t)d - 65536;
}

/**
 * @brief 由计数增量和实际 dt 更新相对里程（cm）、航向（rad）和滤波轮速（cm/s）。
 * 首帧只建立计数基准；右转航向为正，倒退保留距离符号。
 * 标定无效、时序异常或计数跳变超限返回 false；不补偿打滑或多圈采样混叠。
 */
bool odometry_update_with_heading(Odometry *o, EncoderCounts counts, float dt,
                                 bool use_imu, float imu_heading_rad)
{
    if (!o->ready || !isfinite(dt) || dt <= 0.0f ||
        dt > (float)CONTROL_MAX_INTERVAL_MS / 1000.0f ||
        (use_imu && !isfinite(imu_heading_rad))) return false;
    if (!o->primed) {
        o->previous = counts;
        o->primed = true;
        if (use_imu) {
            o->heading_rad = imu_heading_rad;
            o->sensor_x_cm = o->x_cm + GRAY_FORWARD_OFFSET_CM * cosf(o->heading_rad);
            o->sensor_y_cm = o->y_cm + GRAY_FORWARD_OFFSET_CM * sinf(o->heading_rad);
        }
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
    float ds = (l + r) * 0.5f;
    float encoder_dh = (l - r) / o->calibration.track_cm * ODOMETRY_HEADING_SCALE;
    float dh = use_imu ? imu_heading_rad - o->heading_rad : encoder_dh;
    float half = dh * 0.5f;
    float sinc = fabsf(half) < 0.00001f ? 1.0f : sinf(half) / half;
    float mid = o->heading_rad + half;
    o->left_distance_cm += l;
    o->right_distance_cm += r;
    o->distance_cm += ds; /* 有符号轴中点弧长，保留原有诊断/保护语义。 */
    o->x_cm += ds * sinc * cosf(mid);
    o->y_cm += ds * sinc * sinf(mid);
    /* 单周期恒曲率假设下，前置探头速度模长为 hypot(v, L*omega)。 */
    o->sensor_distance_cm += hypotf(ds, GRAY_FORWARD_OFFSET_CM * dh);
    /* Clockwise/right is positive, matching positive line error and wheel mixing. */
    o->heading_rad = use_imu ? imu_heading_rad : o->heading_rad + dh;
    o->sensor_x_cm = o->x_cm + GRAY_FORWARD_OFFSET_CM * cosf(o->heading_rad);
    o->sensor_y_cm = o->y_cm + GRAY_FORWARD_OFFSET_CM * sinf(o->heading_rad);
    float alpha = dt / (SPEED_FILTER_TAU_S + dt);
    o->left_speed_cm_s += alpha * (l / dt - o->left_speed_cm_s);
    o->right_speed_cm_s += alpha * (r / dt - o->right_speed_cm_s);
    return isfinite(o->x_cm) && isfinite(o->y_cm) &&
        isfinite(o->left_distance_cm) && isfinite(o->right_distance_cm) &&
        isfinite(o->sensor_distance_cm) && isfinite(o->distance_cm) && isfinite(o->heading_rad) &&
        isfinite(o->left_speed_cm_s) && isfinite(o->right_speed_cm_s);
}

bool odometry_update(Odometry *o, EncoderCounts counts, float dt)
{
    return odometry_update_with_heading(o, counts, dt, false, 0.0f);
}
