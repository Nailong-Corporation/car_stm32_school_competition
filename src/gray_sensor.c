#include "gray_sensor.h"
#include <math.h>
#include <string.h>

static const float weights[8] = {-7.0f, -5.0f, -3.0f, -1.0f, 1.0f, 3.0f, 5.0f, 7.0f};

/**
 * @brief 清空灰度传感器的滤波历史、写入位置和有效样本数量。
 */
void gray_sensor_init(GraySensor *sensor)
{
    memset(sensor, 0, sizeof(*sensor));
}

/**
 * @brief 按白线掩码统计连续区域，并计算选中区域的加权横向位置。
 * line_mask 的 1 表示白线；无白线或歧义观测不提供普通 PID 位置。
 * 多区域可按 previous 的连续性选区，但上层仍会将多区域按干扰处理。
 */
GrayObservation gray_classify(uint8_t line_mask, bool have_previous, float previous)
{
    GrayObservation result = {0};
    uint8_t masks[4] = {0}, widths[4] = {0};
    float positions[4] = {0};
    bool in_region = false, wide = false;
    result.line_mask = line_mask;
    result.kind = GRAY_NONE;
    for (unsigned i = 0; i < 8; ++i) {
        if ((line_mask & (1U << i)) == 0) {
            in_region = false;
            continue;
        }
        if (!in_region) {
            ++result.region_count;
            in_region = true;
        }
        unsigned region = result.region_count - 1U;
        masks[region] |= (uint8_t)(1U << i);
        ++widths[region];
        ++result.white_count;
        positions[region] += weights[i];
    }
    if (result.white_count == 0) return result;
    for (unsigned i = 0; i < result.region_count; ++i) {
        positions[i] /= (float)widths[i];
        if (widths[i] > SENSOR_MAX_REGION_WIDTH) wide = true;
    }
    result.kind = GRAY_AMBIGUOUS;
    if (wide) return result;
    unsigned selected = 0;
    if (result.region_count > 1) {
        if (!have_previous || !isfinite(previous)) return result;
        float best = 100.0f, second = 100.0f;
        for (unsigned i = 0; i < result.region_count; ++i) {
            float distance = fabsf(positions[i] - previous);
            if (distance < best) {
                second = best;
                best = distance;
                selected = i;
            } else if (distance < second) second = distance;
        }
        if (best > SENSOR_CONTINUITY_DISTANCE ||
            second - best < SENSOR_CONTINUITY_MARGIN) return result;
    }
    result.kind = GRAY_VALID;
    result.selected_mask = masks[selected];
    result.position = positions[selected];
    return result;
}

/**
 * @brief 对原始 GPIO 掩码执行一次反相、逐路多数滤波，再分类白线观测。
 * raw_mask 中白色为 0；have_previous/previous 用于多区域连续性选择。
 */
GrayObservation gray_sensor_update(GraySensor *sensor, uint8_t raw_mask,
                                   bool have_previous, float previous)
{
    /* Only this layer inverts. All later processing uses 1 = white. */
    sensor->history[sensor->next] = (uint8_t)~raw_mask;
    sensor->next = (uint8_t)((sensor->next + 1U) % SENSOR_FILTER_SAMPLES);
    if (sensor->count < SENSOR_FILTER_SAMPLES) ++sensor->count;
    uint8_t filtered = 0;
    for (unsigned bit = 0; bit < 8; ++bit) {
        unsigned votes = 0;
        for (unsigned j = 0; j < sensor->count; ++j)
            if (sensor->history[j] & (1U << bit)) ++votes;
        if (votes > sensor->count / 2U) filtered |= (uint8_t)(1U << bit);
    }
    GrayObservation result = gray_classify(filtered, have_previous, previous);
    result.raw_mask = raw_mask;
    return result;
}
