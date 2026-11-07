#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include <stdbool.h>
#include <stdint.h>
#include "control_config.h"

typedef enum { GRAY_VALID, GRAY_NONE, GRAY_AMBIGUOUS } GrayKind;
typedef struct {
    GrayKind kind;
    uint8_t raw_mask; /* bit i = S_i; 原始 GPIO：1 黑色、0 白色 */
    uint8_t line_mask; /* bit i = S_i; 极性转换与滤波后：1 目标线 */
    uint8_t selected_mask;
    uint8_t white_count; /* 保留字段名，实际统计目标线路数（白线或黑线） */
    uint8_t region_count;
    float position;
    uint8_t prefilter_mask; /* 极性转换/多数滤波后、地板细缝过滤前。 */
    uint8_t seam_mask; /* 本帧被地板模式剔除的独立单路区域；未开启为 0。 */
} GrayObservation;
typedef struct {
    uint8_t history[SENSOR_FILTER_SAMPLES];
    uint8_t next;
    uint8_t count;
} GraySensor;

/* 印刷元件图形特征（空心电阻、三极管 V 形、电池极板），不是路口：
 * 1) 多区域且中心两级全空、两侧外边成组（空心电阻内部）；
 * 2) 单区域对称宽线 5~7 路、左右两侧成组（电阻横边/电池极板）。
 * 真实路口：完整 8 路垂直交叉或单侧 T 形/转角分支，均不命中。 */
static inline bool gray_is_component_pattern(GrayObservation o)
{
    if (o.region_count >= 2U && (o.line_mask & 0x18U) == 0U &&
        (o.line_mask & 0x03U) != 0U && (o.line_mask & 0xc0U) != 0U) return true;
    if (o.region_count == 1U && o.white_count >= 5U && o.white_count <= 7U &&
        (o.line_mask & 0x07U) != 0U && (o.line_mask & 0xe0U) != 0U) return true;
    return false;
}

void gray_sensor_init(GraySensor *sensor);
GrayObservation gray_classify(uint8_t line_mask, bool have_previous, float previous);
GrayObservation gray_sensor_update(GraySensor *sensor, uint8_t raw_mask,
                                   bool have_previous, float previous);

#endif
