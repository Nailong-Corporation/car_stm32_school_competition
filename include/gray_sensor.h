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

void gray_sensor_init(GraySensor *sensor);
GrayObservation gray_classify(uint8_t line_mask, bool have_previous, float previous);
GrayObservation gray_sensor_update(GraySensor *sensor, uint8_t raw_mask,
                                   bool have_previous, float previous);

#endif
