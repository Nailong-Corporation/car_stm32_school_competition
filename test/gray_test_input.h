#ifndef GRAY_TEST_INPUT_H
#define GRAY_TEST_INPUT_H

#include "gray_sensor.h"

/* 将测试场景中的目标线图案转换为对应 GPIO 电平，供两种极性共用。 */
static inline uint8_t raw_for_line(uint8_t line)
{
#if GRAY_INVERT_ENABLED
    return (uint8_t)~line;
#else
    return line;
#endif
}

#endif
