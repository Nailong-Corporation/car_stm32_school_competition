#ifndef ENCODER_H
#define ENCODER_H

#include "odometry.h"
/* TIM1 PA8/9 and TIM4 PB6/7, TI12 x4, no interrupts. */
void encoder_gpio_init(void);
bool encoder_init(void);
bool encoder_read(EncoderCounts *counts);

#endif
