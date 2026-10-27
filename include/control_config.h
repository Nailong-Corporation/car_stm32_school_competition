#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* 原始 GPIO 白色=0、黑色=1：1 巡白线（反相），0 巡黑线（不反相）。
 * 转换后统一以 1 表示目标线；修改后须重新编译，可通过 -D 覆盖。 */
#ifndef GRAY_INVERT_ENABLED
#define GRAY_INVERT_ENABLED         0
#endif
#if GRAY_INVERT_ENABLED != 0 && GRAY_INVERT_ENABLED != 1
#error "GRAY_INVERT_ENABLED must be 0 or 1"
#endif

/* 丢线/宽线/多区域保持超限后是否自动停车，修改后重新编译。
 * 1：保留最后稳定线起 400 ms / 编码器异常后 8 cm 的停车；
 * 0：继续低速保持并等待连续有效线恢复；不关闭路径/反馈/时序/按键停车。
 * 关闭后可能持续离开赛道，调试时须具备人工停车条件。 */
#ifndef LOST_LINE_STOP_ENABLED
#define LOST_LINE_STOP_ENABLED     0
#endif
#if LOST_LINE_STOP_ENABLED != 0 && LOST_LINE_STOP_ENABLED != 1
#error "LOST_LINE_STOP_ENABLED must be 0 or 1"
#endif

/* 地板练习：细瓷砖缝常只覆盖一路，启用后在多数滤波后剔除独立单路区域。
 * 1：目标胶带需连续至少两路；0：正式比赛恢复原始 1 路边缘线响应。
 * 这是宽度启发式，不能区分覆盖两路或与胶带连通的瓷砖缝。 */
#ifndef FLOOR_SEAM_FILTER_ENABLED
#define FLOOR_SEAM_FILTER_ENABLED   0
#endif
#if FLOOR_SEAM_FILTER_ENABLED != 0 && FLOOR_SEAM_FILTER_ENABLED != 1
#error "FLOOR_SEAM_FILTER_ENABLED must be 0 or 1"
#endif

/* Initial bench values only; no track or motor calibration has been performed. */
#define CONTROL_PERIOD_MS           5U
#define CONTROL_MAX_INTERVAL_MS     15U
#define SENSOR_FILTER_SAMPLES       3U
#include "motion_config.h"
#if ENCODER_CONTROL_ENABLED
#define SENSOR_MAX_REGION_WIDTH     6U /* 连续 1～6 路有效；7～8 路按超宽歧义处理。 */
#else
#define SENSOR_MAX_REGION_WIDTH     8U /* 连续 1～7 路有效；8 路全目标缺少位置依据，保持歧义。 */
#endif
#define SENSOR_CONTINUITY_DISTANCE  3.0f
#define SENSOR_CONTINUITY_MARGIN    1.0f
#define LINE_CONFIRM_SAMPLES        3U
#define MOTOR_COMMAND_MAX          255.0f
#define MOTOR_PWM_LIMIT            255.0f /* 255 标尺的满占空比；仍保留非负限幅。 */
#define MOTOR_LEFT_GAIN            1.0f
#define MOTOR_RIGHT_GAIN           1.0f
#define MOTOR_LEFT_DEADZONE         0.0f
#define MOTOR_RIGHT_DEADZONE        0.0f
#define PID_INTEGRAL_LIMIT         20.0f
#define PID_DERIVATIVE_TAU_S        0.020f
#define BUTTON_DEBOUNCE_MS          20U

#endif
