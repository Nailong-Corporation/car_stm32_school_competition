#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

/* Initial bench values only; no track or motor calibration has been performed. */
#define CONTROL_PERIOD_MS           5U
#define CONTROL_MAX_INTERVAL_MS     15U
#define SENSOR_FILTER_SAMPLES       3U
#define SENSOR_MAX_REGION_WIDTH     4U
#define SENSOR_CONTINUITY_DISTANCE  3.0f
#define SENSOR_CONTINUITY_MARGIN    1.0f
#define LINE_CONFIRM_SAMPLES        3U
#define MOTOR_COMMAND_MAX          255.0f
#define MOTOR_PWM_LIMIT            100.0f
#define MOTOR_LEFT_GAIN            1.0f
#define MOTOR_RIGHT_GAIN           1.0f
#define MOTOR_LEFT_DEADZONE         0.0f
#define MOTOR_RIGHT_DEADZONE        0.0f
#define PID_INTEGRAL_LIMIT         20.0f
#define PID_DERIVATIVE_TAU_S        0.020f
#define BUTTON_DEBOUNCE_MS          20U

#endif
