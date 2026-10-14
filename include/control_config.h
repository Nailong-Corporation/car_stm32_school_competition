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
#define LINE_GAP_MS                 120U
#define LINE_SEARCH_MS              280U
#define LINE_AMBIGUITY_MS           120U
#define LINE_DIRECTION_MAX_AGE_MS   500U
#define LINE_DIRECTION_DEADBAND     0.25f
#define LINE_BASE_PWM               55.0f
#define LINE_TURN_BASE_PWM          40.0f
#define LINE_GAP_BASE_PWM           35.0f
#define LINE_SEARCH_BASE_PWM        30.0f
#define LINE_SEARCH_CORRECTION      18.0f
#define LINE_PWM_RISE_PER_SECOND    200.0f
#define MOTOR_COMMAND_MAX          255.0f
#define MOTOR_PWM_LIMIT            100.0f
#define MOTOR_LEFT_GAIN            1.0f
#define MOTOR_RIGHT_GAIN           1.0f
#define MOTOR_LEFT_DEADZONE         0.0f
#define MOTOR_RIGHT_DEADZONE        0.0f
#define PID_KP                     8.0f
#define PID_KI                     0.0f
#define PID_KD                     0.08f
#define PID_INTEGRAL_LIMIT         20.0f
#define PID_DERIVATIVE_TAU_S        0.020f
#define BUTTON_DEBOUNCE_MS          20U

#endif
