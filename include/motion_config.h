#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

/* Fill measured values, then set CONFIRMED=1. Never guess a cm scale. */
#define ODOMETRY_CALIBRATION_CONFIRMED 0
#define LEFT_WHEEL_DIAMETER_CM        0.0f
#define RIGHT_WHEEL_DIAMETER_CM       0.0f
/* Counts per OUTPUT WHEEL revolution, including gearbox and TI12 x4. */
#define LEFT_COUNTS_PER_WHEEL_REV     0.0f
#define RIGHT_COUNTS_PER_WHEEL_REV    0.0f
#define WHEEL_TRACK_CM               0.0f
#define ENCODER_LEFT_SIGN             1
#define ENCODER_RIGHT_SIGN            1
#define ENCODER_INPUT_FILTER         6U
#define ODOMETRY_MAX_SPEED_CM_S      100.0f
#define SPEED_FILTER_TAU_S           0.030f
#define WHEEL_TARGET_MAX_CM_S        25.0f
#define SPEED_KP                     2.0f
#define SPEED_KI                     8.0f
#define SPEED_KD                     0.0f
#define SPEED_INTEGRAL_LIMIT         10.0f
#define SPEED_FEEDFORWARD             3.0f
#define SPEED_STALL_TARGET_CM_S       3.0f
#define SPEED_STALL_MEASURED_CM_S     0.5f
#define SPEED_STALL_TIMEOUT_S         0.350f
#define POSITION_KP_CM_S              1.8f
#define POSITION_KI_CM_S              0.0f
#define POSITION_KD_CM_S              0.025f
#define POSITION_CURVE_SLOWDOWN       0.35f
#define TARGET_RISE_CM_S2            40.0f
#define HOLD_SPEED_CM_S              6.0f
#define HEADING_KP_CM_S_PER_RAD       12.0f
#define INTERFERENCE_TIMEOUT_MS     400U
#define INTERFERENCE_MAX_DISTANCE_CM 8.0f
#define JUNCTION_CONFIRM_SAMPLES     3U
#define JUNCTION_CONFIRM_DISTANCE_CM 0.10f
#define JUNCTION_CONFIRM_TIMEOUT_MS  100U
#define JUNCTION_ACTION_TIMEOUT_MS   2500U
#define JUNCTION_TURN_MIN_RAD         1.0f
#define JUNCTION_TURN_MAX_RAD         2.1f
#define JUNCTION_WRONG_WAY_RAD        0.35f
#define JUNCTION_CENTER_LIMIT         2.0f
#define JUNCTION_TURN_SPEED_CM_S      6.0f
#define JUNCTION_TURN_DIFF_CM_S       4.0f
#define JUNCTION_CLEAR_DISTANCE_CM    3.0f
#define ROUTE_SEGMENT_TIMEOUT_MS     15000U

#endif
