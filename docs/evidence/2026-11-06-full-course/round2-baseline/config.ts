/**
 * STM32 Smart Car Simulator - Parameters & Configuration
 * Exact parity with motion_config.h, control_config.h, board_config.h
 */

export const CONTROL_PERIOD_MS = 5;
export const CONTROL_PERIOD_S = 0.005;
export const CONTROL_MAX_INTERVAL_MS = 15;

export const SENSOR_FILTER_SAMPLES = 3;
export const SENSOR_MAX_REGION_WIDTH = 6;
export const SENSOR_CONTINUITY_DISTANCE = 3.0;
export const SENSOR_CONTINUITY_MARGIN = 1.0;
export const LINE_CONFIRM_SAMPLES = 3;

/* Wheel & Vehicle Dimensions */
export const LEFT_WHEEL_DIAMETER_CM = 6.5;
export const RIGHT_WHEEL_DIAMETER_CM = 6.5;
export const WHEEL_TRACK_CM = 14.0;
export const GRAY_FORWARD_OFFSET_CM = 24.0;
export const CORNER_RADIUS_CM = 24.0;
export const CORNER_HEADING_KP = 8.0;
export const LEFT_COUNTS_PER_WHEEL_REV = 1560.0;
export const RIGHT_COUNTS_PER_WHEEL_REV = 1560.0;
export const ODOMETRY_DISTANCE_SCALE = 3.05;
export const ODOMETRY_HEADING_SCALE = 0.77;

export const VEHICLE_MAX_SPEED_CM_S = 500.0;
export const ODOMETRY_MAX_SPEED_CM_S = 650.0;
export const SPEED_FILTER_TAU_S = 0.030;
export const WHEEL_TARGET_MAX_CM_S = 500.0;

/* Speed PID Inner Loop */
export const SPEED_KP = 2.0;
export const SPEED_KI = 8.0;
export const SPEED_KD = 0.0;
export const SPEED_INTEGRAL_LIMIT = 10.0;
export const SPEED_FEEDFORWARD = 255.0 / VEHICLE_MAX_SPEED_CM_S;

export const MOTOR_PWM_LIMIT = 255.0;
// Simulator plant assumptions, separate from firmware tuning. Bench calibration pending.
export const MOTOR_RESPONSE_TAU_S = 0.120;
export const MOTOR_COAST_TAU_S = 0.350;
export const MOTOR_BRAKE_TAU_S = 0.080;
export const MOTOR_MAX_ACCEL_CM_S2 = 250.0;
export const MOTOR_MAX_DECEL_CM_S2 = 300.0;

/* Stall & Reverse Protection */
export const SPEED_STALL_TARGET_CM_S = 3.0;
export const SPEED_STALL_MEASURED_CM_S = 0.5;
export const SPEED_STALL_TIMEOUT_S = 0.350;
export const SPEED_REVERSE_THRESHOLD_CM_S = 0.5;
export const SPEED_REVERSE_CONFIRM_S = 0.050;
export const SPEED_REVERSE_CONFIRM_CM = 0.10;
export const SPEED_REVERSE_IMMEDIATE_CM_S = 5.0;

/* Line Position Outer Loop */
export const POSITION_KP_CM_S = 1.8;
export const POSITION_KI_CM_S = 0.0;
export const POSITION_KD_CM_S = 0.025;
export const POSITION_CURVE_SLOWDOWN = 0.35;
export const PID_INTEGRAL_LIMIT = 20.0;
export const PID_DERIVATIVE_TAU_S = 0.020;

export const STARTUP_RAMP_TIME_S = 1.0;
export const TARGET_RISE_CM_S2 = 160.0;
export const TURN_EXIT_ACCEL_CM_S2 = 400.0;
export const LINE_FOLLOW_SPEED_CM_S = 12.0;
export const HOLD_SPEED_CM_S = 40.0;
export const AMBIGUOUS_SPEED_CM_S = 80.0;
export const HEADING_KP_CM_S_PER_RAD = 12.0;
export const INTERFERENCE_TIMEOUT_MS = 400;
export const INTERFERENCE_MAX_DISTANCE_CM = 8.0;

/* Junction Evidence & Confirmation */
export const JUNCTION_CONFIRM_SAMPLES = 3;
export const JUNCTION_CONFIRM_DISTANCE_CM = 0.10;
export const JUNCTION_EARLY_CONFIRM_SAMPLES = 2;
export const JUNCTION_EARLY_CONFIRM_CM = 0.05;
export const JUNCTION_EDGE_CONFIRM_SAMPLES = 4;
export const JUNCTION_EDGE_CONFIRM_CM = 0.15;
export const JUNCTION_EDGE_MAX_WIDTH = 3;
export const JUNCTION_SMOOTH_POSITION_STEP = 2.0;
export const JUNCTION_CONFIRM_TIMEOUT_MS = 100;
export const JUNCTION_CENTER_MEMORY_MS = 250;
export const JUNCTION_CENTER_MEMORY_CM = 1.5;
export const JUNCTION_ACTION_TIMEOUT_MS = 3000;
export const JUNCTION_STRAIGHT_TIMEOUT_MS = 4500;
export const JUNCTION_CLEAR_TIMEOUT_MS = 1500;
export const JUNCTION_TURN_MIN_RAD = 1.35;
export const JUNCTION_EXIT_CONFIRM_MS = 50;
export const JUNCTION_EXIT_CONFIRM_CM = 0.25;
export const JUNCTION_EXIT_ANGLE_MARGIN_RATIO = 0.05;
export const JUNCTION_PASS_SPEED_CM_S = 60.0;
export const JUNCTION_TURN_MAX_RAD = 2.1;
export const JUNCTION_WRONG_WAY_RAD = 0.35;
export const JUNCTION_CENTER_LIMIT = 2.0;
export const JUNCTION_CAPTURE_MIN_RAD = 0.70;
export const JUNCTION_CLEAR_MAX_CM = 20.0;
export const JUNCTION_CAPTURE_MIN_TURN_CM_S = 0.8;
export const JUNCTION_CAPTURE_CENTER_BIAS_CM_S = 1.2;
export const JUNCTION_CAPTURE_HEADING_KP = 8.0;
export const JUNCTION_TURN_TARGET_RAD = 1.570796327;
export const JUNCTION_SETTLE_BAND_RAD = 0.60;
export const JUNCTION_PASS_ALIGNED_RAD = 0.10;
export const JUNCTION_PASS_SETTLED_YAW_RAD_S = 0.15;
export const JUNCTION_SETTLE_SPEED_CM_S = 16.0;
export const JUNCTION_SETTLE_HEADING_KP = 24.0;
export const JUNCTION_YAW_DAMPING = 2.5;
export const JUNCTION_YAW_FILTER_TAU_S = 0.030;
export const JUNCTION_EXIT_SPEED_CM_S = 20.0;
export const COMPONENT_RECOVERY_SPEED_CM_S = 16.0;
export const COMPONENT_RECOVERY_MAX_CM = 12.0;
export const COMPONENT_RECOVERY_TIMEOUT_MS = 1200;

export const BRAKING_CALIBRATION_CONFIRMED = 1;
export const BRAKING_SAFE_DECEL_CM_S2 = 40.0;
export const BRAKING_DISTANCE_MARGIN_CM = 2.0;
export const BRAKING_TARGET_JERK_CM_S3 = 800.0;
export const JUNCTION_SEARCH_TIMEOUT_MS = 2000;
export const JUNCTION_SEARCH_MAX_CM = 12.0;
export const JUNCTION_ENTRY_BRAKE_TIMEOUT_MS = 1200;
export const JUNCTION_ENTRY_BRAKE_MAX_CM = 10.0;
export const JUNCTION_EXIT_ALIGNMENT_RAD = 0.15;
export const JUNCTION_APPROACH_TIMEOUT_MS = 3000;
export const JUNCTION_COUNT_CLEAR_MS = 50;
export const JUNCTION_COUNT_CLEAR_CM = 2.0;
export const JUNCTION_STRAIGHT_EXIT_MAX_WIDTH = 3;
export const JUNCTION_ADJACENT_EXIT_MS = 20;
export const JUNCTION_ADJACENT_EXIT_CM = 0.30;

/* Hexagon Roundabout Bounds */
export const HEXAGON_MAX_ANGLE_DEG = 720.0;
export const HEXAGON_SPEED_CM_S = 16.0;
export const HEXAGON_ENTRY_MIN_RAD = 0.35;
export const HEXAGON_ENTRY_TIMEOUT_MS = 3000;
export const HEXAGON_ENTRY_MAX_CM = 30.0;
export const HEXAGON_ACTION_TIMEOUT_MS = 30000;
export const HEXAGON_ACTION_MAX_CM = 400.0;
export const HEXAGON_EXIT_TIMEOUT_MS = 3000;
export const HEXAGON_EXIT_MAX_CM = 30.0;
export const HEXAGON_OVERANGLE_RAD = 2.0943951;
export const HEXAGON_EXIT_CONFIRM_MS = 100;
export const HEXAGON_EXIT_CONFIRM_CM = 0.5;
export const HEXAGON_ENTRY_LOST_MS = 1200;
export const HEXAGON_ENTRY_LOST_CM = 12.0;

export const ROUTE_SHORT_SPEED_CM_S = 60.0;
export const ROUTE_LONG_SEGMENT_CM = 100.0;
export const NAVIGATION_RECOVERY_MS = 1200;
export const NAVIGATION_RECOVERY_CM = 12.0;
export const NAVIGATION_RECOVERY_SPEED_CM_S = 16.0;

export const GRAY_INVERT_ENABLED = 0;
export const LOST_LINE_STOP_ENABLED = 0;
export const FLOOR_SEAM_FILTER_ENABLED = 0;
