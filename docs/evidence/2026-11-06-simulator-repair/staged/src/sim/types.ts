/**
 * STM32 Smart Car Simulator - Type Definitions
 * Exact parity with stm32_car/include/route.h, gray_sensor.h, odometry.h, line_follow.h
 */

export enum Action {
  LEFT = 0,
  RIGHT = 1,
  STRAIGHT = 2,
  STOP = 3,
  CUSTOM_TURN = 4,
  HEXAGON = 5,
}

export interface RouteStep {
  junction_count: number;  /* Maximum junctions before action; 0 ignores count */
  turn_speed: number;      /* Maximum measured wheel speed before entering a turn, cm/s */
  action: Action;
  cruise_speed: number;    /* cm/s, not PWM */
  angle_deg: number;       /* CUSTOM_TURN / HEXAGON: clockwise positive, unwrapped */
  corner_arc_enabled: boolean;
  segment_length_cm: number; /* START / previous action completion -> target; 0 means unknown */
  comment?: string;        /* Associated comment in route_table.c */
  rawLine?: string;
}

export enum RouteState {
  ROUTE_FOLLOW = 0,
  ROUTE_CANDIDATE = 1,
  ROUTE_EXECUTING = 2,
  ROUTE_CLEARING = 3,
  ROUTE_FINISHED = 4,
  ROUTE_FAILED = 5,
  ROUTE_DISABLED = 6,
  ROUTE_BRAKING = 7,
  ROUTE_RECOVERING = 8,
}

export enum RouteFailure {
  ROUTE_FAILURE_NONE = 0,
  ROUTE_FAILURE_INVALID = 1,
  ROUTE_FAILURE_WINDOW = 2,
  ROUTE_FAILURE_CONFIRM_TIMEOUT = 4,
  ROUTE_FAILURE_ACTION_TIMEOUT = 5,
  ROUTE_FAILURE_TURN_LIMIT = 6,
  ROUTE_FAILURE_WRONG_WAY = 7,
  ROUTE_FAILURE_EXIT_RECAPTURE = 8,
  ROUTE_FAILURE_SEARCH = 9,
  ROUTE_FAILURE_ENTRY_SPEED = 10,
  ROUTE_FAILURE_EXIT_LOST = 11,
  ROUTE_FAILURE_CLEAR_LINE_LOST = 12,
  ROUTE_FAILURE_BRAKE_DISTANCE = 13,
}

export enum JunctionEvidence {
  JUNCTION_EVIDENCE_NONE = 0,
  JUNCTION_EVIDENCE_CENTER_BRANCH = 1,
  JUNCTION_EVIDENCE_EDGE = 2,
  JUNCTION_EVIDENCE_SEPARATED = 3,
}

export enum JunctionReject {
  JUNCTION_REJECT_NONE = 0,
  JUNCTION_REJECT_SHAPE = 1,
  JUNCTION_REJECT_HISTORY = 2,
  JUNCTION_REJECT_SAMPLE_GAP = 3,
  JUNCTION_REJECT_SIDE_CHANGE = 4,
  JUNCTION_REJECT_SMOOTH_EDGE = 5,
}

export enum HexagonState {
  HEXAGON_OFF = 0,
  HEXAGON_ENTERING = 1,
  HEXAGON_FOLLOWING = 2,
  HEXAGON_EXIT_CANDIDATE = 3,
  HEXAGON_EXITING = 4,
  HEXAGON_DONE = 5,
}

export interface HexagonNavigation {
  state: HexagonState;
  phase_ms: number;
  action_ms: number;
  line_ms: number;
  branch_ms: number;
  center_ms: number;
  phase_cm: number;
  line_cm: number;
  branch_cm: number;
  center_cm: number;
  hold_heading: number;
  line_samples: number;
  branch_samples: number;
  center_samples: number;
  line_seen: boolean;
  use_line: boolean;
  center_pending: boolean;
  exit_armed: boolean;
}

export enum GrayKind {
  GRAY_VALID = 0,
  GRAY_NONE = 1,
  GRAY_AMBIGUOUS = 2,
}

export interface GrayObservation {
  kind: GrayKind;
  raw_mask: number;       /* bit i = S_i; 1: line detected, 0: ground */
  line_mask: number;      /* After polarity & filter: 1 = target line */
  selected_mask: number;
  white_count: number;
  region_count: number;
  position: number;       /* -7.0 to +7.0 weighted center */
  prefilter_mask: number;
  seam_mask: number;
}

export enum LineState {
  LINE_IDLE = 0,
  LINE_TRACKING = 1,
  LINE_GAP_HOLD = 2,
  LINE_JUNCTION = 3,
  LINE_STOPPED = 4,
}

export enum LineStopReason {
  LINE_STOP_NONE = 0,
  LINE_STOP_REQUEST = 1,
  LINE_STOP_LOST = 2,
  LINE_STOP_FAULT = 3,
  LINE_STOP_ENCODER = 4,
  LINE_STOP_CALIBRATION = 5,
  LINE_STOP_ROUTE = 6,
  LINE_STOP_ROUTE_COMPLETE = 7,
  LINE_STOP_IMU = 8,
  LINE_STOP_TILT = 9,
}

export enum HeadingSource {
  HEADING_ENCODER = 0,
  HEADING_IMU = 1,
}

export interface EncoderCounts {
  left: number;   /* 16-bit uint */
  right: number;  /* 16-bit uint */
}

export interface OdometryCalibration {
  left_diameter_cm: number;
  right_diameter_cm: number;
  left_counts_per_rev: number;
  right_counts_per_rev: number;
  track_cm: number;
  left_sign: number;
  right_sign: number;
  confirmed: boolean;
}

export interface MotorCommand {
  left_pwm: number;   /* 0..255 */
  right_pwm: number;  /* 0..255 */
  left_forward: boolean;
  right_forward: boolean;
  standby: boolean;
}
