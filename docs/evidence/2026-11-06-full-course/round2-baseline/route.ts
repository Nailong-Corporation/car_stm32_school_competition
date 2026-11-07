import {
  Action,
  RouteState,
  RouteFailure,
  JunctionEvidence,
  JunctionReject,
  HexagonState,
  type HexagonNavigation,
  type RouteStep,
  type GrayObservation,
  GrayKind,
} from './types';
import {
  CONTROL_MAX_INTERVAL_MS,
  LINE_CONFIRM_SAMPLES,
  GRAY_FORWARD_OFFSET_CM,
  SENSOR_MAX_REGION_WIDTH,
  JUNCTION_CONFIRM_SAMPLES,
  JUNCTION_CONFIRM_DISTANCE_CM,
  JUNCTION_EARLY_CONFIRM_SAMPLES,
  JUNCTION_EARLY_CONFIRM_CM,
  JUNCTION_EDGE_CONFIRM_SAMPLES,
  JUNCTION_EDGE_CONFIRM_CM,
  JUNCTION_EDGE_MAX_WIDTH,
  JUNCTION_SMOOTH_POSITION_STEP,
  JUNCTION_CONFIRM_TIMEOUT_MS,
  JUNCTION_CENTER_MEMORY_MS,
  JUNCTION_CENTER_MEMORY_CM,
  JUNCTION_ACTION_TIMEOUT_MS,
  JUNCTION_STRAIGHT_TIMEOUT_MS,
  JUNCTION_CLEAR_TIMEOUT_MS,
  JUNCTION_TURN_MIN_RAD,
  JUNCTION_EXIT_CONFIRM_MS,
  JUNCTION_EXIT_CONFIRM_CM,
  JUNCTION_EXIT_ANGLE_MARGIN_RATIO,
  JUNCTION_TURN_MAX_RAD,
  JUNCTION_WRONG_WAY_RAD,
  JUNCTION_CENTER_LIMIT,
  JUNCTION_CAPTURE_MIN_RAD,
  JUNCTION_CLEAR_MAX_CM,
  JUNCTION_TURN_TARGET_RAD,
  JUNCTION_SETTLE_SPEED_CM_S,
  COMPONENT_RECOVERY_MAX_CM,
  COMPONENT_RECOVERY_TIMEOUT_MS,
  BRAKING_CALIBRATION_CONFIRMED,
  BRAKING_SAFE_DECEL_CM_S2,
  JUNCTION_ENTRY_BRAKE_TIMEOUT_MS,
  JUNCTION_ENTRY_BRAKE_MAX_CM,
  JUNCTION_EXIT_ALIGNMENT_RAD,
  JUNCTION_APPROACH_TIMEOUT_MS,
  JUNCTION_COUNT_CLEAR_MS,
  JUNCTION_COUNT_CLEAR_CM,
  JUNCTION_STRAIGHT_EXIT_MAX_WIDTH,
  JUNCTION_ADJACENT_EXIT_MS,
  JUNCTION_ADJACENT_EXIT_CM,
  HEXAGON_MAX_ANGLE_DEG,
  HEXAGON_SPEED_CM_S,
  HEXAGON_ENTRY_MIN_RAD,
  HEXAGON_ENTRY_TIMEOUT_MS,
  HEXAGON_ENTRY_MAX_CM,
  HEXAGON_ACTION_TIMEOUT_MS,
  HEXAGON_ACTION_MAX_CM,
  HEXAGON_EXIT_TIMEOUT_MS,
  HEXAGON_EXIT_MAX_CM,
  HEXAGON_OVERANGLE_RAD,
  HEXAGON_EXIT_CONFIRM_MS,
  HEXAGON_EXIT_CONFIRM_CM,
  HEXAGON_ENTRY_LOST_MS,
  HEXAGON_ENTRY_LOST_CM,
  ROUTE_SHORT_SPEED_CM_S,
  ROUTE_LONG_SEGMENT_CM,
  NAVIGATION_RECOVERY_MS,
  NAVIGATION_RECOVERY_CM,
  WHEEL_TARGET_MAX_CM_S,
  CORNER_RADIUS_CM,
  WHEEL_TRACK_CM,
  INTERFERENCE_TIMEOUT_MS,
  INTERFERENCE_MAX_DISTANCE_CM,
} from './config';

export function routeIsNarrowLine(o: GrayObservation): boolean {
  return (
    o.kind === GrayKind.GRAY_VALID &&
    o.region_count === 1 &&
    o.white_count > 0 &&
    o.white_count <= SENSOR_MAX_REGION_WIDTH
  );
}

export function routeIsClearLine(planner: RoutePlannerModel, o: GrayObservation): boolean {
  if (!routeIsNarrowLine(o)) return false;
  return (
    planner.currentAction() !== Action.STRAIGHT ||
    (Math.abs(o.position) <= JUNCTION_CENTER_LIMIT &&
      o.white_count <= JUNCTION_STRAIGHT_EXIT_MAX_WIDTH)
  );
}

function separatedCenterBranch(o: GrayObservation): boolean {
  return (
    o.region_count === 2 &&
    o.white_count >= 3 &&
    (o.line_mask & 0x18) !== 0 &&
    (o.line_mask & 0xc3) !== 0
  );
}

function branchSides(o: GrayObservation): number {
  const left = o.line_mask & 0x07;
  const right = (o.line_mask >> 5) & 0x07;
  return (
    (left && (left & (left - 1)) ? 1 : 0) |
    (right && (right & (right - 1)) ? 2 : 0)
  );
}

function separatedSides(o: GrayObservation): number {
  if (!separatedCenterBranch(o)) return 0;
  let center = o.line_mask & 0x18;
  for (let i = 0; i < 7; ++i) {
    center |= o.line_mask & (((center << 1) | (center >> 1)) & 0xff);
  }
  if (!(center & (center - 1))) return 0;
  const outer = o.line_mask & (~center & 0xff);
  return ((outer & 0x03) ? 1 : 0) | ((outer & 0xc0) ? 2 : 0);
}

function junctionShape(o: GrayObservation): { kind: JunctionEvidence; sides: number } {
  const separated = separatedSides(o);
  if (separated) {
    return { kind: JunctionEvidence.JUNCTION_EVIDENCE_SEPARATED, sides: separated };
  }
  if (o.region_count > 1 && o.white_count >= 3) {
    let sides = ((o.line_mask & 0x07) ? 1 : 0) | ((o.line_mask & 0xe0) ? 2 : 0);
    if (!sides) sides = 3;
    return { kind: JunctionEvidence.JUNCTION_EVIDENCE_SEPARATED, sides };
  }
  if (o.region_count !== 1) {
    return { kind: JunctionEvidence.JUNCTION_EVIDENCE_NONE, sides: 0 };
  }
  const sides = branchSides(o);
  if (sides && (o.line_mask & 0x18) && o.white_count >= 3) {
    return { kind: JunctionEvidence.JUNCTION_EVIDENCE_CENTER_BRANCH, sides };
  }
  if (sides && routeIsNarrowLine(o) && o.white_count <= JUNCTION_EDGE_MAX_WIDTH) {
    return { kind: JunctionEvidence.JUNCTION_EVIDENCE_EDGE, sides };
  }
  return { kind: JunctionEvidence.JUNCTION_EVIDENCE_NONE, sides: 0 };
}

export class RoutePlannerModel {
  public steps: RouteStep[] = [];
  public count = 0;
  public index = 0;
  public state: RouteState = RouteState.ROUTE_FOLLOW;
  public failure: RouteFailure = RouteFailure.ROUTE_FAILURE_NONE;

  public hexagon: HexagonNavigation = {
    state: HexagonState.HEXAGON_OFF,
    phase_ms: 0,
    action_ms: 0,
    line_ms: 0,
    branch_ms: 0,
    center_ms: 0,
    phase_cm: 0,
    line_cm: 0,
    branch_cm: 0,
    center_cm: 0,
    hold_heading: 0,
    line_samples: 0,
    branch_samples: 0,
    center_samples: 0,
    line_seen: false,
    use_line: false,
    center_pending: false,
    exit_armed: false,
  };

  public navigation_warning = RouteFailure.ROUTE_FAILURE_NONE;
  public navigation_recoveries = 0;
  public next_segment_origin_cm = 0;
  public next_segment_ms = 0;
  public next_segment_origin_valid = false;
  public junctions_seen = 0;
  public passing_junction = false;
  public final_segment_active = false;
  public search_pending = false;
  public final_segment_origin_cm = 0;
  public search_origin_cm = 0;
  public search_ms = 0;
  public last_sample_ms = 0;
  public measured_left_cm_s = 0;
  public measured_right_cm_s = 0;

  public origin_cm = 0;
  public entry_cm = 0;
  public entry_heading = 0;
  public clear_cm = 0;
  public approach_center_cm = 0;
  public action_start_heading = 0;
  public turn_approaching = false;
  public turn_approach_origin_cm = 0;
  public segment_ms = 0;
  public phase_ms = 0;
  public approach_center_ms = 0;
  public exit_candidate_ms = 0;
  public exit_candidate_cm = 0;
  public exit_candidate_pending = false;
  public repeat_action: Action = Action.LEFT;
  public repeat_count = 0;
  public completed_laps = 0;
  public candidate_count = 0;
  public exit_count = 0;
  public approach_center_count = 0;
  public have_approach_center = false;
  public valid = false;
  public stop_pending = false;
  public callback_dispatched = false;
  public corner_arc_complete = false;
  public turn_capturing = false;
  public lap_boundary = false;
  public capture_line_seen = false;
  public line_exit_confirmed = false;
  public capture_line_samples = 0;
  public capture_line_ms = 0;
  public capture_line_cm = 0;
  public line_exit_heading_rad = 0;
  public turn_ended_on_line = false;
  public next_junction_samples = 0;
  public next_junction_ms = 0;
  public next_junction_cm = 0;
  public candidate_separated = false;
  public line_confirmed_exit_used = false;
  public candidate_kind = JunctionEvidence.JUNCTION_EVIDENCE_NONE;
  public last_confirmed_kind = JunctionEvidence.JUNCTION_EVIDENCE_NONE;
  public candidate_reject = JunctionReject.JUNCTION_REJECT_NONE;
  public candidate_sides = 0;
  public next_junction_sides = 0;
  public previous_line_mask = 0;
  public smooth_edge_sides = 0;
  public previous_line_valid = false;
  public previous_line_position = 0;
  public candidate_restarts = 0;
  public smooth_edge_rejections = 0;
  public component_recovery_active = false;
  public component_recovery_used = false;
  public component_recovery_line_samples = 0;
  public component_recovery_ms = 0;
  public component_recovery_cm = 0;
  public component_recovery_heading = 0;

  public stop_callback?: (step_index: number) => void;

  public init(steps: RouteStep[], callback?: (step_index: number) => void): void {
    this.steps = steps.map((s) => ({ ...s }));
    this.count = steps.length;
    this.repeat_count = 0;
    this.repeat_action = Action.LEFT;
    this.stop_callback = callback;
    this.valid = steps !== null && steps.length > 0;

    for (let i = 0; this.valid && i < this.count; ++i) {
      const s = steps[i];
      this.valid =
        (s.junction_count > 0 || s.segment_length_cm > 0) &&
        s.action <= Action.HEXAGON &&
        s.cruise_speed > 0 &&
        s.cruise_speed <= WHEEL_TARGET_MAX_CM_S &&
        Number.isFinite(s.turn_speed) &&
        s.turn_speed > 0 &&
        s.turn_speed <= s.cruise_speed &&
        Number.isFinite(s.segment_length_cm) &&
        s.segment_length_cm >= 0 &&
        (s.segment_length_cm === 0 ||
          (BRAKING_CALIBRATION_CONFIRMED &&
            Number.isFinite(BRAKING_SAFE_DECEL_CM_S2) &&
            BRAKING_SAFE_DECEL_CM_S2 > 0)) &&
        (s.action !== Action.CUSTOM_TURN ||
          (Number.isFinite(s.angle_deg) && s.angle_deg !== 0 && Math.abs(s.angle_deg) <= 360)) &&
        (s.action !== Action.HEXAGON ||
          (Number.isFinite(s.angle_deg) &&
            Math.abs(s.angle_deg) >= 60.0 &&
            Math.abs(s.angle_deg) <= HEXAGON_MAX_ANGLE_DEG &&
            !s.corner_arc_enabled)) &&
        (s.action !== Action.STOP || i === this.count - 1) &&
        (!s.corner_arc_enabled ||
          (Number.isFinite(CORNER_RADIUS_CM) && CORNER_RADIUS_CM > WHEEL_TRACK_CM * 0.5));
    }

    if (this.valid && steps[this.count - 1].action !== Action.STOP) this.valid = false;
    this.state = this.valid ? RouteState.ROUTE_FOLLOW : RouteState.ROUTE_FAILED;
    this.failure = this.valid ? RouteFailure.ROUTE_FAILURE_NONE : RouteFailure.ROUTE_FAILURE_INVALID;
  }

  public start(d: number, now: number): void {
    this.index = 0;
    this.completed_laps = 0;
    this.junctions_seen = 0;
    this.origin_cm = this.final_segment_origin_cm = d;
    this.segment_ms = now;
    this.passing_junction = this.lap_boundary = false;
    this.search_pending = this.stop_pending = this.callback_dispatched = false;
    this.final_segment_active = this.valid;
    this.navigation_warning = RouteFailure.ROUTE_FAILURE_NONE;
    this.navigation_recoveries = 0;
    this.failure = RouteFailure.ROUTE_FAILURE_NONE;
    this.last_confirmed_kind = JunctionEvidence.JUNCTION_EVIDENCE_NONE;
    this.candidate_restarts = this.smooth_edge_rejections = 0;
    this.state = this.valid && Number.isFinite(d) ? RouteState.ROUTE_FOLLOW : RouteState.ROUTE_FAILED;
    if (this.state === RouteState.ROUTE_FAILED) this.failure = RouteFailure.ROUTE_FAILURE_INVALID;
    this.resetEvidence(now);
  }

  public setSpeed(left: number, right: number): void {
    this.measured_left_cm_s = left;
    this.measured_right_cm_s = right;
  }

  public fail(reason: RouteFailure): void {
    if (this.failure === RouteFailure.ROUTE_FAILURE_NONE) this.failure = reason;
    this.state = RouteState.ROUTE_FAILED;
  }

  public anotherLap(): boolean {
    return this.repeat_count === 0 || this.completed_laps < this.repeat_count - 1;
  }

  public currentAction(): Action {
    if (this.passing_junction) return Action.STRAIGHT;
    const action = this.steps[this.index].action;
    if (action === Action.CUSTOM_TURN || action === Action.HEXAGON) {
      return this.steps[this.index].angle_deg < 0 ? Action.LEFT : Action.RIGHT;
    }
    return action === Action.STOP && (this.lap_boundary || this.anotherLap())
      ? this.repeat_action
      : action;
  }

  public usesCornerArc(): boolean {
    return (
      this.steps[this.index].action !== Action.HEXAGON &&
      this.steps[this.index].corner_arc_enabled &&
      Math.abs(Math.abs(this.turnTargetRad()) - JUNCTION_TURN_TARGET_RAD) < 0.0001
    );
  }

  public turnTargetRad(): number {
    if (
      this.steps[this.index].action === Action.CUSTOM_TURN ||
      this.steps[this.index].action === Action.HEXAGON
    ) {
      return this.steps[this.index].angle_deg * (JUNCTION_TURN_TARGET_RAD / 90.0);
    }
    const action = this.currentAction();
    return action === Action.LEFT
      ? -JUNCTION_TURN_TARGET_RAD
      : action === Action.RIGHT
      ? JUNCTION_TURN_TARGET_RAD
      : 0.0;
  }

  public turnThresholdRad(threshold: number): number {
    if (this.steps[this.index].action === Action.HEXAGON) {
      return Math.abs(this.turnTargetRad()) + HEXAGON_OVERANGLE_RAD;
    }
    if (this.steps[this.index].action !== Action.CUSTOM_TURN) return threshold;
    const target = Math.abs(this.turnTargetRad());
    if (threshold === JUNCTION_TURN_MIN_RAD) return target;
    if (threshold > JUNCTION_TURN_TARGET_RAD) {
      return target + threshold - JUNCTION_TURN_TARGET_RAD;
    }
    return Math.max(
      target + threshold - JUNCTION_TURN_TARGET_RAD,
      (target * threshold) / JUNCTION_TURN_TARGET_RAD
    );
  }

  public actionTimeoutMs(): number {
    if (this.steps[this.index].action === Action.HEXAGON && !this.passing_junction) {
      return HEXAGON_ACTION_TIMEOUT_MS;
    }
    if (this.currentAction() === Action.STRAIGHT) return JUNCTION_STRAIGHT_TIMEOUT_MS;
    const scale = Math.max(1.0, Math.abs(this.turnTargetRad()) / JUNCTION_TURN_TARGET_RAD);
    return Math.ceil(JUNCTION_ACTION_TIMEOUT_MS * scale);
  }

  public entrySpeedLimit(): number {
    return this.entrySpeedLimitAt(this.index, true);
  }

  private entrySpeedLimitAt(index: number, active_action: boolean): number {
    const a = this.steps[index].action;
    const turn =
      a === Action.LEFT ||
      a === Action.RIGHT ||
      a === Action.CUSTOM_TURN ||
      a === Action.HEXAGON ||
      (a === Action.STOP &&
        ((active_action && this.lap_boundary) || this.anotherLap()) &&
        this.repeat_action !== Action.STRAIGHT);
    let limit = this.steps[index].turn_speed;
    if (a === Action.HEXAGON) limit = Math.min(limit, HEXAGON_SPEED_CM_S);
    return turn ? Math.min(limit, JUNCTION_SETTLE_SPEED_CM_S) : limit;
  }

  private entrySpeedReady(): boolean {
    const limit = this.entrySpeedLimit();
    return (
      Math.abs(this.measured_left_cm_s) <= limit &&
      Math.abs(this.measured_right_cm_s) <= limit
    );
  }

  public planningIndex(): number {
    if (
      this.state === RouteState.ROUTE_CLEARING &&
      !this.passing_junction &&
      this.next_segment_origin_valid
    ) {
      const next = this.lap_boundary ? 0 : this.index + 1;
      if (next < this.count) return next;
    }
    return this.index;
  }

  public planningOrigin(): number {
    return this.state === RouteState.ROUTE_CLEARING &&
      !this.passing_junction &&
      this.next_segment_origin_valid
      ? this.next_segment_origin_cm
      : this.final_segment_origin_cm;
  }

  public planningEntrySpeedLimit(): number {
    const preview =
      this.state === RouteState.ROUTE_CLEARING &&
      !this.passing_junction &&
      this.next_segment_origin_valid;
    return this.entrySpeedLimitAt(this.planningIndex(), !preview);
  }

  public cruiseSpeed(): number {
    const s = this.steps[this.planningIndex()];
    return s.segment_length_cm > ROUTE_LONG_SEGMENT_CM
      ? s.cruise_speed
      : ROUTE_SHORT_SPEED_CM_S;
  }

  public targetReady(d: number, next: boolean): boolean {
    const s = this.steps[this.index];
    if (s.junction_count > 0) {
      return this.junctions_seen + (next ? 1 : 0) >= s.junction_count;
    }
    return s.segment_length_cm > 0 && d - this.origin_cm >= s.segment_length_cm;
  }

  private refreshDistance(d: number, now: number): void {
    const s = this.steps[this.planningIndex()];
    if (
      s.segment_length_cm > 0 &&
      d - this.planningOrigin() >= s.segment_length_cm &&
      !this.search_pending
    ) {
      this.search_pending = true;
      this.search_ms = now;
      this.search_origin_cm = d;
    }
  }

  private resetEvidence(now: number): void {
    this.next_segment_origin_valid = false;
    this.candidate_count = this.exit_count = this.approach_center_count = 0;
    this.have_approach_center = this.exit_candidate_pending = false;
    this.turn_capturing = this.corner_arc_complete = false;
    this.line_exit_confirmed = this.turn_ended_on_line = this.capture_line_seen = false;
    this.capture_line_samples = 0;
    this.next_junction_samples = 0;
    this.candidate_separated = false;
    this.line_confirmed_exit_used = false;
    this.candidate_kind = JunctionEvidence.JUNCTION_EVIDENCE_NONE;
    this.candidate_sides = 0;
    this.next_junction_sides = 0;
    this.candidate_reject = JunctionReject.JUNCTION_REJECT_NONE;
    this.previous_line_valid = false;
    this.smooth_edge_sides = 0;
    this.component_recovery_active = this.component_recovery_used = false;
    this.component_recovery_line_samples = 0;
    this.last_sample_ms = now;
    this.hexagon = {
      state: HexagonState.HEXAGON_OFF,
      phase_ms: 0,
      action_ms: 0,
      line_ms: 0,
      branch_ms: 0,
      center_ms: 0,
      phase_cm: 0,
      line_cm: 0,
      branch_cm: 0,
      center_cm: 0,
      hold_heading: 0,
      line_samples: 0,
      branch_samples: 0,
      center_samples: 0,
      line_seen: false,
      use_line: false,
      center_pending: false,
      exit_armed: false,
    };
  }

  private beginAction(d: number, h: number, now: number): void {
    this.state = RouteState.ROUTE_EXECUTING;
    this.phase_ms = now;
    this.entry_cm = d;
    this.entry_heading = this.action_start_heading = h;
    this.turn_approach_origin_cm = d;
    this.turn_approaching = !this.passing_junction && !this.usesCornerArc() &&
      Math.abs(Math.abs(this.turnTargetRad()) - JUNCTION_TURN_TARGET_RAD) < 1e-4 &&
      this.steps[this.index].action !== Action.HEXAGON;
    this.resetEvidence(now);
    if (this.steps[this.index].action === Action.HEXAGON && !this.passing_junction) {
      this.hexagon = {
        state: HexagonState.HEXAGON_ENTERING,
        phase_ms: now,
        action_ms: now,
        line_ms: now,
        branch_ms: 0,
        center_ms: 0,
        phase_cm: d,
        line_cm: d,
        branch_cm: 0,
        center_cm: 0,
        hold_heading: h,
        line_samples: 0,
        branch_samples: 0,
        center_samples: 0,
        line_seen: false,
        use_line: false,
        center_pending: false,
        exit_armed: false,
      };
    }
  }

  private acceptTarget(d: number, h: number, now: number): void {
    if (this.steps[this.index].action === Action.STOP) {
      const repeat = this.anotherLap();
      this.completed_laps++;
      if (!repeat) {
        this.state = RouteState.ROUTE_FINISHED;
        this.stop_pending = true;
        return;
      }
      this.lap_boundary = true;
    }
    this.beginAction(d, h, now);
  }

  private anchorNextSegment(d: number, now: number): void {
    this.next_segment_origin_cm = d;
    this.next_segment_ms = now;
    this.next_segment_origin_valid = true;
    this.search_pending = false;
  }

  private enterClear(d: number, now: number): void {
    this.state = RouteState.ROUTE_CLEARING;
    this.phase_ms = now;
    this.clear_cm = d;
    this.exit_count = 0;
    this.exit_candidate_pending = false;
    this.turn_capturing = false;
    this.turn_ended_on_line = true;
    if (!this.passing_junction && this.currentAction() !== Action.STRAIGHT) {
      this.anchorNextSegment(d, now);
    }
    if (this.currentAction() === Action.STRAIGHT) {
      this.capture_line_ms = now;
      this.capture_line_cm = d;
    }
  }

  private completeClear(d: number, now: number): boolean {
    const passed = this.passing_junction;
    if (passed) {
      this.passing_junction = false;
    } else {
      this.index = this.lap_boundary ? 0 : this.index + 1;
      this.lap_boundary = false;
      if (this.index >= this.count) {
        this.fail(RouteFailure.ROUTE_FAILURE_INVALID);
        return false;
      }
      this.junctions_seen = 0;
    }

    if (!passed) {
      this.origin_cm = this.next_segment_origin_valid
        ? this.next_segment_origin_cm
        : this.clear_cm;
      this.segment_ms = this.next_segment_origin_valid
        ? this.next_segment_ms
        : this.phase_ms;
      this.final_segment_origin_cm = this.origin_cm;
      this.search_pending =
        this.steps[this.index].segment_length_cm > 0 &&
        d - this.origin_cm >= this.steps[this.index].segment_length_cm;
      if (this.search_pending) {
        this.search_ms = now;
        this.search_origin_cm = d;
      }
    }

    this.final_segment_active = true;
    this.state = RouteState.ROUTE_FOLLOW;
    this.resetEvidence(now);
    return true;
  }

  private componentRecovery(
    o: GrayObservation,
    d: number,
    h: number,
    now: number,
    adjacent: boolean,
    eligible: boolean
  ): boolean {
    const graphic =
      o.white_count >= 2 &&
      (o.region_count > 1 || o.white_count > JUNCTION_STRAIGHT_EXIT_MAX_WIDTH);
    if (
      !this.component_recovery_active &&
      !this.component_recovery_used &&
      eligible &&
      graphic &&
      adjacent
    ) {
      this.component_recovery_active = this.component_recovery_used = true;
      this.component_recovery_ms = now;
      this.component_recovery_cm = d;
      this.component_recovery_heading =
        this.currentAction() === Action.STRAIGHT ? this.entry_heading : h;
      this.component_recovery_line_samples = 0;
      this.exit_count = 0;
      this.exit_candidate_pending = false;
      this.next_junction_samples = this.approach_center_count = 0;
    }
    if (!this.component_recovery_active) return false;
    this.turn_capturing = false;

    if (
      now - this.component_recovery_ms >= COMPONENT_RECOVERY_TIMEOUT_MS ||
      Math.abs(d - this.component_recovery_cm) >= COMPONENT_RECOVERY_MAX_CM
    ) {
      if (this.steps[this.index].action === Action.HEXAGON) {
        this.fail(RouteFailure.ROUTE_FAILURE_EXIT_RECAPTURE);
        return true;
      }
      this.component_recovery_active = false;
      return false;
    }

    const line =
      routeIsNarrowLine(o) &&
      o.white_count <= JUNCTION_STRAIGHT_EXIT_MAX_WIDTH &&
      Math.abs(o.position) <= JUNCTION_CENTER_LIMIT;
    if (!line || !adjacent) this.component_recovery_line_samples = 0;
    if (line && adjacent && this.component_recovery_line_samples < LINE_CONFIRM_SAMPLES) {
      this.component_recovery_line_samples++;
    }
    if (this.component_recovery_line_samples < LINE_CONFIRM_SAMPLES) return true;

    this.component_recovery_active = false;
    this.capture_line_ms = now;
    this.capture_line_cm = d;
    this.capture_line_samples = LINE_CONFIRM_SAMPLES;
    return false;
  }

  private hexagonUpdate(
    o: GrayObservation,
    d: number,
    h: number,
    now: number,
    adjacent: boolean
  ): void {
    const x = this.hexagon;
    const sign = this.currentAction() === Action.LEFT ? -1.0 : 1.0;
    const turn = sign * (h - this.entry_heading);
    const target = Math.abs(this.turnTargetRad());
    const entering = x.state === HexagonState.HEXAGON_ENTERING;
    const exiting = x.state === HexagonState.HEXAGON_EXITING;
    const line =
      routeIsNarrowLine(o) && o.white_count <= JUNCTION_STRAIGHT_EXIT_MAX_WIDTH;
    const center = line && Math.abs(o.position) <= JUNCTION_CENTER_LIMIT;
    x.use_line = false;

    if (turn < -JUNCTION_WRONG_WAY_RAD || turn > target + HEXAGON_OVERANGLE_RAD) {
      this.fail(
        turn < 0
          ? RouteFailure.ROUTE_FAILURE_WRONG_WAY
          : RouteFailure.ROUTE_FAILURE_TURN_LIMIT
      );
      return;
    }

    if (
      now - x.action_ms >= HEXAGON_ACTION_TIMEOUT_MS ||
      Math.abs(d - this.entry_cm) >= HEXAGON_ACTION_MAX_CM ||
      (entering &&
        (now - x.phase_ms >= HEXAGON_ENTRY_TIMEOUT_MS ||
          Math.abs(d - x.phase_cm) >= HEXAGON_ENTRY_MAX_CM)) ||
      (exiting &&
        (now - x.phase_ms >= HEXAGON_EXIT_TIMEOUT_MS ||
          Math.abs(d - x.phase_cm) >= HEXAGON_EXIT_MAX_CM))
    ) {
      this.fail(RouteFailure.ROUTE_FAILURE_ACTION_TIMEOUT);
      return;
    }

    const loss_ms = entering ? HEXAGON_ENTRY_LOST_MS : INTERFERENCE_TIMEOUT_MS;
    const loss_cm = entering ? HEXAGON_ENTRY_LOST_CM : INTERFERENCE_MAX_DISTANCE_CM;
    if (
      !this.component_recovery_active &&
      (now - x.line_ms >= loss_ms || Math.abs(d - x.line_cm) >= loss_cm)
    ) {
      this.fail(RouteFailure.ROUTE_FAILURE_EXIT_LOST);
      return;
    }

    if (!adjacent) {
      x.line_samples = x.branch_samples = x.center_samples = 0;
      x.center_pending = false;
      x.exit_armed = false;
      this.have_approach_center = false;
      if (x.state === HexagonState.HEXAGON_EXIT_CANDIDATE) {
        x.state = HexagonState.HEXAGON_FOLLOWING;
      }
    }

    const { kind, sides } = junctionShape(o);
    const exit_side = sign > 0 ? 1 : 2;
    const history =
      this.have_approach_center &&
      now - this.approach_center_ms < JUNCTION_CENTER_MEMORY_MS &&
      d >= this.approach_center_cm &&
      d - this.approach_center_cm <= JUNCTION_CENTER_MEMORY_CM;

    if (turn < target || this.component_recovery_active) x.exit_armed = false;
    const branch =
      adjacent &&
      turn >= target &&
      sides === exit_side &&
      kind !== JunctionEvidence.JUNCTION_EVIDENCE_NONE &&
      !this.component_recovery_active;

    if (x.state === HexagonState.HEXAGON_FOLLOWING && x.exit_armed && branch && history) {
      x.exit_armed = false;
      x.state = HexagonState.HEXAGON_EXIT_CANDIDATE;
      x.branch_ms = now;
      x.branch_cm = d;
      x.branch_samples = 0;
      this.candidate_kind = kind;
      x.hold_heading = h;
    }

    if (x.state === HexagonState.HEXAGON_EXIT_CANDIDATE) {
      if (now - x.branch_ms >= JUNCTION_CONFIRM_TIMEOUT_MS) {
        this.fail(RouteFailure.ROUTE_FAILURE_CONFIRM_TIMEOUT);
        return;
      }
      if (!branch) {
        x.state = HexagonState.HEXAGON_FOLLOWING;
        x.branch_samples = 0;
      } else {
        if (kind !== JunctionEvidence.JUNCTION_EVIDENCE_CENTER_BRANCH) {
          this.candidate_kind = kind;
        }
        const weak = this.candidate_kind !== JunctionEvidence.JUNCTION_EVIDENCE_CENTER_BRANCH;
        const samples = weak ? JUNCTION_EDGE_CONFIRM_SAMPLES : JUNCTION_CONFIRM_SAMPLES;
        const distance = weak ? JUNCTION_EDGE_CONFIRM_CM : JUNCTION_CONFIRM_DISTANCE_CM;
        if (x.branch_samples < samples) x.branch_samples++;
        if (x.branch_samples >= samples && d - x.branch_cm >= distance) {
          x.state = HexagonState.HEXAGON_EXITING;
          x.phase_ms = now;
          x.phase_cm = d;
          x.hold_heading = h;
          x.center_samples = 0;
          x.center_pending = false;
          x.line_samples = 0;
        }
        return;
      }
    }

    const recovering = this.component_recovery_active;
    if (this.componentRecovery(o, d, h, now, adjacent, x.line_seen && !line)) {
      x.line_samples = x.center_samples = 0;
      x.center_pending = false;
      this.have_approach_center = false;
      return;
    }

    if (recovering) {
      x.line_ms = now;
      x.line_cm = d;
      x.line_samples = LINE_CONFIRM_SAMPLES;
    }

    if (!line || !adjacent) x.line_samples = 0;
    if (line && adjacent && x.line_samples < LINE_CONFIRM_SAMPLES) x.line_samples++;
    const stable = x.line_samples >= LINE_CONFIRM_SAMPLES;

    if (stable && (!entering || turn >= HEXAGON_ENTRY_MIN_RAD)) {
      x.line_seen = true;
      x.line_ms = now;
      x.line_cm = d;
      x.hold_heading = h;
      this.capture_line_seen = true;
      this.capture_line_ms = now;
      this.capture_line_cm = d;
    }

    if (entering) {
      if (stable && turn >= HEXAGON_ENTRY_MIN_RAD) {
        x.state = HexagonState.HEXAGON_FOLLOWING;
        x.phase_ms = now;
        x.phase_cm = d;
      } else {
        return;
      }
    }

    x.use_line = line && stable;
    if (center && stable) {
      this.have_approach_center = true;
      this.approach_center_ms = now;
      this.approach_center_cm = d;
      if (turn >= target && x.state === HexagonState.HEXAGON_FOLLOWING) {
        x.exit_armed = true;
      }
    }

    if (x.state !== HexagonState.HEXAGON_EXITING) return;

    const exit = center && stable && adjacent && turn >= target;
    if (!exit) {
      x.center_samples = 0;
      x.center_pending = false;
    }
    if (exit && !x.center_pending) {
      x.center_pending = true;
      x.center_ms = now;
      x.center_cm = d;
    }
    if (exit && x.center_samples < LINE_CONFIRM_SAMPLES) x.center_samples++;

    if (
      x.center_samples >= LINE_CONFIRM_SAMPLES &&
      now - x.center_ms >= HEXAGON_EXIT_CONFIRM_MS &&
      d - x.center_cm >= HEXAGON_EXIT_CONFIRM_CM
    ) {
      x.state = HexagonState.HEXAGON_DONE;
      this.line_exit_confirmed = true;
      this.line_exit_heading_rad = h;
      this.enterClear(d, now);
    }
  }

  public recoverNavigation(d: number, h: number, now: number): boolean {
    if (this.state !== RouteState.ROUTE_FAILED) return false;
    switch (this.failure) {
      case RouteFailure.ROUTE_FAILURE_CONFIRM_TIMEOUT:
      case RouteFailure.ROUTE_FAILURE_ACTION_TIMEOUT:
      case RouteFailure.ROUTE_FAILURE_TURN_LIMIT:
      case RouteFailure.ROUTE_FAILURE_WRONG_WAY:
      case RouteFailure.ROUTE_FAILURE_SEARCH:
      case RouteFailure.ROUTE_FAILURE_WINDOW:
      case RouteFailure.ROUTE_FAILURE_ENTRY_SPEED:
        break;
      default:
        return false;
    }
    this.navigation_warning = this.failure;
    this.navigation_recoveries++;
    this.failure = RouteFailure.ROUTE_FAILURE_NONE;
    this.resetEvidence(now);
    this.state = RouteState.ROUTE_RECOVERING;
    this.phase_ms = now;
    this.clear_cm = d;
    this.entry_heading = h;
    this.capture_line_ms = now;
    this.capture_line_cm = d;
    return true;
  }

  public update(o: GrayObservation, d: number, h: number, now: number): void {
    if (
      this.state === RouteState.ROUTE_FINISHED ||
      this.state === RouteState.ROUTE_FAILED ||
      this.state === RouteState.ROUTE_DISABLED
    ) {
      return;
    }

    if (
      !this.valid ||
      this.index >= this.count ||
      !Number.isFinite(d) ||
      !Number.isFinite(h) ||
      !Number.isFinite(this.measured_left_cm_s) ||
      !Number.isFinite(this.measured_right_cm_s)
    ) {
      this.fail(RouteFailure.ROUTE_FAILURE_INVALID);
      return;
    }

    const s = this.steps[this.index];
    if (this.state === RouteState.ROUTE_RECOVERING) {
      if (
        now - this.capture_line_ms >= NAVIGATION_RECOVERY_MS ||
        Math.abs(d - this.capture_line_cm) >= NAVIGATION_RECOVERY_CM
      ) {
        this.fail(RouteFailure.ROUTE_FAILURE_EXIT_LOST);
        return;
      }
      const narrow = routeIsNarrowLine(o);
      const continuous = now - this.last_sample_ms <= CONTROL_MAX_INTERVAL_MS;
      this.last_sample_ms = now;

      if (!narrow || !continuous) {
        this.exit_count = 0;
        this.exit_candidate_pending = false;
      }
      if (narrow && !this.exit_candidate_pending) {
        this.exit_candidate_pending = true;
        this.exit_candidate_ms = now;
        this.exit_candidate_cm = d;
      }
      if (narrow && this.exit_count < LINE_CONFIRM_SAMPLES) this.exit_count++;
      if (narrow && this.exit_count >= LINE_CONFIRM_SAMPLES) {
        this.capture_line_ms = now;
        this.capture_line_cm = d;
      }
      if (
        this.exit_count >= LINE_CONFIRM_SAMPLES &&
        now - this.exit_candidate_ms >= JUNCTION_COUNT_CLEAR_MS &&
        d - this.exit_candidate_cm >= JUNCTION_COUNT_CLEAR_CM
      ) {
        this.passing_junction = false;
        this.state = RouteState.ROUTE_FOLLOW;
        this.resetEvidence(now);
        return;
      }
      return;
    }

    const adjacent = now - this.last_sample_ms <= CONTROL_MAX_INTERVAL_MS;
    this.last_sample_ms = now;

    if (
      this.state === RouteState.ROUTE_FOLLOW ||
      this.state === RouteState.ROUTE_CANDIDATE ||
      this.state === RouteState.ROUTE_CLEARING
    ) {
      this.refreshDistance(d, now);
    }

    if (this.state === RouteState.ROUTE_FOLLOW || this.state === RouteState.ROUTE_CANDIDATE) {
      const narrow = routeIsNarrowLine(o);
      const center = narrow && Math.abs(o.position) <= JUNCTION_CENTER_LIMIT;

      if (
        adjacent &&
        narrow &&
        o.white_count <= JUNCTION_EDGE_MAX_WIDTH &&
        !center &&
        this.previous_line_valid &&
        (this.previous_line_mask & o.line_mask) &&
        Math.abs(o.position - this.previous_line_position) <= JUNCTION_SMOOTH_POSITION_STEP &&
        Math.abs(o.position) > Math.abs(this.previous_line_position)
      ) {
        this.smooth_edge_sides |= o.position < 0 ? 1 : 2;
      }

      this.previous_line_valid =
        adjacent && narrow && o.white_count <= JUNCTION_EDGE_MAX_WIDTH;
      this.previous_line_mask = o.line_mask;
      this.previous_line_position = o.position;

      if (center) {
        if (!adjacent) this.approach_center_count = 0;
        if (this.approach_center_count < LINE_CONFIRM_SAMPLES) this.approach_center_count++;
        if (this.approach_center_count >= LINE_CONFIRM_SAMPLES) {
          this.have_approach_center = true;
          this.approach_center_ms = now;
          this.approach_center_cm = d;
          this.smooth_edge_sides = 0;
        }
      } else {
        this.approach_center_count = 0;
      }

      const recent_center =
        this.have_approach_center &&
        now - this.approach_center_ms < JUNCTION_CENTER_MEMORY_MS &&
        d >= this.approach_center_cm &&
        d - this.approach_center_cm <= JUNCTION_CENTER_MEMORY_CM;

      const history = (this.state === RouteState.ROUTE_CANDIDATE && d >= this.entry_cm) || recent_center;
      const { kind, sides } = junctionShape(o);
      // A gradually shifting single line can still include a center channel
      // (e.g. 0x38 -> 0x30 -> 0x70). Keep its same-side suppression when
      // classified CENTER_BRANCH; an abrupt or separated branch stays eligible.
      const smooth =
        (kind === JunctionEvidence.JUNCTION_EVIDENCE_EDGE ||
          (kind === JunctionEvidence.JUNCTION_EVIDENCE_CENTER_BRANCH &&
            o.white_count <= JUNCTION_EDGE_MAX_WIDTH)) &&
        (sides & this.smooth_edge_sides) !== 0;
      const shape = history && kind !== JunctionEvidence.JUNCTION_EVIDENCE_NONE && !smooth;

      if (!shape || !adjacent) {
        this.candidate_reject = !adjacent
          ? JunctionReject.JUNCTION_REJECT_SAMPLE_GAP
          : smooth
          ? JunctionReject.JUNCTION_REJECT_SMOOTH_EDGE
          : !history
          ? JunctionReject.JUNCTION_REJECT_HISTORY
          : JunctionReject.JUNCTION_REJECT_SHAPE;

        if (smooth) this.smooth_edge_rejections++;
        this.candidate_count = 0;
        this.candidate_separated = false;
        this.candidate_kind = JunctionEvidence.JUNCTION_EVIDENCE_NONE;
        this.candidate_sides = 0;
        this.state = RouteState.ROUTE_FOLLOW;
        return;
      }

      this.candidate_reject = JunctionReject.JUNCTION_REJECT_NONE;

      if (this.state === RouteState.ROUTE_CANDIDATE && !(this.candidate_sides & sides)) {
        this.candidate_restarts++;
        this.candidate_reject = JunctionReject.JUNCTION_REJECT_SIDE_CHANGE;
        this.candidate_count = 0;
        this.candidate_separated = false;
        this.state = RouteState.ROUTE_FOLLOW;
        this.candidate_kind = JunctionEvidence.JUNCTION_EVIDENCE_NONE;
        this.candidate_sides = 0;
        if (!recent_center) return;
      }

      const same_sides =
        this.state === RouteState.ROUTE_CANDIDATE && this.candidate_sides === sides;

      if (this.state === RouteState.ROUTE_FOLLOW) {
        this.state = RouteState.ROUTE_CANDIDATE;
        this.phase_ms = now;
        this.entry_cm = d;
        this.entry_heading = h;
        this.candidate_count = 0;
        this.candidate_kind = kind;
        this.candidate_sides = sides;
      } else {
        this.candidate_sides &= sides;
        if (kind !== JunctionEvidence.JUNCTION_EVIDENCE_CENTER_BRANCH) {
          this.candidate_kind = kind;
        }
      }

      if (now - this.phase_ms >= JUNCTION_CONFIRM_TIMEOUT_MS) {
        this.fail(RouteFailure.ROUTE_FAILURE_CONFIRM_TIMEOUT);
        return;
      }

      const weak =
        this.candidate_kind !== JunctionEvidence.JUNCTION_EVIDENCE_CENTER_BRANCH &&
        !this.candidate_separated;
      const early =
        same_sides &&
        this.candidate_kind === JunctionEvidence.JUNCTION_EVIDENCE_CENTER_BRANCH &&
        this.targetReady(d, true) &&
        ((s.action === Action.LEFT && sides === 1) || (s.action === Action.RIGHT && sides === 2));

      const samples = early
        ? JUNCTION_EARLY_CONFIRM_SAMPLES
        : weak
        ? JUNCTION_EDGE_CONFIRM_SAMPLES
        : JUNCTION_CONFIRM_SAMPLES;

      const distance = early
        ? JUNCTION_EARLY_CONFIRM_CM
        : weak
        ? JUNCTION_EDGE_CONFIRM_CM
        : JUNCTION_CONFIRM_DISTANCE_CM;

      if (this.candidate_count < samples) this.candidate_count++;
      if (this.candidate_count < samples || d - this.entry_cm < distance) return;

      this.last_confirmed_kind = this.candidate_kind;
      this.junctions_seen++;
      this.passing_junction = !this.targetReady(d, false);
      this.search_pending = false;

      if (this.passing_junction) {
        this.beginAction(d, h, now);
        this.enterClear(d, now);
        return;
      }

      if (this.currentAction() !== Action.STRAIGHT && !this.entrySpeedReady()) {
        this.state = RouteState.ROUTE_BRAKING;
        this.phase_ms = now;
        this.entry_cm = d;
        this.entry_heading = h;
        return;
      }

      this.acceptTarget(d, h, now);
      return;
    }

    if (this.state === RouteState.ROUTE_BRAKING) {
      if (
        now - this.phase_ms >= JUNCTION_ENTRY_BRAKE_TIMEOUT_MS ||
        Math.abs(d - this.entry_cm) > JUNCTION_ENTRY_BRAKE_MAX_CM ||
        Math.abs(h - this.entry_heading) > JUNCTION_WRONG_WAY_RAD
      ) {
        this.fail(RouteFailure.ROUTE_FAILURE_ENTRY_SPEED);
        return;
      }
      if (this.entrySpeedReady()) {
        if (o.line_mask === 0) {
          this.fail(RouteFailure.ROUTE_FAILURE_ENTRY_SPEED);
          return;
        }
        this.acceptTarget(d, h, now);
      }
      return;
    }

    if (s.action === Action.HEXAGON && !this.passing_junction && this.state === RouteState.ROUTE_EXECUTING) {
      this.hexagonUpdate(o, d, h, now, adjacent);
      return;
    }

    if (this.state === RouteState.ROUTE_EXECUTING && this.turn_approaching) {
      if (now - this.phase_ms >= JUNCTION_APPROACH_TIMEOUT_MS) {
        this.fail(RouteFailure.ROUTE_FAILURE_ACTION_TIMEOUT); return;
      }
      // The front bar sees a junction L ahead of the axle. A pivot of radius
      // track/2 must start L-track/2 later, not at the detection point.
      if (d - this.turn_approach_origin_cm < GRAY_FORWARD_OFFSET_CM - WHEEL_TRACK_CM * .5) return;
      this.turn_approaching = false;
      this.entry_heading = this.action_start_heading = h;
      this.phase_ms = now;
      this.entry_cm = d;
      this.resetEvidence(now);
    }
    const action = this.currentAction();
    const sign = action === Action.LEFT ? -1.0 : 1.0;
    const turn = (h - this.entry_heading) * sign;
    const target = Math.abs(this.turnTargetRad());

    if (
      action !== Action.STRAIGHT &&
      (turn < -JUNCTION_WRONG_WAY_RAD ||
        turn > this.turnThresholdRad(JUNCTION_TURN_MAX_RAD))
    ) {
      this.fail(
        turn < 0
          ? RouteFailure.ROUTE_FAILURE_WRONG_WAY
          : RouteFailure.ROUTE_FAILURE_TURN_LIMIT
      );
      return;
    }

    if (this.state === RouteState.ROUTE_EXECUTING) {
      if (now - this.phase_ms >= this.actionTimeoutMs()) {
        this.fail(RouteFailure.ROUTE_FAILURE_ACTION_TIMEOUT);
        return;
      }

      if (action === Action.STRAIGHT) {
        const exit = routeIsClearLine(this, o);
        if (!exit || !adjacent) this.exit_count = 0;
        if (exit && this.exit_count < LINE_CONFIRM_SAMPLES) this.exit_count++;
        if (this.exit_count >= LINE_CONFIRM_SAMPLES) {
          this.line_exit_confirmed = true;
          this.line_exit_heading_rad = h;
          this.enterClear(d, now);
          if (!this.passing_junction) this.anchorNextSegment(d, now);
        }
        return;
      }

      const narrow = routeIsNarrowLine(o);
      if (
        this.componentRecovery(
          o,
          d,
          h,
          now,
          adjacent,
          !narrow &&
            this.capture_line_seen &&
            turn >= target * (1.0 - JUNCTION_EXIT_ANGLE_MARGIN_RATIO)
        )
      ) {
        return;
      }

      this.turn_capturing =
        narrow && turn >= this.turnThresholdRad(JUNCTION_CAPTURE_MIN_RAD);
      if (this.turn_capturing) {
        if (!adjacent) this.capture_line_samples = 0;
        if (this.capture_line_samples < LINE_CONFIRM_SAMPLES) this.capture_line_samples++;
        if (!this.capture_line_seen || this.capture_line_samples >= LINE_CONFIRM_SAMPLES) {
          this.capture_line_ms = now;
          this.capture_line_cm = d;
        }
        this.capture_line_seen = true;
      } else {
        this.capture_line_samples = 0;
      }

      if (turn >= target) this.corner_arc_complete = true;

      const ordinary_exit =
        s.action !== Action.CUSTOM_TURN ||
        Math.abs(Math.abs(this.turnTargetRad()) - JUNCTION_TURN_TARGET_RAD) < 0.0001;
      const angle_full = turn >= target;
      const line_ready =
        ordinary_exit && turn >= target - JUNCTION_EXIT_ALIGNMENT_RAD &&
        this.turn_capturing &&
        o.white_count <= JUNCTION_STRAIGHT_EXIT_MAX_WIDTH;
      const exit =
        narrow &&
        Math.abs(o.position) <= JUNCTION_CENTER_LIMIT &&
        (line_ready || angle_full || turn >= target - JUNCTION_EXIT_ANGLE_MARGIN_RATIO * target);

      if (!exit || !adjacent) {
        this.exit_count = 0;
        this.exit_candidate_pending = false;
      }
      if (exit && !this.exit_candidate_pending) {
        this.exit_candidate_pending = true;
        this.exit_candidate_ms = now;
        this.exit_candidate_cm = d;
      }
      if (exit && this.exit_count < LINE_CONFIRM_SAMPLES) this.exit_count++;

      if (
        this.exit_count >= LINE_CONFIRM_SAMPLES &&
        (line_ready ||
          angle_full ||
          (now - this.exit_candidate_ms >= 2 * JUNCTION_EXIT_CONFIRM_MS &&
            d - this.exit_candidate_cm >= 2.0 * JUNCTION_EXIT_CONFIRM_CM))
      ) {
        this.line_confirmed_exit_used = line_ready && !angle_full;
        this.line_exit_confirmed = true;
        this.line_exit_heading_rad = h;
        this.enterClear(d, now);
      }
      return;
    }

    if (
      now - this.phase_ms >= JUNCTION_CLEAR_TIMEOUT_MS ||
      d - this.clear_cm > JUNCTION_CLEAR_MAX_CM
    ) {
      this.fail(RouteFailure.ROUTE_FAILURE_ACTION_TIMEOUT);
      return;
    }

    const straight = this.currentAction() === Action.STRAIGHT;

    if (!straight && this.line_exit_confirmed && !this.passing_junction) {
      const corridor =
        this.exit_candidate_pending &&
        this.approach_center_count >= LINE_CONFIRM_SAMPLES &&
        now - this.exit_candidate_ms >= JUNCTION_ADJACENT_EXIT_MS &&
        d - this.exit_candidate_cm >= JUNCTION_ADJACENT_EXIT_CM;
      const sides = separatedSides(o);
      const branch = sides !== 0;

      if (this.next_junction_samples > 0 && !(this.next_junction_sides & sides)) {
        this.next_junction_samples = 0;
        this.candidate_reject = JunctionReject.JUNCTION_REJECT_SIDE_CHANGE;
        this.candidate_restarts++;
      }

      if (adjacent && branch && (corridor || this.next_junction_samples > 0)) {
        if (this.next_junction_samples === 0) {
          this.next_junction_ms = now;
          this.next_junction_cm = d;
          this.next_junction_sides = sides;
        }
        this.next_junction_sides &= sides;

        if (now - this.next_junction_ms >= JUNCTION_CONFIRM_TIMEOUT_MS) {
          this.fail(RouteFailure.ROUTE_FAILURE_CONFIRM_TIMEOUT);
          return;
        }

        if (this.next_junction_samples < JUNCTION_CONFIRM_SAMPLES) this.next_junction_samples++;
        if (
          this.next_junction_samples >= JUNCTION_CONFIRM_SAMPLES &&
          d - this.next_junction_cm >= JUNCTION_CONFIRM_DISTANCE_CM
        ) {
          const candidate_ms = this.next_junction_ms;
          const candidate_cm = this.next_junction_cm;
          const confirmed_sides = this.next_junction_sides;
          if (!this.completeClear(d, now)) return;
          this.state = RouteState.ROUTE_CANDIDATE;
          this.phase_ms = candidate_ms;
          this.entry_cm = candidate_cm;
          this.candidate_count = JUNCTION_CONFIRM_SAMPLES;
          this.candidate_separated = true;
          this.candidate_kind = JunctionEvidence.JUNCTION_EVIDENCE_SEPARATED;
          this.candidate_sides = confirmed_sides;
          this.update(o, d, h, now);
          return;
        }
      } else {
        this.next_junction_samples = 0;
      }

      if (adjacent && routeIsNarrowLine(o) && Math.abs(o.position) <= JUNCTION_CENTER_LIMIT) {
        if (this.approach_center_count < LINE_CONFIRM_SAMPLES) this.approach_center_count++;
      } else {
        this.approach_center_count = 0;
      }
    }

    if (
      this.next_junction_samples === 0 &&
      this.componentRecovery(
        o,
        d,
        h,
        now,
        adjacent,
        straight || (this.line_exit_confirmed && !routeIsNarrowLine(o))
      )
    ) {
      return;
    }

    const clear = routeIsClearLine(this, o);
    if (!clear || !adjacent) {
      this.exit_count = 0;
      this.exit_candidate_pending = false;
    }
    if (clear && !this.exit_candidate_pending) {
      this.exit_candidate_pending = true;
      this.exit_candidate_ms = now;
      this.exit_candidate_cm = d;
    }
    if (clear && this.exit_count < LINE_CONFIRM_SAMPLES) this.exit_count++;

    if (straight && clear && this.exit_count >= LINE_CONFIRM_SAMPLES) {
      this.capture_line_ms = now;
      this.capture_line_cm = d;
      if (!this.passing_junction && !this.next_segment_origin_valid) {
        this.anchorNextSegment(d, now);
        this.refreshDistance(d, now);
      }
    }

    if (
      this.exit_count < LINE_CONFIRM_SAMPLES ||
      now - this.exit_candidate_ms < JUNCTION_COUNT_CLEAR_MS ||
      d - this.exit_candidate_cm < JUNCTION_COUNT_CLEAR_CM
    ) {
      return;
    }

    const centered_exit =
      Math.abs(o.position) <= JUNCTION_CENTER_LIMIT &&
      o.white_count <= JUNCTION_STRAIGHT_EXIT_MAX_WIDTH &&
      (straight || this.approach_center_count >= LINE_CONFIRM_SAMPLES);

    if (this.completeClear(d, now) && centered_exit) {
      this.have_approach_center = true;
      this.approach_center_count = LINE_CONFIRM_SAMPLES;
      this.approach_center_ms = now;
      this.approach_center_cm = d;
    }
  }

  public dispatchStopCallback(): void {
    if (!this.stop_pending || this.callback_dispatched) return;
    this.stop_pending = false;
    this.callback_dispatched = true;
    if (this.stop_callback) this.stop_callback(this.index);
  }
}
