import {
  LineState,
  LineStopReason,
  Action,
  RouteState,
  RouteFailure,
  HexagonState,
  GrayKind,
  type RouteStep,
  type GrayObservation,
  type EncoderCounts,
  type MotorCommand,
  HeadingSource,
} from './types';
import {
  WHEEL_TARGET_MAX_CM_S,
  STARTUP_RAMP_TIME_S,
  TARGET_RISE_CM_S2,
  TURN_EXIT_ACCEL_CM_S2,
  LINE_FOLLOW_SPEED_CM_S,
  HOLD_SPEED_CM_S,
  AMBIGUOUS_SPEED_CM_S,
  HEADING_KP_CM_S_PER_RAD,
  POSITION_KP_CM_S,
  POSITION_KI_CM_S,
  POSITION_KD_CM_S,
  PID_INTEGRAL_LIMIT,
  PID_DERIVATIVE_TAU_S,
  POSITION_CURVE_SLOWDOWN,
  LINE_CONFIRM_SAMPLES,
  INTERFERENCE_TIMEOUT_MS,
  INTERFERENCE_MAX_DISTANCE_CM,
  BRAKING_SAFE_DECEL_CM_S2,
  BRAKING_DISTANCE_MARGIN_CM,
  BRAKING_TARGET_JERK_CM_S3,
  JUNCTION_PASS_SPEED_CM_S,
  JUNCTION_SETTLE_BAND_RAD,
  JUNCTION_SETTLE_SPEED_CM_S,
  JUNCTION_SETTLE_HEADING_KP,
  JUNCTION_CAPTURE_HEADING_KP,
  JUNCTION_CAPTURE_CENTER_BIAS_CM_S,
  JUNCTION_CAPTURE_MIN_TURN_CM_S,
  JUNCTION_YAW_DAMPING,
  JUNCTION_YAW_FILTER_TAU_S,
  JUNCTION_EXIT_SPEED_CM_S,
  COMPONENT_RECOVERY_SPEED_CM_S,
  HEXAGON_SPEED_CM_S,
  NAVIGATION_RECOVERY_SPEED_CM_S,
  CORNER_RADIUS_CM,
  CORNER_HEADING_KP,
  GRAY_FORWARD_OFFSET_CM,
} from './config';
import { OdometryModel, defaultOdometryCalibration } from './odometry';
import { RoutePlannerModel, routeIsNarrowLine, routeIsClearLine } from './route';
import { SpeedControlModel } from './speed_control';
import { Pid } from './pid';
import { BrakingSlewModel, brakingPlan, brakingDecide, brakingSlew } from './braking_planner';

export class LineFollowModel {
  public state: LineState = LineState.LINE_IDLE;
  public stop_reason: LineStopReason = LineStopReason.LINE_STOP_NONE;
  public heading_source: HeadingSource = HeadingSource.HEADING_IMU;

  public odometry: OdometryModel;
  public route: RoutePlannerModel;
  public speed: SpeedControlModel;
  public braking_slew: BrakingSlewModel;
  public pid: Pid;
  public command: MotorCommand = {
    left_pwm: 0,
    right_pwm: 0,
    left_forward: true,
    right_forward: true,
    standby: false,
  };

  public braking_active = false;
  public braking_reverse_active = false;
  public have_reliable = false;
  public abnormal = false;
  public valid_count = 0;
  public reliable_position = 0;
  public base = 0;
  public correction = 0;
  public left_target_cm_s = 0;
  public right_target_cm_s = 0;
  public hold_heading_rad = 0;
  public abnormal_origin_cm = 0;
  public turn_yaw_rate_rad_s = 0;
  public exit_accelerating = false;
  public startup_elapsed_s = 0;
  public last_reliable_ms = 0;
  public sample_ms = 0;
  public observation: GrayObservation = {
    kind: GrayKind.GRAY_NONE,
    raw_mask: 0,
    line_mask: 0,
    selected_mask: 0,
    white_count: 0,
    region_count: 0,
    position: 0,
    prefilter_mask: 0,
    seam_mask: 0,
  };

  constructor(steps: RouteStep[] = []) {
    this.odometry = new OdometryModel(defaultOdometryCalibration());
    this.route = new RoutePlannerModel();
    this.speed = new SpeedControlModel();
    this.braking_slew = new BrakingSlewModel();
    this.pid = new Pid({
      kp: POSITION_KP_CM_S,
      ki: POSITION_KI_CM_S,
      kd: POSITION_KD_CM_S,
      integral_limit: PID_INTEGRAL_LIMIT,
      derivative_tau: PID_DERIVATIVE_TAU_S,
    });
    this.init(steps);
  }

  public init(steps: RouteStep[], callback?: (step_index: number) => void): void {
    this.state = LineState.LINE_IDLE;
    this.stop_reason = LineStopReason.LINE_STOP_NONE;
    this.heading_source = HeadingSource.HEADING_IMU;
    this.odometry.init(defaultOdometryCalibration());
    this.speed.reset();
    this.braking_slew.reset();
    this.pid.reset();
    this.route.init(steps, callback);

    this.braking_active = false;
    this.braking_reverse_active = false;
    this.have_reliable = false;
    this.abnormal = false;
    this.valid_count = 0;
    this.reliable_position = 0;
    this.base = 0;
    this.correction = 0;
    this.left_target_cm_s = 0;
    this.right_target_cm_s = 0;
    this.hold_heading_rad = 0;
    this.abnormal_origin_cm = 0;
    this.turn_yaw_rate_rad_s = 0;
    this.exit_accelerating = false;
    this.startup_elapsed_s = 0;
    this.last_reliable_ms = 0;
    this.sample_ms = 0;
    this.command = {
      left_pwm: 0,
      right_pwm: 0,
      left_forward: true,
      right_forward: true,
      standby: false,
    };
  }

  public stop(reason: LineStopReason): void {
    this.state = LineState.LINE_STOPPED;
    this.stop_reason = reason;
    this.left_target_cm_s = 0;
    this.right_target_cm_s = 0;
    this.base = 0;
    this.correction = 0;
    this.pid.reset();
    this.speed.reset();
    this.braking_slew.reset();
    this.command = {
      left_pwm: 0,
      right_pwm: 0,
      left_forward: true,
      right_forward: true,
      standby: false,
    };
  }

  private nextOuter(base: number, dt: number): number {
    const previous = Math.max(this.left_target_cm_s, this.right_target_cm_s);
    const rise = this.exit_accelerating ? TURN_EXIT_ACCEL_CM_S2 : TARGET_RISE_CM_S2;
    return Math.min(base, previous + rise * dt);
  }

  private targets(base: number, correction: number, dt: number): void {
    let next_base = base;
    if (this.startup_elapsed_s < STARTUP_RAMP_TIME_S) {
      const startup_cap = (base * this.startup_elapsed_s) / STARTUP_RAMP_TIME_S;
      next_base = Math.min(next_base, startup_cap);
    }
    const previous_outer = Math.max(this.left_target_cm_s, this.right_target_cm_s);
    const rise = this.exit_accelerating ? TURN_EXIT_ACCEL_CM_S2 : TARGET_RISE_CM_S2;
    const outer_cap = previous_outer + rise * dt;
    const requested_outer = next_base + Math.abs(correction);
    if (requested_outer > outer_cap && requested_outer > 0) {
      const scale = outer_cap / requested_outer;
      next_base *= scale;
      correction *= scale;
    }
    this.base = next_base;
    this.correction = correction;
    this.left_target_cm_s = Math.max(0, Math.min(WHEEL_TARGET_MAX_CM_S, next_base + correction));
    this.right_target_cm_s = Math.max(0, Math.min(WHEEL_TARGET_MAX_CM_S, next_base - correction));
  }

  private outerTargets(outer: number, correction: number, dt: number): void {
    let capped_outer = outer;
    if (this.startup_elapsed_s < STARTUP_RAMP_TIME_S) {
      const startup_cap = (outer * this.startup_elapsed_s) / STARTUP_RAMP_TIME_S;
      capped_outer = Math.min(capped_outer, startup_cap);
    }
    capped_outer = this.nextOuter(capped_outer, dt);
    const half = Math.min(capped_outer * 0.5, Math.abs(correction));
    const base = capped_outer - half;
    const signed_corr = correction >= 0 ? half : -half;
    this.base = base;
    this.correction = signed_corr;
    this.left_target_cm_s = Math.max(0, Math.min(WHEEL_TARGET_MAX_CM_S, base + signed_corr));
    this.right_target_cm_s = Math.max(0, Math.min(WHEEL_TARGET_MAX_CM_S, base - signed_corr));
  }

  private turnTargets(outer: number, correction: number, dt: number): void {
    this.outerTargets(outer, correction, dt);
  }

  private straightTargets(base: number, correction: number, dt: number): void {
    this.targets(base, correction, dt);
  }

  public update(
    observation: GrayObservation,
    counts: EncoderCounts,
    now_ms: number,
    dt: number,
    start_request: boolean,
    imu_valid = true,
    imu_heading_rad = 0
  ): void {
    this.sample_ms = now_ms;
    this.observation = observation;

    const previous_heading = this.odometry.heading_rad;
    const previous_route = this.route.state;

    if (!this.odometry.updateWithHeading(counts, dt, imu_valid, imu_heading_rad)) {
      this.stop(LineStopReason.LINE_STOP_ENCODER);
      return;
    }

    if (dt > 0) {
      const yaw_rate = (this.odometry.heading_rad - previous_heading) / dt;
      const alpha = dt / (JUNCTION_YAW_FILTER_TAU_S + dt);
      this.turn_yaw_rate_rad_s += alpha * (yaw_rate - this.turn_yaw_rate_rad_s);
    }

    const obs = observation;
    const narrow = routeIsNarrowLine(obs);

    if (narrow) {
      if (this.valid_count < LINE_CONFIRM_SAMPLES) this.valid_count++;
    } else {
      this.valid_count = 0;
    }
    const stable = this.valid_count >= LINE_CONFIRM_SAMPLES;

    if (this.state === LineState.LINE_IDLE || this.state === LineState.LINE_STOPPED) {
      if (start_request) {
        if (!stable) {
          return;
        }
        this.state = LineState.LINE_TRACKING;
        this.stop_reason = LineStopReason.LINE_STOP_NONE;
        this.startup_elapsed_s = 0;
        this.have_reliable = true;
        this.last_reliable_ms = now_ms;
        this.reliable_position = obs.position;
        this.route.start(this.odometry.distance_cm, now_ms);
      } else {
        return;
      }
    }

    this.startup_elapsed_s += dt;

    this.route.setSpeed(this.odometry.left_speed_cm_s, this.odometry.right_speed_cm_s);
    this.route.update(obs, this.odometry.distance_cm, this.odometry.heading_rad, now_ms);

    if (this.route.state === RouteState.ROUTE_FINISHED) {
      this.route.dispatchStopCallback();
      this.stop(LineStopReason.LINE_STOP_ROUTE_COMPLETE);
      return;
    }

    if (this.route.state === RouteState.ROUTE_FAILED) {
      if (!this.route.recoverNavigation(this.odometry.distance_cm, this.odometry.heading_rad, now_ms)) {
        this.stop(LineStopReason.LINE_STOP_ROUTE);
        return;
      }
    }

    const planning_step = this.route.steps[this.route.planningIndex()];
    const step = this.route.steps[this.route.index];
    const cruise = this.route.cruiseSpeed();
    const planning_origin = this.route.planningOrigin();
    const planning_entry_speed = this.route.planningEntrySpeedLimit();
    const entry_speed_limit = this.route.entrySpeedLimit();
    let base_speed = cruise;
    let navigation_wheel_cap = WHEEL_TARGET_MAX_CM_S;

    const short_turn =
      planning_step.segment_length_cm <= 100.0 &&
      (planning_step.action === Action.LEFT ||
        planning_step.action === Action.RIGHT ||
        planning_step.action === Action.CUSTOM_TURN ||
        planning_step.action === Action.HEXAGON);

    if (
      short_turn &&
      (this.route.state === RouteState.ROUTE_FOLLOW ||
        this.route.state === RouteState.ROUTE_CANDIDATE ||
        this.route.state === RouteState.ROUTE_CLEARING)
    ) {
      base_speed = Math.min(base_speed, planning_entry_speed);
      navigation_wheel_cap = planning_entry_speed;
    }

    const passing_without_exit =
      (this.route.passing_junction &&
        this.route.state === RouteState.ROUTE_CLEARING &&
        (this.route.component_recovery_active ||
          !stable ||
          !routeIsClearLine(this.route, obs) ||
          this.route.exit_count < LINE_CONFIRM_SAMPLES)) ||
      (this.route.state === RouteState.ROUTE_CANDIDATE &&
        !this.route.targetReady(this.odometry.distance_cm, true) &&
        (!narrow || Math.abs(obs.position) > 2.0));

    if (passing_without_exit) {
      base_speed = Math.min(base_speed, COMPONENT_RECOVERY_SPEED_CM_S);
      navigation_wheel_cap = Math.min(navigation_wheel_cap, COMPONENT_RECOVERY_SPEED_CM_S);
    }

    if (
      planning_step.segment_length_cm > 0 &&
      this.route.final_segment_active &&
      (this.route.state === RouteState.ROUTE_FOLLOW ||
        this.route.state === RouteState.ROUTE_CANDIDATE ||
        this.route.state === RouteState.ROUTE_CLEARING)
    ) {
      const target_dist = Math.max(0, planning_step.segment_length_cm - GRAY_FORWARD_OFFSET_CM);
      const plan = brakingPlan(
        base_speed,
        Math.min(base_speed, planning_entry_speed),
        target_dist,
        this.odometry.distance_cm - planning_origin,
        Math.max(Math.abs(this.odometry.left_speed_cm_s), Math.abs(this.odometry.right_speed_cm_s)),
        BRAKING_SAFE_DECEL_CM_S2,
        BRAKING_DISTANCE_MARGIN_CM,
        true
      );
      if (!plan.valid) {
        this.stop(LineStopReason.LINE_STOP_ROUTE);
        return;
      }
      navigation_wheel_cap = Math.min(navigation_wheel_cap, plan.allowed_speed);
      this.braking_active = plan.valid && plan.allowed_speed < cruise;
      base_speed = brakingSlew(
        this.braking_slew,
        plan.allowed_speed,
        dt,
        TARGET_RISE_CM_S2,
        BRAKING_TARGET_JERK_CM_S3
      );
    } else {
      this.braking_active = false;
      this.braking_reverse_active = false;
    }

    if (this.route.search_pending && planning_step.segment_length_cm > 0) {
      base_speed = Math.min(base_speed, planning_entry_speed);
    }

    if (this.route.state === RouteState.ROUTE_CLEARING && this.route.currentAction() !== Action.STRAIGHT) {
      base_speed = Math.min(base_speed, Math.min(step.turn_speed, JUNCTION_EXIT_SPEED_CM_S));
    }

    if (this.route.state === RouteState.ROUTE_CANDIDATE && this.route.targetReady(this.odometry.distance_cm, true)) {
      base_speed = Math.min(base_speed, entry_speed_limit);
    }

    if (this.route.state === RouteState.ROUTE_BRAKING) {
      this.abnormal = false;
      this.pid.reset();
      this.targets(0, 0, dt);
      this.state = LineState.LINE_JUNCTION;
      this.speed.update(0, 0, this.odometry.left_speed_cm_s, this.odometry.right_speed_cm_s, dt);
      this.command = { left_pwm: 0, right_pwm: 0, left_forward: true, right_forward: true, standby: false };
      return;
    }

    if (this.route.state === RouteState.ROUTE_CANDIDATE && !this.route.targetReady(this.odometry.distance_cm, true)) {
      this.state = LineState.LINE_JUNCTION;
      this.abnormal = false;
      this.pid.reset();
      this.outerTargets(
        base_speed,
        HEADING_KP_CM_S_PER_RAD * (this.route.entry_heading - this.odometry.heading_rad),
        dt
      );
    } else if (this.route.state === RouteState.ROUTE_RECOVERING) {
      this.state = LineState.LINE_JUNCTION;
      this.abnormal = false;
      if (narrow && stable) {
        const res = this.pid.update(obs.position, dt, NAVIGATION_RECOVERY_SPEED_CM_S * 0.5);
        this.outerTargets(NAVIGATION_RECOVERY_SPEED_CM_S, res.output, dt);
      } else {
        this.pid.reset();
        this.outerTargets(
          NAVIGATION_RECOVERY_SPEED_CM_S,
          HEADING_KP_CM_S_PER_RAD * (this.route.entry_heading - this.odometry.heading_rad) -
            JUNCTION_YAW_DAMPING * this.turn_yaw_rate_rad_s,
          dt
        );
      }
    } else if (this.route.component_recovery_active) {
      this.state = LineState.LINE_JUNCTION;
      this.abnormal = false;
      this.exit_accelerating = false;
      this.pid.reset();
      this.outerTargets(
        this.route.passing_junction ? base_speed : Math.min(step.turn_speed, COMPONENT_RECOVERY_SPEED_CM_S),
        HEADING_KP_CM_S_PER_RAD * (this.route.component_recovery_heading - this.odometry.heading_rad) -
          JUNCTION_YAW_DAMPING * this.turn_yaw_rate_rad_s,
        dt
      );
    } else if (this.route.state === RouteState.ROUTE_EXECUTING && step.action === Action.HEXAGON) {
      const x = this.route.hexagon;
      this.state = LineState.LINE_JUNCTION;
      this.abnormal = false;
      this.exit_accelerating = false;
      if (previous_route !== RouteState.ROUTE_EXECUTING) this.pid.reset();
      const outer = Math.min(step.turn_speed, Math.min(HEXAGON_SPEED_CM_S, JUNCTION_SETTLE_SPEED_CM_S));
      const direction = this.route.currentAction() === Action.LEFT ? -1.0 : 1.0;
      let correction = 0;
      if (x.use_line) {
        const res = this.pid.update(obs.position, dt, 0.5 * outer);
        correction = res.output;
      } else {
        this.pid.reset();
        const guide =
          x.state === HexagonState.HEXAGON_ENTERING ||
          (x.state === HexagonState.HEXAGON_FOLLOWING && obs.line_mask !== 0 && !x.line_samples);
        correction = guide
          ? direction * 0.5 * outer
          : HEADING_KP_CM_S_PER_RAD * (x.hold_heading - this.odometry.heading_rad);
      }
      correction -= JUNCTION_YAW_DAMPING * this.turn_yaw_rate_rad_s;
      this.outerTargets(outer, correction, dt);
    } else if (this.route.state === RouteState.ROUTE_EXECUTING) {
      this.exit_accelerating = false;
      if (previous_route !== RouteState.ROUTE_EXECUTING) {
        this.pid.reset();
        this.speed.reset();
      }
      this.abnormal = false;
      this.state = LineState.LINE_JUNCTION;
      const direction = this.route.currentAction() === Action.LEFT ? -1.0 : 1.0;
      const target = this.route.turnTargetRad();
      const turn = direction * (this.odometry.heading_rad - this.route.entry_heading);
      const outer =
        this.route.currentAction() === Action.STRAIGHT
          ? step.turn_speed
          : Math.min(step.turn_speed, JUNCTION_SETTLE_SPEED_CM_S);
      const remaining = Math.abs(target) - turn;
      const band = Math.min(JUNCTION_SETTLE_BAND_RAD, 0.4 * Math.abs(target));
      const settling = this.route.currentAction() !== Action.STRAIGHT && band > 0 && remaining <= band;
      const base = 0.5 * outer;
      let correction = direction * base;

      if (this.route.currentAction() === Action.STRAIGHT) {
        const pass = Math.min(base_speed, Math.max(outer, JUNCTION_PASS_SPEED_CM_S));
        this.straightTargets(
          pass,
          HEADING_KP_CM_S_PER_RAD * (this.route.entry_heading - this.odometry.heading_rad),
          dt
        );
      } else {
        if (
          this.route.usesCornerArc() &&
          !this.route.corner_arc_complete &&
          !settling &&
          !this.route.capture_line_seen
        ) {
          const track = this.odometry.calibration.track_cm;
          const arcBase = outer / (1 + track / (2 * CORNER_RADIUS_CM));
          const lead = GRAY_FORWARD_OFFSET_CM - CORNER_RADIUS_CM;
          const planned = Math.max(
            0,
            Math.min(
              Math.abs(target),
              (this.odometry.distance_cm - this.route.entry_cm - lead) / CORNER_RADIUS_CM
            )
          );
          correction =
            direction *
            ((arcBase * track) / (2 * CORNER_RADIUS_CM) + CORNER_HEADING_KP * (planned - turn));
        } else if (this.route.turn_capturing || this.route.capture_line_seen || settling) {
          let line = 0;
          if (this.route.turn_capturing) {
            const res = this.pid.update(obs.position, dt, base);
            line = res.output;
          } else {
            this.pid.reset();
          }
          const heading_kp = settling ? JUNCTION_SETTLE_HEADING_KP : JUNCTION_CAPTURE_HEADING_KP;
          correction = this.route.turn_capturing
            ? line
            : heading_kp * (this.route.entry_heading + target - this.odometry.heading_rad);
          if (
            this.route.turn_capturing &&
            Math.abs(obs.position) < 0.01 &&
            Math.abs(line) < 0.05 &&
            turn < Math.abs(target)
          ) {
            correction = direction * Math.min(base, JUNCTION_CAPTURE_CENTER_BIAS_CM_S);
          }
          if (turn < Math.abs(target)) {
            correction =
              direction *
              Math.max(direction * correction, Math.min(base, JUNCTION_CAPTURE_MIN_TURN_CM_S));
          }
          correction -= JUNCTION_YAW_DAMPING * this.turn_yaw_rate_rad_s;
        } else {
          this.pid.reset();
        }
        this.turnTargets(outer, correction, dt);
      }
    } else if (this.route.state === RouteState.ROUTE_CLEARING && this.route.next_junction_samples > 0) {
      this.state = LineState.LINE_JUNCTION;
      this.abnormal = false;
      this.pid.reset();
      this.outerTargets(
        Math.min(base_speed, JUNCTION_SETTLE_SPEED_CM_S),
        HEADING_KP_CM_S_PER_RAD * (this.route.line_exit_heading_rad - this.odometry.heading_rad),
        dt
      );
    } else if (this.route.state === RouteState.ROUTE_CLEARING && this.route.currentAction() === Action.STRAIGHT) {
      this.state = LineState.LINE_JUNCTION;
      this.abnormal = false;
      const pass = base_speed;
      if (routeIsClearLine(this.route, obs) && stable && this.route.exit_count >= LINE_CONFIRM_SAMPLES) {
        const res = this.pid.update(obs.position, dt, 0.5 * pass);
        this.outerTargets(pass, res.output, dt);
      } else {
        this.pid.reset();
        this.outerTargets(
          pass,
          HEADING_KP_CM_S_PER_RAD * (this.route.entry_heading - this.odometry.heading_rad),
          dt
        );
      }
    } else if (narrow && (stable || !this.abnormal) && this.route.state !== RouteState.ROUTE_CANDIDATE) {
      if (this.abnormal) {
        this.pid.reset();
        this.speed.reset();
        this.abnormal = false;
      }
      this.state = LineState.LINE_TRACKING;
      if (stable) {
        this.have_reliable = true;
        this.last_reliable_ms = now_ms;
        this.reliable_position = obs.position;
      }
      const base = base_speed * (1.0 - (POSITION_CURVE_SLOWDOWN * Math.abs(obs.position)) / 7.0);
      const next = this.nextOuter(base, dt);
      const res = this.pid.update(obs.position, dt, 0.5 * next);
      let correction = res.output;
      if (this.route.state === RouteState.ROUTE_CLEARING && this.route.currentAction() !== Action.STRAIGHT) {
        correction -= JUNCTION_YAW_DAMPING * this.turn_yaw_rate_rad_s;
      }
      this.outerTargets(base, correction, dt);
    } else {
      if (!this.abnormal) {
        this.abnormal = true;
        this.hold_heading_rad = this.odometry.heading_rad;
        if (
          this.route.state === RouteState.ROUTE_CLEARING &&
          this.route.line_exit_confirmed &&
          this.route.currentAction() !== Action.STRAIGHT
        ) {
          this.hold_heading_rad = this.route.line_exit_heading_rad;
        }
        this.abnormal_origin_cm = this.odometry.distance_cm;
      }
      this.state = LineState.LINE_GAP_HOLD;
      const base =
        obs.kind === GrayKind.GRAY_NONE
          ? Math.min(base_speed, HOLD_SPEED_CM_S)
          : Math.min(base_speed, AMBIGUOUS_SPEED_CM_S);
      let correction = HEADING_KP_CM_S_PER_RAD * (this.hold_heading_rad - this.odometry.heading_rad);
      if (
        this.route.state === RouteState.ROUTE_CLEARING &&
        this.route.line_exit_confirmed &&
        this.route.currentAction() !== Action.STRAIGHT
      ) {
        correction -= JUNCTION_YAW_DAMPING * this.turn_yaw_rate_rad_s;
        this.outerTargets(base, correction, dt);
      } else {
        this.targets(base, correction, dt);
      }
    }

    const peak_target = Math.max(this.left_target_cm_s, this.right_target_cm_s);
    if (peak_target > navigation_wheel_cap && peak_target > 0) {
      const ratio = navigation_wheel_cap / peak_target;
      this.left_target_cm_s *= ratio;
      this.right_target_cm_s *= ratio;
      this.base = 0.5 * (this.left_target_cm_s + this.right_target_cm_s);
      this.correction = 0.5 * (this.left_target_cm_s - this.right_target_cm_s);
    }

    const { success, command } = this.speed.update(
      this.left_target_cm_s,
      this.right_target_cm_s,
      this.odometry.left_speed_cm_s,
      this.odometry.right_speed_cm_s,
      dt
    );

    if (!success) {
      this.stop(LineStopReason.LINE_STOP_FAULT);
      return;
    }

    this.command = command;
  }
}
