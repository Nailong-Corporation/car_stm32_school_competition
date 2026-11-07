import { LineFollowModel } from './line_follow';
import { graySensorUpdate, GraySensorModel } from './gray_sensor';
import {
  START_POINT,
  START_HEADING_RAD,
  isSensorOnTrackLine,
  TRACK_LINE_WIDTH_CM,
  type Point2D,
} from '../track/map_data';
import {
  CONTROL_PERIOD_S,
  LINE_CONFIRM_SAMPLES,
  GRAY_FORWARD_OFFSET_CM,
  WHEEL_TRACK_CM,
  VEHICLE_MAX_SPEED_CM_S,
} from './config';
import { LineState, type EncoderCounts, type GrayObservation } from './types';

export interface CarPose {
  x: number;          // cm
  y: number;          // cm
  heading: number;    // rad (standard 2D: 0 = +x East, PI/2 = +y South, -PI/2 = -y North)
}

export interface SensorPoint {
  index: number;
  x: number;
  y: number;
  active: boolean;
}

export class VehicleSimulator {
  public pose: CarPose = {
    x: START_POINT.x,
    y: START_POINT.y,
    heading: START_HEADING_RAD,
  };

  public actual_left_speed = 0;   // cm/s
  public actual_right_speed = 0;  // cm/s
  public encoder_counts: EncoderCounts = { left: 0, right: 0 };
  private left_tick_accum = 0;
  private right_tick_accum = 0;
  private startupSamplesRemaining = 0;
  public raw_sensor_mask = 0;
  public sensor_points: SensorPoint[] = [];

  public follow: LineFollowModel;
  public gray_sensor: GraySensorModel = new GraySensorModel();

  public sensor_pitch_cm = 1.1; // Distance between adjacent sensor channels (~7.7 cm total span)
  public sim_time_s = 0;
  public sim_time_ms = 0;
  public trajectory: Point2D[] = [];
  public max_trajectory_points = 5000;

  constructor(follow: LineFollowModel) {
    this.follow = follow;
    this.reset();
  }

  public reset(pose?: Partial<CarPose>): void {
    this.pose = {
      x: pose?.x ?? START_POINT.x,
      y: pose?.y ?? START_POINT.y,
      heading: pose?.heading ?? START_HEADING_RAD,
    };
    this.actual_left_speed = 0;
    this.actual_right_speed = 0;
    this.left_tick_accum = 0;
    this.right_tick_accum = 0;
    this.startupSamplesRemaining = 0;
    this.encoder_counts = { left: 0, right: 0 };
    this.raw_sensor_mask = 0;
    this.sim_time_s = 0;
    this.sim_time_ms = 0;
    this.trajectory = [{ x: this.pose.x, y: this.pose.y }];
    this.gray_sensor.reset();
    this.updateSensors();
  }

  public updateSensors(): number {
    const fx = Math.cos(this.pose.heading);
    const fy = Math.sin(this.pose.heading);

    // Front sensor array center
    const cx = this.pose.x + GRAY_FORWARD_OFFSET_CM * fx;
    const cy = this.pose.y + GRAY_FORWARD_OFFSET_CM * fy;

    // Lateral rightward vector (perpendicular to heading)
    // If heading = -PI/2 (North: fx=0, fy=-1), rightward is East (lx=1, ly=0)
    const lx = -fy;
    const ly = fx;

    let mask = 0;
    this.sensor_points = [];

    for (let i = 0; i < 8; ++i) {
      // S0 is leftmost (-3.5), S7 is rightmost (+3.5)
      const offset = (i - 3.5) * this.sensor_pitch_cm;
      const sx = cx + offset * lx;
      const sy = cy + offset * ly;

      const onLine = isSensorOnTrackLine(sx, sy, TRACK_LINE_WIDTH_CM / 2);
      if (onLine) {
        mask |= 1 << i;
      }

      this.sensor_points.push({
        index: i,
        x: sx,
        y: sy,
        active: onLine,
      });
    }

    this.raw_sensor_mask = mask;
    return mask;
  }

  /** One user START: sample the line first, then submit a single controller request. */
  public requestStart(): void {
    if (this.follow.state === LineState.LINE_IDLE && this.startupSamplesRemaining === 0) {
      this.startupSamplesRemaining = LINE_CONFIRM_SAMPLES;
    }
  }

  public step(dt = CONTROL_PERIOD_S, start_request = false): void {
    this.sim_time_s += dt;
    this.sim_time_ms += Math.round(dt * 1000);

    // 1. Sample track under the optical sensor bar
    const raw_mask = this.updateSensors();

    // 2. Filter & classify gray sensors through STM32 algorithm
    const obs: GrayObservation = graySensorUpdate(
      this.gray_sensor,
      raw_mask,
      this.follow.have_reliable,
      this.follow.reliable_position
    );

    // 3. Compute relative heading for STM32 algorithm (0 at start, clockwise positive)
    const relative_heading_rad = this.pose.heading - START_HEADING_RAD;

    // The idle browser has no control ticks. Warm up real observations before START;
    // keep the controller's stable-line guard and never retry a rejected request forever.
    if (start_request) this.startupSamplesRemaining = 0;
    if (this.startupSamplesRemaining > 0) {
      this.startupSamplesRemaining--;
      start_request = this.startupSamplesRemaining === 0;
    }

    // 4. Run STM32 Line Follow & Control loop
    this.follow.update(
      obs,
      this.encoder_counts,
      this.sim_time_ms,
      dt,
      start_request,
      true,
      relative_heading_rad
    );

    // 5. Convert motor PWM to physical response
    const cmd = this.follow.command;
    const target_v_left = cmd.standby ? (cmd.left_pwm / 255.0) * VEHICLE_MAX_SPEED_CM_S : 0;
    const target_v_right = cmd.standby ? (cmd.right_pwm / 255.0) * VEHICLE_MAX_SPEED_CM_S : 0;

    // Motor mechanical time constant (~35 ms response)
    const motor_tau = 0.035;
    const alpha = dt / (motor_tau + dt);
    this.actual_left_speed += alpha * (target_v_left - this.actual_left_speed);
    this.actual_right_speed += alpha * (target_v_right - this.actual_right_speed);

    // 6. Update hardware encoder count registers
    const left_cm_per_count = this.follow.odometry.left_cm_per_count || 0.013;
    const right_cm_per_count = this.follow.odometry.right_cm_per_count || 0.013;

    this.left_tick_accum += (this.actual_left_speed * dt) / left_cm_per_count;
    this.right_tick_accum += (this.actual_right_speed * dt) / right_cm_per_count;

    const dl_ticks = Math.trunc(this.left_tick_accum) * this.follow.odometry.calibration.left_sign;
    const dr_ticks = Math.trunc(this.right_tick_accum) * this.follow.odometry.calibration.right_sign;

    this.left_tick_accum -= Math.trunc(this.left_tick_accum);
    this.right_tick_accum -= Math.trunc(this.right_tick_accum);

    this.encoder_counts.left = (this.encoder_counts.left + dl_ticks) & 0xffff;
    this.encoder_counts.right = (this.encoder_counts.right + dr_ticks) & 0xffff;

    // 7. Vehicle Kinematics integration (Differential drive)
    const v_linear = (this.actual_left_speed + this.actual_right_speed) * 0.5;
    // Right wheel faster -> turns left (counter-clockwise, -dheading)
    // Left wheel faster -> turns right (clockwise, +dheading)
    const omega = (this.actual_left_speed - this.actual_right_speed) / WHEEL_TRACK_CM;

    this.pose.heading += omega * dt;
    this.pose.x += v_linear * Math.cos(this.pose.heading) * dt;
    this.pose.y += v_linear * Math.sin(this.pose.heading) * dt;

    // Record trajectory
    const lastP = this.trajectory[this.trajectory.length - 1];
    if (!lastP || Math.hypot(this.pose.x - lastP.x, this.pose.y - lastP.y) >= 0.5) {
      this.trajectory.push({ x: this.pose.x, y: this.pose.y });
      if (this.trajectory.length > this.max_trajectory_points) {
        this.trajectory.shift();
      }
    }
  }
}
