import type { EncoderCounts, OdometryCalibration } from './types';
import {
  GRAY_FORWARD_OFFSET_CM,
  ODOMETRY_DISTANCE_SCALE,
  ODOMETRY_HEADING_SCALE,
  ODOMETRY_MAX_SPEED_CM_S,
  SPEED_FILTER_TAU_S,
  CONTROL_MAX_INTERVAL_MS,
  LEFT_WHEEL_DIAMETER_CM,
  RIGHT_WHEEL_DIAMETER_CM,
  LEFT_COUNTS_PER_WHEEL_REV,
  RIGHT_COUNTS_PER_WHEEL_REV,
  WHEEL_TRACK_CM,
} from './config';

export function defaultOdometryCalibration(): OdometryCalibration {
  return {
    left_diameter_cm: LEFT_WHEEL_DIAMETER_CM,
    right_diameter_cm: RIGHT_WHEEL_DIAMETER_CM,
    left_counts_per_rev: LEFT_COUNTS_PER_WHEEL_REV,
    right_counts_per_rev: RIGHT_COUNTS_PER_WHEEL_REV,
    track_cm: WHEEL_TRACK_CM,
    left_sign: 1,
    right_sign: -1,
    confirmed: true,
  };
}

export class OdometryModel {
  public calibration: OdometryCalibration;
  public previous: EncoderCounts = { left: 0, right: 0 };
  public ready = false;
  public primed = false;
  public left_cm_per_count = 0;
  public right_cm_per_count = 0;
  public distance_cm = 0;
  public heading_rad = 0;
  public left_speed_cm_s = 0;
  public right_speed_cm_s = 0;
  public left_distance_cm = 0;
  public right_distance_cm = 0;
  public x_cm = 0;
  public y_cm = 0;
  public sensor_x_cm = GRAY_FORWARD_OFFSET_CM;
  public sensor_y_cm = 0;
  public sensor_distance_cm = 0;
  public left_delta = 0;
  public right_delta = 0;

  constructor(cal?: OdometryCalibration) {
    this.calibration = cal ? { ...cal } : defaultOdometryCalibration();
    this.init(this.calibration);
  }

  public init(c: OdometryCalibration): void {
    this.calibration = { ...c };
    this.previous = { left: 0, right: 0 };
    this.ready = false;
    this.primed = false;
    this.distance_cm = 0;
    this.heading_rad = 0;
    this.left_speed_cm_s = 0;
    this.right_speed_cm_s = 0;
    this.left_distance_cm = 0;
    this.right_distance_cm = 0;
    this.x_cm = 0;
    this.y_cm = 0;
    this.sensor_x_cm = GRAY_FORWARD_OFFSET_CM;
    this.sensor_y_cm = 0;
    this.sensor_distance_cm = 0;
    this.left_delta = 0;
    this.right_delta = 0;

    const positive = (v: number) => Number.isFinite(v) && v > 0;
    this.ready =
      c.confirmed &&
      positive(ODOMETRY_DISTANCE_SCALE) &&
      positive(ODOMETRY_HEADING_SCALE) &&
      positive(c.left_diameter_cm) &&
      positive(c.right_diameter_cm) &&
      positive(c.left_counts_per_rev) &&
      positive(c.right_counts_per_rev) &&
      positive(c.track_cm) &&
      (c.left_sign === 1 || c.left_sign === -1) &&
      (c.right_sign === 1 || c.right_sign === -1);

    if (!this.ready) return;

    this.left_cm_per_count =
      (Math.PI * c.left_diameter_cm) / c.left_counts_per_rev * ODOMETRY_DISTANCE_SCALE;
    this.right_cm_per_count =
      (Math.PI * c.right_diameter_cm) / c.right_counts_per_rev * ODOMETRY_DISTANCE_SCALE;

    const max_dt = CONTROL_MAX_INTERVAL_MS / 1000.0;
    this.ready =
      positive(this.left_cm_per_count) &&
      positive(this.right_cm_per_count) &&
      (ODOMETRY_MAX_SPEED_CM_S * max_dt) / this.left_cm_per_count + 4.0 < 32768.0 &&
      (ODOMETRY_MAX_SPEED_CM_S * max_dt) / this.right_cm_per_count + 4.0 < 32768.0;
  }

  private delta(now: number, before: number): number {
    const d = (now - before) & 0xffff;
    return d < 32768 ? d : d - 65536;
  }

  public updateWithHeading(
    counts: EncoderCounts,
    dt: number,
    use_imu: boolean,
    imu_heading_rad: number
  ): boolean {
    if (
      !this.ready ||
      !Number.isFinite(dt) ||
      dt <= 0.0 ||
      dt > CONTROL_MAX_INTERVAL_MS / 1000.0 ||
      (use_imu && !Number.isFinite(imu_heading_rad))
    ) {
      return false;
    }

    if (!this.primed) {
      this.previous = { ...counts };
      this.primed = true;
      if (use_imu) {
        this.heading_rad = imu_heading_rad;
        this.sensor_x_cm = this.x_cm + GRAY_FORWARD_OFFSET_CM * Math.cos(this.heading_rad);
        this.sensor_y_cm = this.y_cm + GRAY_FORWARD_OFFSET_CM * Math.sin(this.heading_rad);
      }
      return true;
    }

    let dl = this.delta(counts.left, this.previous.left);
    let dr = this.delta(counts.right, this.previous.right);
    if (dl === -32768 || dr === -32768) return false;

    dl *= this.calibration.left_sign;
    dr *= this.calibration.right_sign;

    const l = dl * this.left_cm_per_count;
    const r = dr * this.right_cm_per_count;

    if (
      Math.abs(l) > ODOMETRY_MAX_SPEED_CM_S * dt + 4.0 * this.left_cm_per_count ||
      Math.abs(r) > ODOMETRY_MAX_SPEED_CM_S * dt + 4.0 * this.right_cm_per_count
    ) {
      return false;
    }

    this.previous = { ...counts };
    this.left_delta = dl;
    this.right_delta = dr;

    const ds = (l + r) * 0.5;
    const encoder_dh = ((l - r) / this.calibration.track_cm) * ODOMETRY_HEADING_SCALE;
    const dh = use_imu ? imu_heading_rad - this.heading_rad : encoder_dh;
    const half = dh * 0.5;
    const sinc = Math.abs(half) < 0.00001 ? 1.0 : Math.sin(half) / half;
    const mid = this.heading_rad + half;

    this.left_distance_cm += l;
    this.right_distance_cm += r;
    this.distance_cm += ds;
    this.x_cm += ds * sinc * Math.cos(mid);
    this.y_cm += ds * sinc * Math.sin(mid);

    this.sensor_distance_cm += Math.hypot(ds, GRAY_FORWARD_OFFSET_CM * dh);
    this.heading_rad = use_imu ? imu_heading_rad : this.heading_rad + dh;
    this.sensor_x_cm = this.x_cm + GRAY_FORWARD_OFFSET_CM * Math.cos(this.heading_rad);
    this.sensor_y_cm = this.y_cm + GRAY_FORWARD_OFFSET_CM * Math.sin(this.heading_rad);

    const alpha = dt / (SPEED_FILTER_TAU_S + dt);
    this.left_speed_cm_s += alpha * (l / dt - this.left_speed_cm_s);
    this.right_speed_cm_s += alpha * (r / dt - this.right_speed_cm_s);

    return (
      Number.isFinite(this.x_cm) &&
      Number.isFinite(this.y_cm) &&
      Number.isFinite(this.left_distance_cm) &&
      Number.isFinite(this.right_distance_cm) &&
      Number.isFinite(this.sensor_distance_cm) &&
      Number.isFinite(this.distance_cm) &&
      Number.isFinite(this.heading_rad) &&
      Number.isFinite(this.left_speed_cm_s) &&
      Number.isFinite(this.right_speed_cm_s)
    );
  }

  public update(counts: EncoderCounts, dt: number): boolean {
    return this.updateWithHeading(counts, dt, false, 0.0);
  }
}
