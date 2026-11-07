export interface PidConfig {
  kp: number;
  ki: number;
  kd: number;
  integral_limit: number;
  derivative_tau: number;
}

export class Pid {
  public config: PidConfig;
  public integral = 0;
  public previous_error = 0;
  public derivative = 0;
  public primed = false;

  constructor(config: PidConfig) {
    this.config = { ...config };
    this.reset();
  }

  public reset(): void {
    this.integral = 0;
    this.previous_error = 0;
    this.derivative = 0;
    this.primed = false;
  }

  public update(error: number, dt: number, limit: number): { success: boolean; output: number } {
    return this.updateBounded(error, dt, -limit, limit, 0.0);
  }

  public updateBounded(
    error: number,
    dt: number,
    low: number,
    high: number,
    feedforward: number
  ): { success: boolean; output: number } {
    const c = this.config;
    if (
      !Number.isFinite(error) ||
      !Number.isFinite(dt) ||
      dt <= 0.0 ||
      dt > 0.1 ||
      !Number.isFinite(low) ||
      !Number.isFinite(high) ||
      low > high ||
      !Number.isFinite(feedforward) ||
      !Number.isFinite(c.kp) ||
      c.kp < 0 ||
      !Number.isFinite(c.ki) ||
      c.ki < 0 ||
      !Number.isFinite(c.kd) ||
      c.kd < 0 ||
      !Number.isFinite(c.integral_limit) ||
      c.integral_limit < 0 ||
      !Number.isFinite(c.derivative_tau) ||
      c.derivative_tau < 0
    ) {
      this.reset();
      return { success: false, output: 0.0 };
    }

    const slope = this.primed ? (error - this.previous_error) / dt : 0.0;
    const alpha = dt / (c.derivative_tau + dt);
    this.derivative += alpha * (slope - this.derivative);

    let integral = this.integral + error * dt;
    if (integral > c.integral_limit) integral = c.integral_limit;
    if (integral < -c.integral_limit) integral = -c.integral_limit;

    const candidate = feedforward + c.kp * error + c.ki * integral + c.kd * this.derivative;
    if (!((candidate > high && error > 0.0) || (candidate < low && error < 0.0))) {
      this.integral = integral;
    }

    const value = feedforward + c.kp * error + c.ki * this.integral + c.kd * this.derivative;
    if (!Number.isFinite(value)) {
      this.reset();
      return { success: false, output: 0.0 };
    }

    const output = Math.max(low, Math.min(high, value));
    this.previous_error = error;
    this.primed = true;
    return { success: true, output };
  }
}
