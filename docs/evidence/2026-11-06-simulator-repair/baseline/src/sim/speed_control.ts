import { Pid } from './pid';
import type { MotorCommand } from './types';
import {
  SPEED_KP,
  SPEED_KI,
  SPEED_KD,
  SPEED_INTEGRAL_LIMIT,
  PID_DERIVATIVE_TAU_S,
  SPEED_FEEDFORWARD,
  MOTOR_PWM_LIMIT,
  WHEEL_TARGET_MAX_CM_S,
  SPEED_STALL_TARGET_CM_S,
  SPEED_STALL_MEASURED_CM_S,
  SPEED_STALL_TIMEOUT_S,
  SPEED_REVERSE_THRESHOLD_CM_S,
  SPEED_REVERSE_CONFIRM_S,
  SPEED_REVERSE_CONFIRM_CM,
  SPEED_REVERSE_IMMEDIATE_CM_S,
  CONTROL_MAX_INTERVAL_MS,
} from './config';

export enum SpeedFault {
  SPEED_FAULT_NONE = 0,
  SPEED_FAULT_INPUT = 1,
  SPEED_FAULT_INTERVAL = 2,
  SPEED_FAULT_REVERSE = 3,
  SPEED_FAULT_STALL = 4,
  SPEED_FAULT_PID = 5,
}

export class SpeedControlModel {
  public left: Pid;
  public right: Pid;
  public left_stall_s = 0;
  public right_stall_s = 0;
  public left_reverse_s = 0;
  public right_reverse_s = 0;
  public left_reverse_cm = 0;
  public right_reverse_cm = 0;
  public reverse_pending = false;
  public fault = SpeedFault.SPEED_FAULT_NONE;
  public fault_wheel = 0;

  constructor() {
    const config = {
      kp: SPEED_KP,
      ki: SPEED_KI,
      kd: SPEED_KD,
      integral_limit: SPEED_INTEGRAL_LIMIT,
      derivative_tau: PID_DERIVATIVE_TAU_S,
    };
    this.left = new Pid(config);
    this.right = new Pid(config);
    this.reset();
  }

  public reset(): void {
    this.left.reset();
    this.right.reset();
    this.left_stall_s = 0;
    this.right_stall_s = 0;
    this.left_reverse_s = 0;
    this.right_reverse_s = 0;
    this.left_reverse_cm = 0;
    this.right_reverse_cm = 0;
    this.reverse_pending = false;
    this.fault = SpeedFault.SPEED_FAULT_NONE;
    this.fault_wheel = 0;
  }

  private wheel(
    pid: Pid,
    target: number,
    measured: number,
    dt: number,
    which: 'left' | 'right'
  ): { fault: SpeedFault; pwm: number } {
    if (
      !Number.isFinite(target) ||
      target < 0.0 ||
      target > WHEEL_TARGET_MAX_CM_S ||
      !Number.isFinite(measured)
    ) {
      return { fault: SpeedFault.SPEED_FAULT_INPUT, pwm: 0.0 };
    }

    if (target === 0.0) {
      pid.reset();
      if (which === 'left') this.left_stall_s = 0;
      else this.right_stall_s = 0;
      return { fault: SpeedFault.SPEED_FAULT_NONE, pwm: 0.0 };
    }

    let stall = which === 'left' ? this.left_stall_s : this.right_stall_s;
    stall =
      target >= SPEED_STALL_TARGET_CM_S && measured < SPEED_STALL_MEASURED_CM_S
        ? stall + dt
        : 0.0;
    if (which === 'left') this.left_stall_s = stall;
    else this.right_stall_s = stall;

    if (stall >= SPEED_STALL_TIMEOUT_S) {
      return { fault: SpeedFault.SPEED_FAULT_STALL, pwm: 0.0 };
    }

    const res = pid.updateBounded(
      target - measured,
      dt,
      0.0,
      MOTOR_PWM_LIMIT,
      target * SPEED_FEEDFORWARD
    );

    return {
      fault: res.success ? SpeedFault.SPEED_FAULT_NONE : SpeedFault.SPEED_FAULT_PID,
      pwm: res.output,
    };
  }

  private reverseConfirmed(
    measured: number,
    dt: number,
    which: 'left' | 'right'
  ): boolean {
    if (measured >= -SPEED_REVERSE_THRESHOLD_CM_S) {
      if (which === 'left') {
        this.left_reverse_s = 0;
        this.left_reverse_cm = 0;
      } else {
        this.right_reverse_s = 0;
        this.right_reverse_cm = 0;
      }
      return false;
    }

    if (which === 'left') {
      this.left_reverse_s += dt;
      this.left_reverse_cm += -measured * dt;
      return (
        measured <= -SPEED_REVERSE_IMMEDIATE_CM_S ||
        this.left_reverse_s + 0.0000001 >= SPEED_REVERSE_CONFIRM_S ||
        this.left_reverse_cm + 0.0000001 >= SPEED_REVERSE_CONFIRM_CM
      );
    } else {
      this.right_reverse_s += dt;
      this.right_reverse_cm += -measured * dt;
      return (
        measured <= -SPEED_REVERSE_IMMEDIATE_CM_S ||
        this.right_reverse_s + 0.0000001 >= SPEED_REVERSE_CONFIRM_S ||
        this.right_reverse_cm + 0.0000001 >= SPEED_REVERSE_CONFIRM_CM
      );
    }
  }

  public update(
    lt: number,
    rt: number,
    lm: number,
    rm: number,
    dt: number
  ): { success: boolean; command: MotorCommand } {
    const emptyCommand: MotorCommand = {
      left_pwm: 0,
      right_pwm: 0,
      left_forward: true,
      right_forward: true,
      standby: false,
    };

    this.fault = SpeedFault.SPEED_FAULT_NONE;
    this.fault_wheel = 0;
    this.reverse_pending = false;

    if (!Number.isFinite(dt) || dt <= 0.0 || dt > CONTROL_MAX_INTERVAL_MS / 1000.0) {
      this.fault = SpeedFault.SPEED_FAULT_INTERVAL;
      return { success: false, command: emptyCommand };
    }

    let fault = SpeedFault.SPEED_FAULT_NONE;
    let failed_wheel = 0;

    if (!Number.isFinite(lt) || lt < 0 || lt > WHEEL_TARGET_MAX_CM_S || !Number.isFinite(lm)) {
      fault = SpeedFault.SPEED_FAULT_INPUT;
      failed_wheel = 1;
    } else if (!Number.isFinite(rt) || rt < 0 || rt > WHEEL_TARGET_MAX_CM_S || !Number.isFinite(rm)) {
      fault = SpeedFault.SPEED_FAULT_INPUT;
      failed_wheel = 2;
    } else {
      const left_rev = this.reverseConfirmed(lm, dt, 'left');
      const right_rev = this.reverseConfirmed(rm, dt, 'right');
      if (left_rev || right_rev) {
        fault = SpeedFault.SPEED_FAULT_REVERSE;
        failed_wheel = left_rev ? 1 : 2;
      } else if (this.left_reverse_s > 0 || this.right_reverse_s > 0) {
        this.left.reset();
        this.right.reset();
        this.left_stall_s = 0;
        this.right_stall_s = 0;
        this.reverse_pending = true;
        return { success: true, command: emptyCommand };
      }
    }

    let l = 0;
    let r = 0;
    if (fault === SpeedFault.SPEED_FAULT_NONE) {
      const res = this.wheel(this.left, lt, lm, dt, 'left');
      fault = res.fault;
      l = res.pwm;
      failed_wheel = 1;
    }
    if (fault === SpeedFault.SPEED_FAULT_NONE) {
      const res = this.wheel(this.right, rt, rm, dt, 'right');
      fault = res.fault;
      r = res.pwm;
      failed_wheel = 2;
    }

    if (fault !== SpeedFault.SPEED_FAULT_NONE) {
      this.reset();
      this.fault = fault;
      this.fault_wheel = failed_wheel;
      return { success: false, command: emptyCommand };
    }

    const command: MotorCommand = {
      left_pwm: l,
      right_pwm: r,
      left_forward: true,
      right_forward: true,
      standby: lt > 0.0 || rt > 0.0,
    };

    return { success: true, command };
  }
}
