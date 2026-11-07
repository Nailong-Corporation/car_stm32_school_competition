/**
 * Straight-line braking benchmark for the speed loop + plant.
 *
 * Uses the real SpeedControlModel, the real OdometryModel and the real
 * advanceWheel() plant. The wheel/travel/encoder integration is the
 * speed/pose subset of VehicleSimulator.step(); no route or line logic runs.
 * (The full VehicleSimulator cannot express an explicit short brake, because
 * its command interface maps "both PWM = 0" to coast, which is exactly the
 * production zero-output semantics we must not change.)
 *
 * Methods compared from a settled cruise speed down to the 16 cm/s entry cap:
 *   coast       = both-zero command, STBY low (motor_stop), plant tau 0.35 s
 *   short_brake = zero duty with STBY high (TB6612 short brake), plant tau 0.08 s
 *   reverse     = controller target 16 cm/s with reverse brake armed; the speed
 *                 PID may emit negative PWM down to -REVERSE_BRAKE_MAX_PWM
 *
 * Run with the reverse switch on:
 *   SIM_MOTOR_REVERSE_BRAKE=1 SIM_BRAKING_REVERSE_HW=1 node tests/speed_brake.mjs /tmp/spd-brake
 */
import { createServer } from 'vite';
import { writeFileSync, mkdirSync } from 'node:fs';

const out = process.argv[2] ?? '/tmp/spd-brake';
mkdirSync(out, { recursive: true });
const dt = 0.005;
const CROSS = 16.0;      // entry speed cap to reach
const STOPPED = 0.5;     // physical standstill threshold (cm/s)
const CRUISES = [40, 80, 120, 160, 200, 300];

const server = await createServer({
  server: { middlewareMode: true },
  logLevel: 'error',
  optimizeDeps: { noDiscovery: true, include: [] },
});
try {
  const { SpeedControlModel } = await server.ssrLoadModule('/src/sim/speed_control.ts');
  const { advanceWheel } = await server.ssrLoadModule('/src/sim/physics.ts');
  const { OdometryModel } = await server.ssrLoadModule('/src/sim/odometry.ts');
  const cfg = await server.ssrLoadModule('/src/sim/config.ts');

  /**
   * One two-wheel rig: physical wheel speeds -> encoder counts -> odometry ->
   * measured wheel speeds. Mirrors VehicleSimulator.step sections 5..6.
   */
  function makeRig() {
    return {
      odometry: new OdometryModel(),
      speed: new SpeedControlModel(),
      v: { left: 0, right: 0 },
      counts: { left: 0, right: 0 },
      accum: { left: 0, right: 0 },
      time_s: 0,
    };
  }

  /** Advance the plant by one control period for an explicit driver command. */
  function advance(rig, command) {
    const signedLeft = command.left_forward ? command.left_pwm : -command.left_pwm;
    const signedRight = command.right_forward ? command.right_pwm : -command.right_pwm;
    const targetLeft = command.standby ? (signedLeft / 255.0) * cfg.VEHICLE_MAX_SPEED_CM_S : 0;
    const targetRight = command.standby ? (signedRight / 255.0) * cfg.VEHICLE_MAX_SPEED_CM_S : 0;
    const enabled = command.standby && (command.left_pwm !== 0 || command.right_pwm !== 0);
    const oldLeft = rig.v.left;
    const oldRight = rig.v.right;
    rig.v.left = advanceWheel(oldLeft, targetLeft, enabled, dt);
    rig.v.right = advanceWheel(oldRight, targetRight, enabled, dt);
    const leftTravel = (oldLeft + rig.v.left) * 0.5 * dt;
    const rightTravel = (oldRight + rig.v.right) * 0.5 * dt;

    rig.accum.left += leftTravel / rig.odometry.left_cm_per_count;
    rig.accum.right += rightTravel / rig.odometry.right_cm_per_count;
    const dl = Math.trunc(rig.accum.left) * rig.odometry.calibration.left_sign;
    const dr = Math.trunc(rig.accum.right) * rig.odometry.calibration.right_sign;
    rig.accum.left -= Math.trunc(rig.accum.left);
    rig.accum.right -= Math.trunc(rig.accum.right);
    rig.counts.left = (rig.counts.left + dl) & 0xffff;
    rig.counts.right = (rig.counts.right + dr) & 0xffff;
    rig.odometry.update(rig.counts, dt);
    rig.time_s += dt;
  }

  /** Advance with an explicit short brake (zero duty, STBY high). */
  function advanceShortBrake(rig) {
    const oldLeft = rig.v.left;
    const oldRight = rig.v.right;
    rig.v.left = advanceWheel(oldLeft, 0, true, dt);
    rig.v.right = advanceWheel(oldRight, 0, true, dt);
    const leftTravel = (oldLeft + rig.v.left) * 0.5 * dt;
    const rightTravel = (oldRight + rig.v.right) * 0.5 * dt;
    rig.accum.left += leftTravel / rig.odometry.left_cm_per_count;
    rig.accum.right += rightTravel / rig.odometry.right_cm_per_count;
    const dl = Math.trunc(rig.accum.left) * rig.odometry.calibration.left_sign;
    const dr = Math.trunc(rig.accum.right) * rig.odometry.calibration.right_sign;
    rig.accum.left -= Math.trunc(rig.accum.left);
    rig.accum.right -= Math.trunc(rig.accum.right);
    rig.counts.left = (rig.counts.left + dl) & 0xffff;
    rig.counts.right = (rig.counts.right + dr) & 0xffff;
    rig.odometry.update(rig.counts, dt);
    rig.time_s += dt;
  }

  const measured = (rig) => 0.5 * (rig.odometry.left_speed_cm_s + rig.odometry.right_speed_cm_s);

  function cruiseTo(v0) {
    const rig = makeRig();
    let settled = 0;
    for (let i = 0; i < 8000; i++) {
      const cmd = rig.speed.update(v0, v0, rig.odometry.left_speed_cm_s, rig.odometry.right_speed_cm_s, dt, false);
      if (!cmd.success) throw new Error(`cruise fault ${rig.speed.fault}`);
      advance(rig, cmd.command);
      settled = Math.abs(measured(rig) - v0) <= 1.5 ? settled + 1 : 0;
      if (settled >= 200) return rig;
    }
    throw new Error(`cruise ${v0} never settled, got ${measured(rig)}`);
  }

  function brakedRun(v0, method) {
    const rig = cruiseTo(v0);
    const d0 = rig.odometry.distance_cm;
    const t0 = rig.time_s;
    let t16 = null, d16 = null, tStop = null, dStop = null, maxDecel = 0, maxReversePwm = 0, reverseMs = 0;
    let previous = measured(rig);
    for (let i = 0; i < 40000; i++) {
      if (method === 'coast') {
        advance(rig, { left_pwm: 0, right_pwm: 0, left_forward: true, right_forward: true, standby: false });
      } else if (method === 'short_brake') {
        advanceShortBrake(rig);
      } else {
        const cmd = rig.speed.update(CROSS, CROSS, rig.odometry.left_speed_cm_s, rig.odometry.right_speed_cm_s, dt, true);
        if (!cmd.success) throw new Error(`reverse fault ${rig.speed.fault}`);
        if (!cmd.command.left_forward) {
          maxReversePwm = Math.max(maxReversePwm, cmd.command.left_pwm);
          reverseMs += dt * 1000;
        }
        advance(rig, cmd.command);
      }
      const v = measured(rig);
      const decel = (previous - v) / dt;
      if (decel > maxDecel) maxDecel = decel;
      previous = v;
      const t = rig.time_s - t0;
      const d = rig.odometry.distance_cm - d0;
      if (t16 === null && v <= CROSS) { t16 = t; d16 = d; }
      if (tStop === null && v <= STOPPED) { tStop = t; dStop = d; }
      if (method !== 'reverse' && tStop !== null) break;
      if (method === 'reverse' && t16 !== null && rig.time_s - t0 > t16 + 0.2) break;
    }
    return {
      v0, method,
      t16, d16,
      t_stop: tStop, d_stop: dStop,
      mean_decel_16: t16 && t16 > 0 ? (v0 - CROSS) / t16 : null,
      max_decel: maxDecel,
      max_reverse_pwm: maxReversePwm,
      reverse_ms: reverseMs,
    };
  }

  /** Max cruise that can still brake to CROSS within `dist` at decel `a`. */
  function envelopeTable(decel, distances) {
    return distances.map((d) => ({
      distance_cm: d,
      max_cruise_cm_s: Math.sqrt(CROSS * CROSS + 2 * decel * d),
    }));
  }

  const rows = [];
  for (const v0 of CRUISES) {
    for (const method of ['coast', 'short_brake', 'reverse']) rows.push(brakedRun(v0, method));
  }

  const fmt = (v, digits = 2) => (v === null || v === undefined ? '—' : v.toFixed(digits));
  const md = [];
  md.push('### 制动到入口速度 16 cm/s（直线，仿真）');
  md.push('');
  md.push('| 巡航 cm/s | 方式 | 时间 s | 距离 cm | 平均减速度 cm/s² | 峰值减速度 cm/s² | 反向 PWM 峰值 | 反向作用 ms |');
  md.push('|---:|---|---:|---:|---:|---:|---:|---:|');
  for (const r of rows) {
    md.push(
      `| ${r.v0} | ${r.method} | ${fmt(r.t16, 3)} | ${fmt(r.d16)} | ${fmt(r.mean_decel_16, 0)} | ${fmt(r.max_decel, 0)} | ${fmt(r.max_reverse_pwm, 0)} | ${fmt(r.reverse_ms, 0)} |`
    );
  }
  md.push('');
  md.push('### 从巡航速度刹到物理停止（仅滑行 / 短刹可到达；反向按设计不换向）');
  md.push('');
  md.push('| 巡航 cm/s | 方式 | 停止时间 s | 停止距离 cm |');
  md.push('|---:|---|---:|---:|');
  for (const r of rows) {
    if (r.method === 'reverse') continue;
    md.push(`| ${r.v0} | ${r.method} | ${fmt(r.t_stop, 3)} | ${fmt(r.d_stop)} |`);
  }
  md.push('');
  md.push('### 给定可用制动距离的安全巡航上限（v = sqrt(16² + 2·a·d)）');
  md.push('');
  const distances = [10, 20, 30, 40, 60, 80, 100, 150, 200];
  const distances2 = [20, 30, 40, 60, 80, 100, 150, 190];
  md.push('| 可用距离 cm | a=120 cm/s²（规划值） | a=300 cm/s²（当前减速度上限） | a=600 cm/s²（反向力矩，牵引极限） |');
  md.push('|---:|---:|---:|---:|');
  const e120 = envelopeTable(cfg.BRAKING_SAFE_DECEL_CM_S2, distances);
  const e300 = envelopeTable(cfg.MOTOR_MAX_DECEL_CM_S2, distances);
  const e600 = envelopeTable(cfg.MOTOR_REVERSE_MAX_DECEL_CM_S2, distances);
  distances.forEach((d, i) => {
    md.push(`| ${d} | ${e120[i].max_cruise_cm_s.toFixed(0)} | ${e300[i].max_cruise_cm_s.toFixed(0)} | ${e600[i].max_cruise_cm_s.toFixed(0)} |`);
  });

  md.push('');
  md.push('### 给定段内可用制动距离、从巡航刹到 16 cm/s 所需的平均减速度（cm/s²）');
  md.push('');
  md.push('| 巡航 cm/s | 20 cm | 30 cm | 40 cm | 60 cm | 80 cm | 100 cm | 150 cm | 190 cm |');
  md.push('|---:|---:|---:|---:|---:|---:|---:|---:|---:|');
  for (const v of [100, 150, 200, 300, 500]) {
    const cells = distances2.map((d) => ((v * v - CROSS * CROSS) / (2 * d)).toFixed(0));
    md.push(`| ${v} | ${cells.join(' | ')} |`);
  }

  const result = {
    generated_at: new Date().toISOString(),
    switches: {
      MOTOR_REVERSE_BRAKE_ENABLED: cfg.MOTOR_REVERSE_BRAKE_ENABLED,
      BRAKING_REVERSE_HARDWARE_CONFIRMED: cfg.BRAKING_REVERSE_HARDWARE_CONFIRMED,
      REVERSE_BRAKE_ARMED: cfg.REVERSE_BRAKE_ARMED,
      REVERSE_BRAKE_MAX_PWM: cfg.REVERSE_BRAKE_MAX_PWM,
      MOTOR_MAX_DECEL_CM_S2: cfg.MOTOR_MAX_DECEL_CM_S2,
      MOTOR_REVERSE_MAX_DECEL_CM_S2: cfg.MOTOR_REVERSE_MAX_DECEL_CM_S2,
      BRAKING_SAFE_DECEL_CM_S2: cfg.BRAKING_SAFE_DECEL_CM_S2,
    },
    rows,
    markdown: md.join('\n'),
  };
  writeFileSync(`${out}/speed_brake.json`, JSON.stringify(result, null, 2));
  writeFileSync(`${out}/speed_brake.md`, result.markdown + '\n');
  console.log(result.markdown);
  console.log(`switches: ${JSON.stringify(result.switches)}`);
} finally {
  await server.close();
}
