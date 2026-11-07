/**
 * Aggregate cruise_scan summaries into the markdown tables used in the
 * evidence README. Every number in the report comes from these JSON files.
 *
 *   node tests/scan_report.mjs s2-60-rev0=/tmp/spd-scan/s2-60-rev0 ...
 */
import { readFileSync } from 'node:fs';

const entries = process.argv.slice(2).map((arg) => {
  const at = arg.indexOf('=');
  return { label: arg.slice(0, at), dir: arg.slice(at + 1) };
});
const runs = entries.map((e) => ({
  label: e.label,
  ...JSON.parse(readFileSync(`${e.dir}/summary.json`, 'utf8')),
}));

const rows = [];
rows.push('### 方案 2 前 6 步：不同巡航速度与反向制动开关（完整仿真回放）');
rows.push('');
rows.push('| 运行 | 反向开关 | 达到步索引 | 终态 | 失败 | 仿真时长 s | 里程 cm | 峰值轮速 cm/s | 贴线帧占比 % | 贴线 max\\|pos\\| | 反向作用 ms |');
rows.push('|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|');
for (const r of runs) {
  const f = r.final;
  rows.push(
    `| ${r.label} | ${r.switches.REVERSE_BRAKE_ARMED ? '开' : '关'} | ${r.reached_index} | ${f.state} | ${f.failure_name} (idx ${f.index}) | ${(f.ms / 1000).toFixed(2)} | ${f.distance_cm.toFixed(1)} | ${r.peak_speed_cm_s.toFixed(1)} | ${r.tracking_pct} | ${r.max_abs_pos_tracking} | ${r.reverse_active_ms} |`
  );
}
rows.push('');
rows.push('### 逐步完成情况（✓ = 该步动作已完成并推进到下一步）');
rows.push('');
const stepsHeader = Array.from({ length: 6 }, (_, i) => `步骤 ${i}`).join(' | ');
rows.push(`| 运行 | ${stepsHeader} | 失败步 |`);
rows.push(`|---|${Array(6).fill(':-:').join('|')}|---:|`);
for (const r of runs) {
  const cells = [];
  for (let i = 0; i < 6; i++) {
    const completed = r.reached_index > i;
    cells.push(completed ? '✓' : '✗');
  }
  rows.push(`| ${r.label} | ${cells.join(' | ')} | ${r.reached_index <= 5 ? r.reached_index : '—'} |`);
}
rows.push('');
rows.push('### 各次转弯执行时的入口轮速（EXECUTING 前 100 ms 实测均速）');
rows.push('');
rows.push('| 运行 | step 0 | step 1 | step 2 | step 3 | step 4 | step 5 |');
rows.push('|---|---:|---:|---:|---:|---:|---:|');
for (const r of runs) {
  const speeds = [];
  for (let i = 0; i < 6; i++) {
    const entry = r.executing_entries.find((e) => e.index === i);
    speeds.push(entry ? entry.entry_speed_100ms_before.toFixed(1) : '—');
  }
  rows.push(`| ${r.label} | ${speeds.join(' | ')} |`);
}
rows.push('');
rows.push('### 每步起步位置与时刻');
rows.push('');
rows.push('| 运行 | 步骤起止 | 位置 / 里程 |');
rows.push('|---|---|---|');
for (const r of runs) {
  const parts = r.step_starts
    .filter((s) => s.index <= 6)
    .map((s) => `#${s.index}@${(s.ms / 1000).toFixed(2)}s (${s.x.toFixed(0)},${s.y.toFixed(0)})`);
  rows.push(`| ${r.label} | ${parts.join(' → ')} | 终态 ${r.final.distance_cm.toFixed(0)} cm |`);
}
console.log(rows.join('\n'));
