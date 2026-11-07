/** Traction-limit sensitivity: reverse-brake distance from 300 to 16 cm/s. */
import { readFileSync } from 'node:fs';
const files = process.argv.slice(2);
const rows = [];
for (const file of files) {
  const d = JSON.parse(readFileSync(file, 'utf8'));
  const r = d.rows.find((x) => x.v0 === 300 && x.method === 'reverse');
  rows.push(
    `| ${d.switches.MOTOR_REVERSE_MAX_DECEL_CM_S2} | ${r.t16.toFixed(3)} | ${r.d16.toFixed(2)} | ${r.mean_decel_16.toFixed(0)} |`
  );
}
console.log('| 反向牵引极限 cm/s² | 300→16 时间 s | 300→16 距离 cm | 平均减速度 cm/s² |');
console.log('|---:|---:|---:|---:|');
console.log(rows.join('\n'));
