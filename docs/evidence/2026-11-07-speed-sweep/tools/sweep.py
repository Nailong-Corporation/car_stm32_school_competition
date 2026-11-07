#!/usr/bin/env python3
"""Parameter sweep driver for the STM32 car simulator copy at /tmp/sim-speed-sweep.

Each experiment:
  1. restore src/sim/config.ts from work/config.baseline.ts
  2. apply numeric constant overrides
  3. regenerate work/table.c from work/manifest.json with the requested cruise/turn
  4. run tests/replay.mjs 1 and 2 (parallel) with SIM_ROUTE_TABLE=work/table.c
  5. parse runs/<tag>/scheme<n>.json final row -> result row

Usage:
  python3 sweep.py <experiments.json> <results.jsonl>

experiments.json: [{"tag": "...", "config": {"NAME": value, ...},
                    "cruise": 60, "turn": 16}, ...]
"""
import concurrent.futures as cf
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

SIM = Path('/tmp/sim-speed-sweep')
WORK = SIM / 'work'
CONFIG = SIM / 'src/sim/config.ts'
BASELINE = WORK / 'config.baseline.ts'
ROUTE = SIM / 'src/sim/route.ts'
ROUTE_BASELINE = WORK / 'route.baseline.ts'
FOLLOW = SIM / 'src/sim/line_follow.ts'
FOLLOW_BASELINE = WORK / 'line_follow.baseline.ts'
FOLLOW_ORIG = WORK / 'line_follow.orig.ts'


def apply_constants(path, baseline, overrides):
    text = baseline.read_text()
    for name, value in overrides.items():
        pat = re.compile(r'((?:export )?const ' + re.escape(name) + r' = )([-+0-9.eE]+)(;)')
        if not pat.search(text):
            raise SystemExit(f'constant not found in {path.name}: {name}')
        repl = value if isinstance(value, str) else (repr(value) if isinstance(value, float) else str(value))
        text = pat.sub(lambda m: m.group(1) + repl + m.group(3), text, count=1)
    path.write_text(text)
MANIFEST = WORK / 'manifest.json'
TABLE = WORK / 'table.c'

FAILURE_NAMES = {
    0: 'NONE', 1: 'INVALID', 2: 'WINDOW', 4: 'CONFIRM_TIMEOUT', 5: 'ACTION_TIMEOUT',
    6: 'TURN_LIMIT', 7: 'WRONG_WAY', 8: 'EXIT_RECAPTURE', 9: 'SEARCH',
    10: 'ENTRY_SPEED', 11: 'EXIT_LOST', 12: 'CLEAR_LINE_LOST', 13: 'BRAKE_DISTANCE',
}


def apply_config(overrides):
    text = BASELINE.read_text()
    for name, value in overrides.items():
        pat = re.compile(r'(export const ' + re.escape(name) + r' = )([-+0-9.eE]+)(;)')
        if not pat.search(text):
            raise SystemExit(f'config constant not found: {name}')
        if isinstance(value, str):
            repl = value  # raw expression (e.g. 'Math.PI / 2')
        elif isinstance(value, float):
            repl = repr(value)
        else:
            repl = str(value)
        text = pat.sub(lambda m: m.group(1) + repl + m.group(3), text, count=1)
    CONFIG.write_text(text)


def apply_manifest(cruise, turn, table=None):
    m = json.loads((WORK / 'manifest.orig.json').read_text())
    m['cruise_speed_cm_s'] = cruise
    m['turn_speed_cm_s'] = turn
    for sid, over in (table or {}).items():
        sc = m['schemes'][str(sid)]
        if 'turn' in over:
            sc['turn_speed_overrides'] = over['turn']
        if 'cruise' in over:
            sc['cruise_speed_overrides'] = over['cruise']
    MANIFEST.write_text(json.dumps(m, indent=1, ensure_ascii=False))
    assert turn <= cruise, (turn, cruise)
    subprocess.run([sys.executable, str(WORK / 'gentable.py'), str(MANIFEST), str(TABLE)],
                   check=True, capture_output=True)


def run_scheme(scheme, tag):
    out = SIM / 'runs' / tag
    out.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, SIM_ROUTE_TABLE=str(TABLE), NO_COLOR='1')
    p = subprocess.run(['node', 'tests/replay.mjs', str(scheme), str(out)],
                       cwd=SIM, env=env, capture_output=True, text=True, timeout=300)
    (out / f'scheme{scheme}.log').write_text(p.stdout + p.stderr)
    jf = out / f'scheme{scheme}.json'
    if p.returncode != 0 or not jf.exists():
        return {'scheme': scheme, 'ok': False, 'error': (p.stdout + p.stderr)[-800:]}
    d = json.loads(jf.read_text())
    f = d['final']
    return {
        'scheme': scheme,
        'ok': True,
        'state': f['state'],
        'failure': f['failure'],
        'failure_name': FAILURE_NAMES.get(f['failure'], str(f['failure'])),
        'index': f['index'],
        'last_index': len(d['steps']) - 1,
        'ms': f['ms'],
        'sec': round(f['ms'] / 1000.0, 2),
        'count': f['count'],
        'x': round(f['x'], 2),
        'y': round(f['y'], 2),
        'line': f['line'],
        'route_state': f['state'],
    }


def run_experiment(exp):
    tag = exp['tag']
    apply_config(exp.get('config', {}))
    ROUTE.write_text(ROUTE_BASELINE.read_text())
    FOLLOW.write_text((FOLLOW_ORIG if exp.get('line_follow') == 'orig' else FOLLOW_BASELINE).read_text())
    apply_constants(ROUTE, ROUTE_BASELINE, exp.get('route', {}))
    apply_constants(FOLLOW, FOLLOW_BASELINE, exp.get('follow', {}))
    apply_manifest(exp.get('cruise', 60), exp.get('turn', 16), exp.get('table'))
    with cf.ThreadPoolExecutor(max_workers=2) as pool:
        res = list(pool.map(lambda s: run_scheme(s, tag), exp.get('schemes', [1, 2])))
    row = {'tag': tag, 'config': exp.get('config', {}), 'route': exp.get('route', {}), 'follow': exp.get('follow', {}), 'table': exp.get('table', {}),
           'cruise': exp.get('cruise', 60), 'turn': exp.get('turn', 16),
           'schemes': {r['scheme']: r for r in res}}
    ok = all(r.get('ok') and r['state'] == 'ROUTE_FINISHED' and r['failure'] == 0
             and r['index'] == r['last_index'] for r in res)
    row['both_finished'] = ok
    return row


def main():
    exps = json.loads(Path(sys.argv[1]).read_text())
    out_path = Path(sys.argv[2])
    with out_path.open('w') as fh:
        for exp in exps:
            try:
                row = run_experiment(exp)
            except Exception as e:  # noqa: BLE001 - report and keep sweeping
                row = {'tag': exp.get('tag'), 'error': str(e), 'both_finished': False}
            fh.write(json.dumps(row, ensure_ascii=False) + '\n')
            fh.flush()
            parts = []
            for s, r in row.get('schemes', {}).items():
                if not r.get('ok'):
                    parts.append(f's{s}:RUN-FAIL')
                else:
                    parts.append(f"s{s}:{'OK' if r['state']=='ROUTE_FINISHED' and r['failure']==0 and r['index']==r['last_index'] else 'FAIL'}"
                                 f" {r['sec']}s idx{r['index']}/{r['last_index']} {r['failure_name']}")
            print(f"{row['tag']:<52} {'PASS' if row.get('both_finished') else 'fail'}  " + ' | '.join(parts), flush=True)


if __name__ == '__main__':
    main()
