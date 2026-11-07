#!/usr/bin/env python3
"""Generate route_table.c from a manifest, same format as
car_stm32/test/sync_new_track_routes.py but writing into the sweep work dir only.

Usage: python3 gentable.py <manifest.json> <out.c> [cruise] [turn]
"""
import json
import sys
from pathlib import Path

manifest_path = Path(sys.argv[1])
out_path = Path(sys.argv[2])
cruise_cli = int(sys.argv[3]) if len(sys.argv) > 3 else None
turn_cli = float(sys.argv[4]) if len(sys.argv) > 4 else None

manifest = json.loads(manifest_path.read_text())
margin = manifest['distance_margin_cm']
cruise_default = cruise_cli if cruise_cli is not None else int(manifest.get('cruise_speed_cm_s', 60))
turn_default = turn_cli if turn_cli is not None else float(manifest.get('turn_speed_cm_s', 16.0))
assert margin == 5
assert turn_default <= cruise_default, (turn_default, cruise_default)

header = '''#include "route.h"

/* 字段：最多路口数、转弯速度、动作、巡航速度、右正左负角度、圆弧开关、总段距离、短段几何长度。
 * 本文件由 sweep 脚本从 manifest 生成，仅用于仿真器参数扫描。 */
#ifndef ROUTE_SCHEME_SELECT
#define ROUTE_SCHEME_SELECT 0
#endif

const RouteStep route[] = {
'''


TURN_OVERRIDES = {}
CRUISE_OVERRIDES = {}


def row(count, angle, geometric, note, turn=None, cruise=None, short_link=0, row_index=None):
    if turn is None:
        turn = turn_default
    if cruise is None:
        cruise = cruise_default
    action = 'STOP' if angle == 0 else 'HEXAGON' if abs(angle) > 180 else 'CUSTOM_TURN'
    length = geometric - margin
    assert length > 0
    return (f'    {{ {count}, {turn:.1f}f, {action:11s}, {cruise:3}, {angle:7.1f}f, false, {length:6.1f}f, {short_link:.1f}f}},'
            f' /* {note}；几何 {geometric:g} - 5 = {length:g} cm */\n')


body = '#if (ROUTE_SCHEME_SELECT == 0)\n'
for n, (count, cruise, angle, length) in enumerate([(1, 50, -90, 60), (4, 200, -90, 180), (2, 200, -90, 60), (3, 200, 0, 200)]):
    body += row(count, angle, length, f'调试 {n}', turn=50, cruise=cruise)
for key, scheme in manifest['schemes'].items():
    body += f'\n#elif (ROUTE_SCHEME_SELECT == {key})\n    /* {scheme["name"]} */\n'
    TURN_OVERRIDES.clear(); TURN_OVERRIDES.update({int(k): float(v) for k, v in scheme.get('turn_speed_overrides', {}).items()})
    CRUISE_OVERRIDES.clear(); CRUISE_OVERRIDES.update({int(k): int(v) for k, v in scheme.get('cruise_speed_overrides', {}).items()})
    for n, (count, angle, geometric, note) in enumerate(scheme['steps']):
        short_link = geometric if n in scheme.get('short_link_steps', []) else 0
        if short_link:
            assert n > 0 and 20 <= geometric <= 40 and abs(angle) == 90
            assert abs(scheme['steps'][n - 1][1]) == 90
        body += row(count, angle, geometric, f'{n}: {note}', short_link=short_link,
                    turn=TURN_OVERRIDES.get(n), cruise=CRUISE_OVERRIDES.get(n))
body += '\n#elif (ROUTE_SCHEME_SELECT == 3)\n    /* 外框测试圈，保留用于基本回归。 */\n'
for n, (count, angle, length) in enumerate([(2, 90, 110), (11, 90, 375), (5, 90, 220), (6, 90, 375), (2, 0, 110)]):
    body += row(count, angle, length, f'外框 {n}')
c_source = header + body + '\n#endif\n};\n\nconst size_t route_count = sizeof(route) / sizeof(route[0]);\n'
out_path.write_text(c_source)
print(f'wrote {out_path} (cruise={cruise_default}, turn={turn_default})')
