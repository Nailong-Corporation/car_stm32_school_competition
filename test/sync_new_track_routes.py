#!/usr/bin/env python3
"""Generate C and bundled TS tables from geometric lengths, subtracting 5 cm once."""
import json
from pathlib import Path
import sys

root = Path(__file__).resolve().parent.parent
simulator = root.parents[2] / 'stm32car_simulator'
manifest = json.loads((root / 'test/new_track_routes.json').read_text())
margin = manifest['distance_margin_cm']
cruise_default = int(manifest.get('cruise_speed_cm_s', 60))
turn_default = float(manifest.get('turn_speed_cm_s', 16.0))
assert margin == 5
header = '''#include "route.h"

/* 字段：最多路口数、转弯速度、动作、巡航速度、右正左负角度、圆弧开关、总段距离、短段几何长度。
 * 普通行：计数包含当前节点；计数或距离任一达到后，仍须确认灰度节点才执行动作。
 * short_link_cm>0：明确标注的短连接，使用前一路口确认后的有界编码器/航向投影；长度不减5。
 * 电阻及其他印刷图形参与节点计数，使用连续证据及离口去重。
 * 所有行距离 = 图纸几何长度 - 5 cm；原始长度见 test/new_track_routes.json。
 * 传感器前伸 24 cm，短段节点可能在上一动作内通过，计数需逐段回放校准。
 * 方案 0：保留四步调试路线；1：走虚线全元件；2：不走虚线；3：外框测试圈。
 * 正式方案仍在仿真调试中，不能据表名宣称已经完赛或实车验证。
 * 地图：375x220，Q6(245,140)，右中线 y110，环岛中心(310,165)，P(0,110)。
 * 本文件由 test/sync_new_track_routes.py 生成。
 */
#ifndef ROUTE_SCHEME_SELECT
#define ROUTE_SCHEME_SELECT 0
#endif

const RouteStep route[] = {
'''

# Rolling corner (2026-11-08): ordinary 90 degree rows may run a tangent arc of
# radius CORNER_ARC_RADIUS_CM instead of a stopped pivot.  The predicate mirrors
# src/route.c corner_arc_row(): no explicit short link on this row or the next,
# and the next row must leave room for the arc exit tangent (2R + margin).
CORNER_ARC_RADIUS_CM = 15.0
CORNER_ARC_MIN_LEG_MARGIN_CM = 4.0


def arc_flag(angle, length, next_length, short_link, next_short_link):
    if short_link or next_short_link:
        return False
    if next_length is not None and next_length + margin < 2.0 * CORNER_ARC_RADIUS_CM + CORNER_ARC_MIN_LEG_MARGIN_CM:
        return False
    return abs(angle) == 90


def row(count, angle, geometric, note, turn=None, cruise=60, short_link=0, arc=False, action=None):
    if turn is None:
        turn = globals().get('turn_speed_default', 16)
    if action is None:
        action = 'STOP' if angle == 0 else 'HEXAGON' if abs(angle) > 180 else 'CUSTOM_TURN'
    length = geometric - margin
    assert length > 0
    arc_text = 'true ' if arc else 'false'
    return (f'    {{ {count}, {turn:.1f}f, {action:11s}, {cruise:3}, {angle:7.1f}f,{arc_text}, {length:6.1f}f, {short_link:.1f}f}},'
            f' /* {note}；几何 {geometric:g} - 5 = {length:g} cm */\n')

body = '#if (ROUTE_SCHEME_SELECT == 0)\n'
debug_rows = [(1,50,-90,60),(4,200,-90,180),(2,200,-90,60),(3,200,0,200)]
for n, (count, cruise, angle, length) in enumerate(debug_rows):
    nxt = debug_rows[n+1][3] - margin if n+1 < len(debug_rows) else None
    body += row(count, angle, length, f'调试 {n}', turn=50, cruise=cruise,
                arc=arc_flag(angle, length, nxt, 0, 0))
for key, scheme in manifest['schemes'].items():
    body += f'\n#elif (ROUTE_SCHEME_SELECT == {key})\n    /* {scheme["name"]} */\n'
    scheme_cruise = int(scheme.get('cruise_speed_cm_s', cruise_default))
    scheme_turn = float(scheme.get('turn_speed_cm_s', turn_default))
    steps = scheme['steps']
    short_steps = scheme.get('short_link_steps', [])
    for n, step in enumerate(steps):
        count, angle, geometric, note = step[0], step[1], step[2], step[3]
        action = step[4] if len(step) > 4 else None
        short_link = geometric if n in short_steps else 0
        if short_link:
            assert n > 0 and 20 <= geometric <= 40 and abs(angle) == 90
            assert abs(steps[n-1][1]) == 90
        nxt = (steps[n+1][2] - margin) if n+1 < len(steps) else None
        next_short = (steps[n+1][2] if (n+1) in short_steps else 0) if n+1 < len(steps) else 0
        body += row(count, angle, geometric, f'{n}: {note}', turn=scheme_turn, cruise=scheme_cruise,
                    short_link=short_link, arc=arc_flag(angle, geometric-margin, nxt, short_link, next_short),
                    action=action)
c_source = header + body + '\n#endif\n};\n\nconst size_t route_count = sizeof(route) / sizeof(route[0]);\n'
parser_path = simulator / 'src/sim/route_parser.ts'
parser = parser_path.read_text()
marker = 'export const DEFAULT_ROUTE_TABLE_C = `'
start = parser.index(marker)
end = parser.index('`;', start) + 2
# Keep firmware default scheme 0; GUI opens the official dashed route.
bundled = c_source.replace('#define ROUTE_SCHEME_SELECT 0', '#define ROUTE_SCHEME_SELECT 1')
parser = parser[:start] + marker + bundled + '`;' + parser[end:]
outputs = {root / 'src/route_table.c': c_source, parser_path: parser}
checking = '--check' in sys.argv
for path, expected in outputs.items():
    if checking:
        assert path.read_text() == expected, f'out of sync: {path}'
    else:
        path.write_text(expected)
print('PASS: C/TS route tables match geometric lengths minus 5 cm' if checking else 'Generated C and TS route tables')
