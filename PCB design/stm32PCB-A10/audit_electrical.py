"""Read-only A10 electrical audit of the saved native PCB; requires Shapely 2.x.

Prints JSON to stdout. Does not alter manufacturing files or operate hardware.
Replays native object updates/deletions, including tracks omitted by the rounded
editor snapshot. Connectivity is a geometric check, not a load/thermal model.
"""
import copy
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path

from shapely.geometry import LineString, Point, Polygon
from shapely.strtree import STRtree

ROOT = Path(__file__).resolve().parent
SCALE = .0254


def require(condition, message):
    if not condition:
        raise ValueError(message)


# Reuse only the pad shape converter; this module has no file-writing side effect
# until its final report. Avoid executing it by extracting its function AST.
import ast
source = ast.parse((ROOT / 'verify_copper.py').read_text())
function = next(node for node in source.body
                if isinstance(node, ast.FunctionDef) and node.name == 'pad_shape')
from shapely import affinity
from shapely.geometry import box
scope = dict(affinity=affinity, Point=Point, LineString=LineString, box=box,
             SCALE=SCALE)
exec(compile(ast.Module(body=[function], type_ignores=[]),
             str(ROOT / 'verify_copper.py'), 'exec'), scope)
pad_shape = scope['pad_shape']

documents = []
current = None
for line in (ROOT / 'stm32PCB/pcb/PCB1.epcb2').read_text().splitlines():
    if '||' not in line:
        continue
    metadata, payload = line.split('||', 1)
    metadata = json.loads(metadata)
    payload = json.loads(payload.rstrip('|')) if payload.rstrip('|') else None
    if metadata['type'] == 'DOCHEAD':
        current = {'head': payload, 'records': {}}
        documents.append(current)
    if current is not None:
        current['records'][(metadata['type'], metadata.get('id'))] = payload

pcb = next(d['records'] for d in documents if d['head']['docType'] == 'PCB')
footprints = {d['head']['uuid']: d['records'] for d in documents
              if d['head']['docType'] == 'FOOTPRINT'}
state = json.loads((ROOT / 'pcb-state.json').read_text())
netlist = json.loads((ROOT / 'stm32PCB-A10.netlist.json').read_text())
schematic = {q['props']['Designator']: q for q in netlist['components'].values()}
native_components = {identifier: value for (kind, identifier), value in pcb.items()
                     if kind == 'COMPONENT' and value is not None}
require(len(native_components) == 32, 'Native component count')
pad_nets = {}
for (kind, identifier), value in pcb.items():
    if kind == 'PAD_NET' and value is not None:
        _, component_id, number, pad_id = json.loads(identifier)
        pad_nets[(component_id, number, pad_id)] = value['padNet']

nets = defaultdict(list)
pin_count = 0
pad_report = {}
for part in state['parts']:
    identifier = part['component']['primitiveId']
    name = part['component']['designator']
    component = native_components[identifier]
    attrs = {v['key']: v['value'] for (t, _), v in pcb.items()
             if t == 'ATTR' and v is not None and v.get('parentId') == identifier}
    require(attrs['Designator'] == name, f'Native designator: {name}')
    definitions = {v['num']: (i, v) for (t, i), v
                   in footprints[attrs['Footprint']].items()
                   if t == 'PAD' and v is not None}
    require(set(definitions) == set(schematic[name]['pinInfoMap']),
            f'Native footprint pin set: {name}')
    for cached in part['pads']:
        number = cached['padNumber']
        pad_id, definition = definitions[number]
        angle = math.radians(component['angle'])
        x = component['x'] + definition['centerX'] * math.cos(angle) - definition['centerY'] * math.sin(angle)
        y = component['y'] + definition['centerX'] * math.sin(angle) + definition['centerY'] * math.cos(angle)
        require(math.hypot(x - cached['x'], y - cached['y']) * SCALE < .003,
                f'Native pad transform: {name}.{number}')
        net = pad_nets[(identifier, number, pad_id)]
        require(net == cached['net'] == schematic[name]['pinInfoMap'][number]['net'],
                f'Native/snapshot/schematic net: {name}.{number}')
        pad = copy.deepcopy(cached)
        pad.update(x=x, y=y, rotation=component['angle'] + definition['padAngle'])
        require(abs((pad['rotation'] - cached['rotation'] + 180) % 360 - 180) < .001,
                f'Native pad rotation: {name}.{number}')
        shape = definition['defaultPad']
        require(all(abs(shape[field] - cached['pad'][i]) * SCALE < .003
                    for i, field in [(1, 'width'), (2, 'height')]),
                f'Native pad dimensions: {name}.{number}')
        pad['pad'] = [shape['padType'], shape['width'], shape['height']]
        if net:
            nets[net].append((f'{name}.{number}', [1, 2] if cached['layer'] == 12 else [cached['layer']], pad_shape(pad)))
        pad_report[f'{name}.{number}'] = dict(net=net, x_mm=x*SCALE, y_mm=y*SCALE)
        pin_count += 1
require(pin_count == 143, 'Native pin count')

counts = defaultdict(int)
for (kind, identifier), value in pcb.items():
    if value is None:
        continue
    if kind == 'LINE' and value['layerId'] in [1, 2]:
        geometry = LineString([(value['startX']*SCALE, value['startY']*SCALE),
                               (value['endX']*SCALE, value['endY']*SCALE)]).buffer(value['width']*SCALE/2)
        nets[value['netName']].append((identifier, [value['layerId']], geometry))
        counts['tracks'] += 1
    elif kind == 'VIA':
        geometry = Point(value['centerX']*SCALE, value['centerY']*SCALE).buffer(value['viaDiameter']*SCALE/2)
        nets[value['netName']].append((identifier, [1, 2], geometry))
        counts['vias'] += 1
    elif kind == 'FILL':
        rings = []
        for path in value['path']:
            require(path[2] == 'L', 'Unsupported native fill path')
            numbers = path[:2] + path[3:]
            rings.append([(numbers[i]*SCALE, numbers[i+1]*SCALE)
                          for i in range(0, len(numbers), 2)])
        geometry = Polygon(rings[0], rings[1:])
        require(geometry.is_valid, 'Invalid native fill')
        nets[value['netName']].append((identifier, [value['layerId']], geometry))
        counts['fills'] += 1
require('' not in nets, 'Unexpected unassigned routing')

disconnected = {}
for net, items in nets.items():
    parent = list(range(len(items)))
    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for layer in [1, 2]:
        indices = [i for i, (_, layers, _) in enumerate(items) if layer in layers]
        shapes = [items[i][2].buffer(.0025) for i in indices]
        tree = STRtree(shapes)
        for local, geometry in enumerate(shapes):
            for other in tree.query(geometry, predicate='intersects'):
                parent[find(indices[local])] = find(indices[int(other)])
    groups = defaultdict(list)
    for i, item in enumerate(items):
        groups[find(i)].append(item[0])
    if len(groups) > 1:
        disconnected[net] = list(groups.values())
require(not disconnected, f'Disconnected native copper: {disconnected}')
shorts = []
for layer in [1, 2]:
    items = [(net, identifier, geometry) for net, objects in nets.items()
             for identifier, layers, geometry in objects if layer in layers]
    tree = STRtree([q[2] for q in items])
    for i, (net, identifier, geometry) in enumerate(items):
        for j in tree.query(geometry, predicate='intersects'):
            j = int(j)
            if j > i and net != items[j][0] and geometry.intersection(items[j][2]).area > .0001:
                shorts.append((layer, identifier, items[j][1]))
require(not shorts, f'Native copper shorts: {shorts}')

def distance(a, b):
    return round(math.hypot(pad_report[a]['x_mm'] - pad_report[b]['x_mm'],
                           pad_report[a]['y_mm'] - pad_report[b]['y_mm']), 4)

report = {
    'revision': 'A10', 'date': '2026-10-17',
    'scope': 'saved native PCB, schematic netlist and supplied A10 manufacturing package; no order-portal access',
    'components_checked': len(native_components), 'native_pin_assignments_checked': pin_count,
    'native_copper': dict(counts), 'connected_nets': len(nets),
    'disconnected_copper_groups': 0, 'shorted_net_pairs': 0,
    'connectivity_coordinate_tolerance_mm': .005,
    'short_detection_minimum_overlap_area_mm2': .0001,
    'native_extra_tracks_vs_rounded_snapshot': counts['tracks'] - len(state['lines']),
    'nominal_output_voltage_V': .596 * (1 + 100/13.3),
    'layout_pad_center_distances_mm': {
        'U4_VIN_to_C7_VIN': distance('U4.3', 'C7.1'),
        'U4_VIN_to_C5_VIN': distance('U4.3', 'C5.1'),
        'U4_GND_to_C9_GND': distance('U4.1', 'C9.2'),
        'U4_FB_to_R2_FB': distance('U4.4', 'R2.2'),
    },
    'critical_pad_nets': {key: pad_report[key]['net'] for key in pad_report
                          if key.split('.')[0] in ['U4', 'D1', 'C8', 'R2', 'R3', 'R4', 'J8', 'J9', 'J10']},
    'sha256': {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in [
        'stm32PCB/pcb/PCB1.epcb2', 'stm32PCB-A10.netlist.json',
        'stm32PCB-A10-REVIEW-ONLY-Gerber.zip', 'stm32PCB-A10-SMT-BOM.csv',
        'stm32PCB-A10-PickAndPlace.csv']},
    'actual_order_bom_substitutions_verified': False,
    'hardware_tested': False, 'power_load_and_thermal_test_completed': False,
}
print(json.dumps(report, ensure_ascii=False, indent=2))
