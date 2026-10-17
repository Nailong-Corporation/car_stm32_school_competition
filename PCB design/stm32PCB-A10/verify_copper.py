"""Independent copper geometry verification. Requires Shapely 2.x.
Run after verify_design.py, using the saved native project and pcb-state.json.
"""
import json
from collections import defaultdict
from pathlib import Path
from shapely import affinity
from shapely.geometry import Point, LineString, Polygon, box
from shapely.strtree import STRtree

ROOT = Path(__file__).resolve().parent
SCALE = .0254
TOLERANCE = .005  # Native/DSN coordinate rounding; not a clearance allowance.
state = json.loads((ROOT / 'pcb-state.json').read_text())

def pad_shape(pad):
    kind, width, height, *_ = pad['pad']
    width *= SCALE
    height *= SCALE
    if kind == 'ELLIPSE':
        shape = affinity.scale(Point(0, 0).buffer(1, quad_segs=32), width / 2, height / 2)
    elif kind == 'OVAL':
        if width >= height:
            shape = LineString([(-(width-height)/2, 0), ((width-height)/2, 0)]).buffer(height/2)
        else:
            shape = LineString([(0, -(height-width)/2), (0, (height-width)/2)]).buffer(width/2)
    else:
        shape = box(-width/2, -height/2, width/2, height/2)
    return affinity.translate(affinity.rotate(shape, pad['rotation'], origin=(0, 0)), pad['x']*SCALE, pad['y']*SCALE)

# getAll() JSON serializes only the outer polygon. Read all hole rings from
# the saved native FILL records rather than treating these as solid rectangles.
fills = {}
wanted = {fill['primitiveId'] for fill in state['fills']}
for line in (ROOT / 'stm32PCB/pcb/PCB1.epcb2').read_text().splitlines():
    if '||' not in line:
        continue
    metadata, payload = line.split('||', 1)
    metadata = json.loads(metadata)
    if metadata.get('type') == 'FILL' and metadata.get('id') in wanted and payload.rstrip('|'):
        data = json.loads(payload.rstrip('|'))
        rings = []
        for source in data['path']:
            assert source[2] == 'L', 'Unsupported non-linear ground polygon'
            numbers = source[:2] + source[3:]
            rings.append([(numbers[i]*SCALE, numbers[i+1]*SCALE) for i in range(0, len(numbers), 2)])
        geometry = Polygon(rings[0], rings[1:])
        assert geometry.is_valid, 'Invalid ground polygon'
        fills[metadata['id']] = geometry
assert fills.keys() == wanted

nets = defaultdict(list)
def add(net, identifier, layers, shape):
    if net:
        nets[net].append((identifier, layers, shape))
for part in state['parts']:
    designator = part['component']['designator']
    for pad in part['pads']:
        add(pad['net'], f"{designator}.{pad['padNumber']}", [1,2] if pad['layer']==12 else [pad['layer']], pad_shape(pad))
for line in state['lines']:
    shape = LineString([(line['startX']*SCALE,line['startY']*SCALE),(line['endX']*SCALE,line['endY']*SCALE)]).buffer(line['lineWidth']*SCALE/2, quad_segs=16)
    add(line['net'], line['primitiveId'], [line['layer']], shape)
for via in state['vias']:
    add(via['net'], via['primitiveId'], [1,2], Point(via['x']*SCALE,via['y']*SCALE).buffer(via['diameter']*SCALE/2, quad_segs=16))
for fill in state['fills']:
    add(fill['net'], fill['primitiveId'], [fill['layer']], fills[fill['primitiveId']])

failures = {}
for net, items in nets.items():
    parent = list(range(len(items)))
    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    def union(i,j):
        parent[find(i)] = find(j)
    for layer in [1,2]:
        indices = [i for i, (_, layers, _) in enumerate(items) if layer in layers]
        shapes = [items[i][2].buffer(TOLERANCE/2) for i in indices]
        tree = STRtree(shapes)
        for local, shape in enumerate(shapes):
            for other in tree.query(shape, predicate='intersects'):
                union(indices[local], indices[int(other)])
    groups = defaultdict(list)
    for i, item in enumerate(items):
        groups[find(i)].append(item[0])
    if len(groups) != 1:
        failures[net] = list(groups.values())
assert not failures, f'Disconnected copper: {failures}'

# Every input via in the two VIN banks must touch a wide track on each layer.
via_checks = []
for via in state['vias']:
    if via['net'] != 'VIN_PROTECTED':
        continue
    position = Point(via['x']*SCALE, via['y']*SCALE)
    for layer in [1,2]:
        connected = [line for line in state['lines'] if line['net']==via['net'] and line['layer']==layer and line['lineWidth']*SCALE>=1.19 and LineString([(line['startX']*SCALE,line['startY']*SCALE),(line['endX']*SCALE,line['endY']*SCALE)]).distance(position)<.01]
        assert connected, f"Power via not attached to a wide track on layer {layer}: {via['primitiveId']}"
    assert via['holeDiameter'] >= 12
    via_checks.append(via['primitiveId'])
assert len(via_checks)==6
# Detect shorts independently across distinct net geometry. No spacing waiver.
shorts = []
for layer in [1,2]:
    items=[(net,identifier,shape) for net,objects in nets.items() for identifier,layers,shape in objects if layer in layers]
    tree=STRtree([x[2] for x in items])
    for i,(net,identifier,shape) in enumerate(items):
        for j in tree.query(shape,predicate='intersects'):
            j=int(j)
            if j>i and net!=items[j][0]:
                overlap=shape.intersection(items[j][2]).area
                if overlap>.0001:shorts.append((layer,net,identifier,items[j][0],items[j][1],overlap))
assert not shorts, f'Copper shorts: {shorts}'
report = {'independent_geometry_connected_nets':len(nets),'disconnected_copper_groups':0,'shorted_net_pairs':0,'vin_parallel_vias_connected_on_both_layers':len(via_checks),'native_ground_fill_holes_read':sum(len(g.interiors) for g in fills.values()),'coordinate_tolerance_mm':TOLERANCE}
(ROOT/'copper-verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
