"""Verify A6 revision scope, physical envelopes, net mapping and exports.

Run with Python 3; independent copper check additionally needs Shapely 2.x.
"""
import csv
import io
import json
import math
import re
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BASE = ROOT.parent / 'stm32PCB-A5'
state = json.loads((ROOT / 'pcb-state.json').read_text())
baseline = json.loads((BASE / 'pcb-state.json').read_text())
parts = {p['component']['designator']: p for p in state['parts']}
old_parts = {p['component']['designator']: p for p in baseline['parts']}
live_baseline = {c['designator']: c for c in json.loads((ROOT / 'baseline-live-parts.json').read_text())}
assert set(parts) == set(old_parts) and len(parts) == 32
assert state['drc'] == json.loads((ROOT / 'pcb-drc.json').read_text()) == []
assert state['rules'] == baseline['rules'], 'Design rules changed'
reopened = json.loads((ROOT / 'reopened-check.json').read_text())
assert reopened['inductor']['x'] == 470 and reopened['pcb_drc'] == []

def clean(obj):
    return {k: v for k, v in obj.items() if k != 'async'}

def component_properties(obj):
    obj = json.loads(json.dumps(clean(obj)))
    # Folder-project library IDs change with the project path. Check the
    # concrete device/footprint UUIDs and all other properties independently.
    for key in ['component', 'footprint']:
        obj[key]['libraryUuid'] = '<project-local>'
    return obj

changed_components = []
checked_pins = 0
netlist = json.loads((ROOT / 'stm32PCB-A6.netlist.json').read_text())
schematic = {c['props']['Designator']: c for c in netlist['components'].values()}
for name, part in parts.items():
    a, b = component_properties(live_baseline[name]), component_properties(part['component'])
    if a != b:
        changed_components.append(name)
        assert name == 'L1' and b['x'] == 470
        assert {k: v for k, v in a.items() if k != 'x'} == {k: v for k, v in b.items() if k != 'x'}
    old_pads = {p['padNumber']: p for p in old_parts[name]['pads']}
    for pad in part['pads']:
        checked_pins += 1
        assert pad['net'] == schematic[name]['pinInfoMap'][pad['padNumber']]['net']
        if name != 'L1':
            assert math.dist((pad['x'], pad['y']), (old_pads[pad['padNumber']]['x'], old_pads[pad['padNumber']]['y'])) < .01
assert changed_components == ['L1'] and checked_pins == 143

old_lines = {p['primitiveId']: clean(p) for p in baseline['lines']}
lines = {p['primitiveId']: clean(p) for p in state['lines']}
# EDA normalized one near-horizontal edited segment on save, assigning it
# a new primitive ID. Verify the replacement rather than ignoring additions.
replacements = {'254810a2704a2a56': '49f454e936f96854'}
assert set(lines) == (set(old_lines) - set(replacements)) | set(replacements.values())
matched = {key: lines[replacements.get(key, key)] for key in old_lines}
changed_lines = [key for key in matched if matched[key] != old_lines[key]]
assert set(changed_lines) == {'c87f3e730f4bad89', '77cd383f44f12dbf', 'fe1b4d434c5d829f', '254810a2704a2a56'}
for key in changed_lines:
    assert matched[key]['lineWidth'] == old_lines[key]['lineWidth']
    assert matched[key]['net'] == old_lines[key]['net']
assert any(s['text'] == 'REAR / STM32 CAR A6' for s in state['strings'])

# Maximum body dimensions from Bourns SRP1038A and Samsung CL31A226KAHNNN.
# This check covers body size tolerance, not factory placement tolerances.
def bounds(name, maximum):
    c = parts[name]['component']
    w, h = ((10.3, 11.5) if maximum else (10, 11)) if name == 'L1' else ((3.4, 1.8) if maximum else (3.2, 1.6))
    x, y = c['x'] * .0254, c['y'] * .0254
    return x-w/2, y-h/2, x+w/2, y+h/2

def distance(a, b):
    return math.hypot(max(a[0]-b[2], b[0]-a[2], 0), max(a[1]-b[3], b[1]-a[3], 0))

clearances = {}
for name in ['C5', 'C6']:
    nominal = distance(bounds('L1', False), bounds(name, False))
    maximum = distance(bounds('L1', True), bounds(name, True))
    assert maximum > 2, f'Insufficient body clearance: {name}'
    clearances[name] = {'nominal_body_gap_mm': nominal, 'maximum_body_size_gap_mm': maximum}

# Read current native source, including tombstones left by EDA saves.
native = {}
for row in (ROOT / 'stm32PCB/pcb/PCB1.epcb2').read_text().splitlines():
    if '||' not in row:
        continue
    header, payload = row.split('||', 1)
    header = json.loads(header)
    if payload.rstrip('|'):
        native[(header['type'], header.get('id'))] = json.loads(payload.rstrip('|'))
assert native['COMPONENT', parts['L1']['component']['primitiveId']]['x'] == 470
for key in changed_lines:
    actual = native['LINE', replacements.get(key, key)]
    for field in ['startX', 'startY', 'endX', 'endY']:
        assert abs(actual[field] - matched[key][field]) < .01

for path in (ROOT / 'stm32PCB/sch').rglob('*'):
    if path.is_file() and not path.name.startswith('.'):
        assert path.read_bytes() == (BASE / 'stm32PCB/sch' / path.relative_to(ROOT / 'stm32PCB/sch')).read_bytes()

def read_table(path):
    raw = path.read_bytes()
    text = raw.decode('utf-16' if raw.startswith((b'\xff\xfe', b'\xfe\xff')) else 'utf-8-sig')
    return list(csv.DictReader(io.StringIO(text), delimiter='\t'))

pnp = {r['Designator']: r for r in read_table(ROOT / 'stm32PCB-A6-PickAndPlace.csv')}
assert set(pnp) == set(parts)
for name, part in parts.items():
    c = part['component']
    for key, axis in [('Mid X', 'x'), ('Mid Y', 'y')]:
        assert abs(float(pnp[name][key].removesuffix('mm')) - c[axis]*.0254) < .002
assert float(pnp['L1']['Rotation']) % 360 == 270
bom = read_table(ROOT / 'stm32PCB-A6-BOM.csv')
assert sum(int(r['Quantity']) for r in bom) == 32
assert {n for r in bom for n in r['Designator'].split(',')} == set(parts)

with zipfile.ZipFile(ROOT / 'stm32PCB-A6-REVIEW-ONLY-Gerber.zip') as archive:
    assert archive.testzip() is None
    top = archive.read('Gerber_TopLayer.GTL').decode()
    apertures = {m[0]: (float(m[1]), float(m[2])) for m in re.findall(r'%ADD(\d+)R,([\d.]+)X([\d.]+)\*%', top)}
    flashes = []
    aperture = None
    for row in top.splitlines():
        select = re.fullmatch(r'(?:G54)?D(\d+)\*', row)
        if select:
            aperture = select[1]
        flash = re.fullmatch(r'(?:G01)?X(-?\d+)Y(-?\d+)D03\*', row)
        if flash and aperture in apertures:
            flashes.append((int(flash[1])/100000, int(flash[2])/100000, apertures[aperture]))
    for pad in parts['L1']['pads']:
        assert any(math.dist((x,y), (pad['x']*.0254,pad['y']*.0254)) < .005 and abs(size[0]-3.5)<.01 and abs(size[1]-3.5)<.01 for x,y,size in flashes), 'L1 pad missing in Gerber'
    outline = archive.read('Gerber_BoardOutlineLayer.GKO').decode()
    points = [(int(x)/100000,int(y)/100000) for x,y in re.findall(r'X(-?\d+)Y(-?\d+)D0[12]', outline)]
    assert abs(max(x for x,y in points)-min(x for x,y in points)-100)<.005
    assert abs(max(y for x,y in points)-min(y for x,y in points)-90)<.005

with zipfile.ZipFile(ROOT / 'stm32PCB-A6-project.zip') as archive:
    assert archive.testzip() is None
    for name in archive.namelist():
        assert archive.read(name) == (ROOT / name).read_bytes()

report = {'revision': 'A6', 'components': 32, 'checked_pin_nets': checked_pins,
    'pcb_drc': [], 'design_rules_unchanged': True, 'changed_components': changed_components,
    'changed_track_count': len(changed_lines), 'clearances': clearances,
    'schematics_byte_identical_to_A5': True, 'pnp_all_positions_match': True,
    'gerber_L1_pad_positions_verified': True, 'native_archive_matches_disk': True,
    'hardware_tested': False, 'factory_assembly_reviewed': False}
(ROOT / 'verification.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
