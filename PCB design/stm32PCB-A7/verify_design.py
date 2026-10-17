import csv
import io
import json
import math
import re
import zipfile
from pathlib import Path


root = Path(__file__).resolve().parent
native = json.loads((root / 'stm32PCB-A7.netlist.json').read_text())
pcb = json.loads((root / 'pcb-state.json').read_text())
expected = {
    'J1': ['+5V', 'GND'],
    'J2': ['VM', 'GND'],
    'J3': ['AO1', 'AO2'],
    'J4': ['BO1', 'BO2'],
    'SW1': ['BTN', 'BTN', 'GND', 'GND'],
    'R1': ['STBY', 'GND'],
    'C1': ['VM', 'GND'],
    'C2': ['VM', 'GND'],
    'C3': ['+3V3', 'GND'],
    'C4': ['+3V3', 'GND'],
    'J6': ['VBAT_EXT', 'PC13', 'PC14', 'PC15', 'S0', 'S1', 'S2', 'S3', 'S4', 'S5', 'PWMA', 'PWMB', 'S6', 'S7', 'BTN', 'PB11', 'NRST', '+3V3', 'GND', 'GND'],
    'J7': ['AIN1', 'AIN2', 'BIN1', 'BIN2', 'PA8', 'PA9', 'PA10', 'PA11', 'PA12', 'PA15', 'PB3', 'PB4', 'STBY', 'PB6', 'PB7', 'PB8', 'PB9', '+5V', 'GND', '+3V3'],
    'J8': ['VM', '+3V3', 'GND', 'AO1', 'AO2', 'BO2', 'BO1', 'GND'],
    'J9': ['PWMA', 'AIN2', 'AIN1', 'STBY', 'BIN1', 'BIN2', 'PWMB', 'GND'],
    'J10': ['+3V3', 'GND', 'S0', 'S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'S7'],
    'J11': ['PC13', 'PC14', 'PC15', 'PA8', 'PA9', 'PA10', 'PA11', 'PA12', 'PA15', 'GND'],
    'J12': ['PB3', 'PB4', 'PB6', 'PB7', 'PB8', 'PB9', 'PB11', 'NRST', 'VBAT_EXT', '+3V3'],
    'J13': ['DC_RAW', 'GND', ''],
    'F1': ['DC_RAW', 'DC_FUSED'],
    'D1': ['VIN_PROTECTED', 'DC_FUSED'],
    'U4': ['GND', 'BUCK_SW', 'VIN_PROTECTED', 'BUCK_FB', '', 'BUCK_BOOT'],
    'L1': ['BUCK_SW', 'VM'],
    'C5': ['VIN_PROTECTED', 'GND'],
    'C6': ['VIN_PROTECTED', 'GND'],
    'C7': ['VIN_PROTECTED', 'GND'],
    'C8': ['BUCK_BOOT', 'BUCK_SW'],
    'C9': ['VM', 'GND'],
    'C10': ['VM', 'GND'],
    'C11': ['VM', 'BUCK_FB'],
    'R2': ['VM', 'BUCK_FB'],
    'R3': ['BUCK_FB', 'GND'],
    'R4': ['VM', '+5V'],
}


def require(condition, description):
    if not condition:
        raise ValueError(description)


components = {component['props']['Designator']: component for component in native['components'].values()}
pcb_parts = {part['component']['designator']: part for part in pcb['parts']}
require(set(components) == set(expected) == set(pcb_parts), 'Unexpected or missing components')
checked_pins = 0
nets = set()
for designator, expected_nets in expected.items():
    component = components[designator]
    properties = component['props']
    require(properties.get('Footprint') and properties.get('FootprintName'), f'Missing footprint: {designator}')
    require(properties['Add into BOM'] == 'yes' and properties['Convert to PCB'] == 'yes', f'Manufacturing flags: {designator}')
    schematic_pins = component['pinInfoMap']
    pcb_pads = {pad['padNumber']: pad for pad in pcb_parts[designator]['pads']}
    numbers = {str(number) for number in range(1, len(expected_nets) + 1)}
    require(set(schematic_pins) == set(pcb_pads) == numbers, f'Pin numbering mismatch: {designator}')
    for number, expected_net in enumerate(expected_nets, 1):
        require(schematic_pins[str(number)]['net'] == expected_net, f'Schematic net mismatch: {designator}-{number}')
        require(pcb_pads[str(number)]['net'] == expected_net, f'PCB net mismatch: {designator}-{number}')
        checked_pins += 1
        if expected_net:
            nets.add(expected_net)


def pad_position(designator, number):
    pad = next(pad for pad in pcb_parts[designator]['pads'] if pad['padNumber'] == str(number))
    return pad['x'] * .0254, pad['y'] * .0254


def close(actual, target, description):
    require(abs(actual - target) < .005, f'{description}: {actual} != {target}')


for designator in ['J6', 'J7', 'J8', 'J9', 'J10', 'J11', 'J12']:
    points = [pad_position(designator, number) for number in range(1, len(expected[designator]) + 1)]
    for previous, current in zip(points, points[1:]):
        close(math.dist(previous, current), 2.54, f'{designator} pitch')
        close(previous[1], current[1], f'{designator} horizontal orientation')
close(pad_position('J6', 1)[1] - pad_position('J7', 20)[1], 16, 'STM32 row center spacing')
close(pad_position('J8', 1)[1] - pad_position('J9', 1)[1], 16, 'TB6612 row center spacing')
close(pad_position('J6', 1)[0], pad_position('J7', 20)[0], 'STM32 opposite row endpoints')
close(pad_position('J8', 1)[0], pad_position('J9', 1)[0], 'TB6612 opposite row endpoints')
# IMG_0962: component side up, USB at left, SWD at right. Mirroring
# each row is intentional; logical pin-to-net assignments stay unchanged.
photo_top = list(reversed(expected['J6']))
photo_bottom = expected['J7']
for header, photo in [('J6', photo_top), ('J7', photo_bottom)]:
    ordered = sorted(pcb_parts[header]['pads'], key=lambda pad: pad['x'])
    require([pad['net'] for pad in ordered] == photo, f'{header} photo left-to-right pin order')
    require((pad_position(header, 20)[0] - pad_position(header, 1)[0]) * (-1 if header == 'J6' else 1) > 48, f'{header} orientation')
    silk_y = 61 if header == 'J6' else 39
    silk = [item for item in pcb['strings'] if item['layer'] == 3 and abs(item['y'] * .0254 - silk_y) < .005]
    ordered_silk = sorted(silk, key=lambda item: item['x'])
    expected_labels = {
        'J6': ['GND', 'GND', '3V3', 'NRST', 'PB11', 'PB10', 'PB1', 'PB0', 'PA7', 'PA6', 'PA5', 'PA4', 'PA3', 'PA2', 'PA1', 'PA0', 'PC15', 'PC14', 'PC13', 'VBAT'],
        'J7': ['PB12', 'PB13', 'PB14', 'PB15', 'PA8', 'PA9', 'PA10', 'PA11', 'PA12', 'PA15', 'PB3', 'PB4', 'PB5', 'PB6', 'PB7', 'PB8', 'PB9', '5V', 'GND', '3V3'],
    }
    require([item['text'] for item in ordered_silk] == expected_labels[header], f'{header} assembly silkscreen order')
    for pad, item in zip(ordered, ordered_silk):
        close(pad['x'] * .0254, item['x'] * .0254, f'{header} silkscreen registration')

gray_left_to_right = sorted(pcb_parts['J10']['pads'], key=lambda pad: pad['x'])
require([pad['net'] for pad in gray_left_to_right] == ['S7', 'S6', 'S5', 'S4', 'S3', 'S2', 'S1', 'S0', 'GND', '+3V3'], 'Gray left-to-right order')
require(pad_position('J10', 1)[1] > pad_position('J11', 1)[1] > pad_position('J6', 1)[1] > pad_position('J7', 1)[1] > pad_position('J8', 1)[1] > pad_position('J9', 1)[1], 'Front-to-rear module arrangement')
unused = set(expected['J11'][:-1] + expected['J12'][:-1])
require(len(unused) == 18, 'Unused pin count')
for net in unused:
    require(sum(net in expected[header] for header in ['J6', 'J7']) == 1, f'Missing MCU socket connection: {net}')
silkscreen = {item['text'] for item in pcb['strings'] if item['layer'] == 3}
require((unused - {'VBAT_EXT'}) | {'VBAT'} <= silkscreen, 'Missing unused-pin silkscreen labels')
for filename in ['pcb-drc.json']:
    require(json.loads((root / filename).read_text()) == [], f'Archived DRC violations: {filename}')
require(pcb['drc'] == [], 'PCB state contains DRC violations')

with zipfile.ZipFile(root / 'stm32PCB-A7-REVIEW-ONLY-Gerber.zip') as archive:
    require(archive.testzip() is None, 'Gerber archive CRC failure')
    required = {'Gerber_TopLayer.GTL', 'Gerber_BottomLayer.GBL', 'Gerber_TopSilkscreenLayer.GTO', 'Gerber_TopSolderMaskLayer.GTS', 'Gerber_BottomSolderMaskLayer.GBS', 'Gerber_BoardOutlineLayer.GKO', 'Drill_PTH_Through.DRL', 'Drill_NPTH_Through.DRL'}
    require(required <= set(archive.namelist()), 'Missing fabrication layers')
    outline = archive.read('Gerber_BoardOutlineLayer.GKO').decode()
    coordinates = [(int(x_value) / 100000, int(y_value) / 100000) for x_value, y_value in re.findall(r'X(-?\d+)Y(-?\d+)D0[12]', outline)]
    close(max(point[0] for point in coordinates) - min(point[0] for point in coordinates), 100, 'Gerber board width')
    close(max(point[1] for point in coordinates) - min(point[1] for point in coordinates), 90, 'Gerber board height')
    drill_text = archive.read('Drill_NPTH_Through.DRL').decode()
    require('T01C3.20000' in drill_text, 'Mounting hole diameter')
    holes = {(float(x_value), float(y_value)) for x_value, y_value in re.findall(r'^X([\d.]+)Y([\d.]+)$', drill_text, re.MULTILINE)}
    require(holes == {(4, 4), (96, 4), (4, 86), (96, 86)}, 'Mounting hole positions')
    plated = archive.read('Drill_PTH_Through.DRL').decode()
    tools = {}
    drill_points = []
    current_tool = None
    for line in plated.splitlines():
        definition = re.fullmatch(r'T(\d+)C([\d.]+)', line)
        selection = re.fullmatch(r'T(\d+)', line)
        coordinate = re.fullmatch(r'X([\d.]+)Y([\d.]+)', line)
        if definition:
            tools[definition[1]] = float(definition[2])
        elif selection:
            current_tool = selection[1]
        elif coordinate:
            drill_points.append((float(coordinate[1]), float(coordinate[2]), tools[current_tool]))
    header_holes = {}
    for designator in ['J6', 'J7', 'J8', 'J9', 'J10', 'J11', 'J12']:
        diameters = set()
        for number in range(1, len(expected[designator]) + 1):
            position = pad_position(designator, number)
            point = min(drill_points, key=lambda point: math.dist(position, point[:2]))
            require(math.dist(position, point[:2]) < .005, f'Missing Gerber drill: {designator}-{number}')
            require(1 <= point[2] <= 1.25, f'Unexpected connector drill: {designator}-{number}')
            diameters.add(point[2])
        header_holes[designator] = sorted(diameters)
with zipfile.ZipFile(root / 'stm32PCB-A7-project.zip') as archive:
    require(archive.testzip() is None, 'Project archive CRC failure')
    require('stm32PCB/stm32PCB.eprj3' in archive.namelist(), 'Missing native project entry')
    require('stm32PCB/pcb/PCB1.epcb2' in archive.namelist(), 'Missing native PCB source')
    for entry in archive.namelist():
        if entry.endswith('.esch2'):
            for line in archive.read(entry).decode().splitlines():
                if '\"type\":\"TEXT\"' not in line:
                    continue
                metadata, payload = line.split('||', 1)
                content = json.loads(payload.rstrip('|')).get('value', '')
                require('15.00mm' not in content and 'A3:' not in content and 'A3 /' not in content and 'PCB A4:' not in content and 'A4 /' not in content, f'Stale schematic note: {entry}')

# Routing checks matching the review requests. These are geometric targets,
# not current/temperature ratings.
power_nets = ['DC_RAW', 'DC_FUSED', 'VIN_PROTECTED', 'BUCK_SW', 'VM', '+5V', '+3V3']
widths = {net: sorted({round(line['lineWidth'] * .0254, 4) for line in pcb['lines'] if line['net'] == net}) for net in power_nets}
for net in ['DC_RAW', 'DC_FUSED']:
    require(min(widths[net]) >= 3.09, f'Main input width: {net}')
for net in ['VIN_PROTECTED', 'BUCK_SW', 'VM']:
    require(max(widths[net]) >= 3.09, f'Missing 3.1mm power trunk: {net}')
require(min(widths['+5V']) >= .94, 'MCU 5V branch width')
require(min(widths['+3V3']) >= .49, '3V3 distribution width')
for net in ['AO1', 'AO2', 'BO1', 'BO2']:
    tracks = [line for line in pcb['lines'] if line['net'] == net]
    require(tracks and {line['layer'] for line in tracks} == {2}, f'Motor layer: {net}')
    require(min(line['lineWidth'] * .0254 for line in tracks) >= 1.49, f'Motor width: {net}')
    require(not any(via['net'] == net for via in pcb['vias']), f'Motor layer-change via: {net}')
for net, count in [('VIN_PROTECTED', 6), ('+5V', 2)]:
    vias = [via for via in pcb['vias'] if via['net'] == net]
    require(len(vias) == count, f'Parallel power via count: {net}')
    require(all(via['holeDiameter'] * .0254 >= .39 for via in vias), f'Power via drill: {net}')
with zipfile.ZipFile(root / 'stm32PCB-A7-REVIEW-ONLY-Gerber.zip') as archive:
    via_drill = archive.read('Drill_PTH_Through_Via.DRL').decode()
    require(any(abs(float(value) - .40132) < .02 for value in re.findall(r'T\d+C([\d.]+)', via_drill)), '0.4mm power via drill missing from fabrication data')
    tools = {}
    points = []
    tool = None
    for row in via_drill.splitlines():
        definition = re.fullmatch(r'T(\d+)C([\d.]+)', row)
        selection = re.fullmatch(r'T(\d+)', row)
        coordinate = re.fullmatch(r'X([\d.]+)Y([\d.]+)', row)
        if definition:
            tools[definition[1]] = float(definition[2])
        elif selection:
            tool = selection[1]
        elif coordinate:
            points.append((float(coordinate[1]), float(coordinate[2]), tools[tool]))
    for via in pcb['vias']:
        if via['net'] not in ['VIN_PROTECTED', '+5V']:
            continue
        position = (via['x'] * .0254, via['y'] * .0254)
        match = min(points, key=lambda point: math.dist(position, point[:2]))
        require(math.dist(position, match[:2]) < .005 and match[2] >= .39, f"Power via drill missing in Gerber: {via['primitiveId']}")
    top = archive.read('Gerber_TopLayer.GTL').decode()
    bottom = archive.read('Gerber_BottomLayer.GBL').decode()
    require(any(abs(float(value)-3.1)<.005 for value in re.findall(r'%ADD\d+C,([\d.]+)', top)), '3.1mm top copper aperture missing')
    require(any(abs(float(value)-1.5)<.005 for value in re.findall(r'%ADD\d+C,([\d.]+)', bottom)), '1.5mm bottom copper aperture missing')
require(pcb['rules']['config']['Physics']['Track']['copperThickness1oz']['form']['data']['1']['maxValue'] == 3.81, 'Power track rule maximum')

u4 = pcb_parts['U4']
pads = {p['padNumber']: p for p in u4['pads']}
close(pads['1']['pad'][1] * .0254, .6, 'U4 pad width')
close(pads['1']['pad'][2] * .0254, 1.1, 'U4 pad length')
close(abs(pads['1']['y']-pads['6']['y']) * .0254, 2.7, 'U4 row center spacing')
close(abs(pads['1']['x']-pads['2']['x']) * .0254, .95, 'U4 lead pitch')
require(u4['component']['supplierId']=='C311983', 'U4 original TI part retained')
sw = pcb_parts['SW1']
require(sw['component']['supplierId']=='C318884', 'SW1 basic-library replacement')
require(sw['component']['manufacturerId']=='TS-1187A-B-A-B', 'SW1 manufacturer part')
require(sw['component']['otherProperty']['JLCPCB Part Class']=='Basic Part', 'SW1 library class')
sw_pads={p['padNumber']:p for p in sw['pads']}
close(abs(sw_pads['1']['x']-sw_pads['2']['x'])*.0254,6,'SW1 pad column spacing')
close(abs(sw_pads['1']['y']-sw_pads['3']['y'])*.0254,3.75,'SW1 manufacturer pad row spacing')
close(sw_pads['1']['pad'][1]*.0254,1,'SW1 pad width')
close(sw_pads['1']['pad'][2]*.0254,.75,'SW1 pad height')
require('USB <' in silkscreen and 'SWD >' in silkscreen, 'Core-board direction cues')
usb=next(t for t in pcb['strings'] if t['text']=='USB <')
swd=next(t for t in pcb['strings'] if t['text']=='SWD >')
require(usb['x'] < swd['x'] and not usb['mirror'] and not swd['mirror'], 'USB left / SWD right top view')
require(pcb['rules']==json.loads((root.parent/'stm32PCB-A6/pcb-state.json').read_text())['rules'],'Design rules unchanged')

# Check the exported manufacturing geometry, rather than only the editor cache.
with zipfile.ZipFile(root / 'stm32PCB-A7-REVIEW-ONLY-Gerber.zip') as archive:
    copper = archive.read('Gerber_TopLayer.GTL').decode()
    rectangles = {key: (float(w), float(h)) for key, w, h in
                  re.findall(r'%ADD(\d+)R,([\d.]+)X([\d.]+)\*%', copper)}
    selected = None
    flashes = []
    for row in copper.splitlines():
        selection = re.fullmatch(r'(?:G54)?D(\d+)\*', row)
        flash = re.fullmatch(r'(?:G01)?X(-?\d+)Y(-?\d+)D03\*', row)
        if selection:
            selected = selection[1]
        elif flash and selected in rectangles:
            flashes.append((int(flash[1])/100000, int(flash[2])/100000,
                            *rectangles[selected]))
    for designator in ['U4', 'SW1', 'L1']:
        for pad in pcb_parts[designator]['pads']:
            target = (pad['x']*.0254, pad['y']*.0254,
                      pad['pad'][1]*.0254, pad['pad'][2]*.0254)
            require(any(all(abs(a-b)<.005 for a,b in zip(target, flash))
                        for flash in flashes),
                    f"Missing final Gerber pad: {designator}-{pad['padNumber']}")

placement = list(csv.DictReader(io.StringIO(
    (root/'stm32PCB-A7-PickAndPlace.csv').read_bytes().decode('utf-16')), delimiter='\t'))
require({row['Designator'] for row in placement} == set(pcb_parts) and len(placement)==32,
        'Placement completeness')
for row in placement:
    part = pcb_parts[row['Designator']]['component']
    close(float(row['Mid X'].removesuffix('mm')), part['x']*.0254, 'Placement X')
    close(float(row['Mid Y'].removesuffix('mm')), part['y']*.0254, 'Placement Y')
    close((float(row['Rotation'])-part['rotation'])%360, 0, 'Placement rotation')

native_bom = list(csv.DictReader(io.StringIO(
    (root/'stm32PCB-A7-BOM.csv').read_bytes().decode('utf-16')), delimiter='\t'))
smt_bom = list(csv.DictReader(io.StringIO(
    (root/'stm32PCB-A7-SMT-BOM.csv').read_text(encoding='utf-8-sig'))))
for rows, code_column in [(native_bom, 'Supplier Part'), (smt_bom, 'LCSC Part #')]:
    require(len(rows)==19 and sum(int(row['Quantity']) for row in rows)==32, 'BOM count')
    seen = set()
    for row in rows:
        designators = row['Designator'].split(',')
        require(len(designators)==int(row['Quantity']), 'BOM quantity/designator mismatch')
        for designator in designators:
            require(designator not in seen, 'Duplicate BOM designator')
            seen.add(designator)
            require(row[code_column]==pcb_parts[designator]['component']['supplierId'],
                    f'BOM supplier mismatch: {designator}')
    require(seen==set(pcb_parts), 'BOM missing part')

with zipfile.ZipFile(root/'stm32PCB-A7-project.zip') as archive:
    archived = {name for name in archive.namelist() if not name.endswith('/')}
    on_disk = {str(path.relative_to(root)) for path in (root/'stm32PCB').rglob('*')
               if path.is_file()}
    require(archived==on_disk, 'Project archive file set differs from native project')
    for name in archived:
        require(archive.read(name)==(root/name).read_bytes(), f'Project ZIP differs: {name}')

reopened = json.loads((root/'reopened-check.json').read_text())
require(reopened['native_project']==str((root/'stm32PCB/stm32PCB.eprj3').resolve()),
        'Fresh-session proof refers to another native project')
require(reopened['strict_pcb_drc']==[], 'Reopened PCB DRC violations')
for key in ['all_32_components_positions_rotation_footprints_supplier_and_manufacturer_part_ids_match',
            'all_143_pads_match_export_snapshot', 'all_silkscreen_primitives_match_export_snapshot']:
    require(reopened[key], f'Failed saved-project readback: {key}')


report = {
    'revision': 'A7',
    'date': '2026-10-17',
    'power_trace_widths_mm': widths,
    'motor_output_layer': 'BottomLayer',
    'motor_output_width_mm': 1.5,
    'vin_parallel_via_bank_count': 2,
    'vin_parallel_vias_per_bank': 3,
    'mcu_5v_parallel_vias': 2,
    'power_via_drill_mm_nominal': .4,
    'board_components': len(components),
    'schematic_and_pcb_pin_assignments_checked': checked_pins,
    'connected_nets': len(nets),
    'unused_mcu_pins_brought_out': len(unused),
    'pcb_tracks': len(pcb['lines']),
    'pcb_vias': len(pcb['vias']),
    'ground_solid_regions': len(pcb['fills']),
    'board_size_mm': [100, 90],
    'socket_pitch_mm': 2.54,
    'module_row_center_distance_mm': 16,
    'actual_row_center_distance_mm': {
        'stm32': round(pad_position('J6', 1)[1] - pad_position('J7', 20)[1], 6),
        'tb6612': round(pad_position('J8', 1)[1] - pad_position('J9', 1)[1], 6),
    },
    'connector_drills_from_gerber_mm': header_holes,
    'schematic_drc': json.loads((root / 'schematic-drc.json').read_text()),
    'pcb_drc_violations': 0,
    'input_voltage_design_range_V': [7, 12],
    'output_current_design_target_A_total': 3,
    'pcb_routing_completed': True,
    'stm32_photo_pin_order_verified': True,
    'stm32_top_view_usb_side': 'left',
    'stm32_top_view_swd_side': 'right',
    'gerber_archive_verified': True,
    'u4_sw1_l1_gerber_pad_geometry_verified': True,
    'placement_rows_verified': len(placement),
    'bom_groups_verified': len(native_bom),
    'native_and_smt_bom_supplier_ids_verified': True,
    'project_zip_matches_native_files': True,
    'saved_project_fresh_session_readback_verified': True,
    'u4_pad_mm': [.6, 1.1],
    'u4_row_center_spacing_mm': 2.7,
    'u4_pitch_mm': .95,
    'sw1_supplier_id': 'C318884',
    'sw1_height_mm': 1.5,
    'sw1_pad_mm': [1, .75],
    'sw1_pad_column_and_row_spacing_mm': [6, 3.75],
    'module_mechanical_fit_verified': False,
    'full_module_3d_collision_check_completed': False,
    'power_load_and_thermal_test_completed': False,
    'hardware_tested': False,
    'fabrication_release_approved': False,
}
print(json.dumps(report, ensure_ascii=False, indent=2))

(root/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
