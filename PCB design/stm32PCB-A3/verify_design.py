import json
import math
import re
import zipfile
from pathlib import Path


root = Path(__file__).resolve().parent
native = json.loads((root / 'stm32PCB-A3.netlist.json').read_text())
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
close(pad_position('J6', 1)[1] - pad_position('J7', 20)[1], 15, 'STM32 row center spacing')
close(pad_position('J8', 1)[1] - pad_position('J9', 1)[1], 15, 'TB6612 row center spacing')
close(pad_position('J6', 1)[0], pad_position('J7', 20)[0], 'STM32 opposite row endpoints')
close(pad_position('J8', 1)[0], pad_position('J9', 1)[0], 'TB6612 opposite row endpoints')
gray_left_to_right = sorted(pcb_parts['J10']['pads'], key=lambda pad: pad['x'])
require([pad['net'] for pad in gray_left_to_right] == ['S7', 'S6', 'S5', 'S4', 'S3', 'S2', 'S1', 'S0', 'GND', '+3V3'], 'Gray left-to-right order')
require(pad_position('J10', 1)[1] > pad_position('J11', 1)[1] > pad_position('J6', 1)[1] > pad_position('J7', 1)[1] > pad_position('J8', 1)[1] > pad_position('J9', 1)[1], 'Front-to-rear module arrangement')
unused = set(expected['J11'][:-1] + expected['J12'][:-1])
require(len(unused) == 18, 'Unused pin count')
for net in unused:
    require(sum(net in expected[header] for header in ['J6', 'J7']) == 1, f'Missing MCU socket connection: {net}')
silkscreen = {item['text'] for item in pcb['strings'] if item['layer'] == 3}
require((unused - {'VBAT_EXT'}) | {'VBAT'} <= silkscreen, 'Missing unused-pin silkscreen labels')
for filename in ['schematic-drc.json', 'pcb-drc.json']:
    require(json.loads((root / filename).read_text()) == [], f'Archived DRC violations: {filename}')
require(pcb['drc'] == [], 'PCB state contains DRC violations')

with zipfile.ZipFile(root / 'stm32PCB-A3-REVIEW-ONLY-Gerber.zip') as archive:
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
with zipfile.ZipFile(root / 'stm32PCB-A3-project.zip') as archive:
    require(archive.testzip() is None, 'Project archive CRC failure')
    require('stm32PCB/stm32PCB.eprj3' in archive.namelist(), 'Missing native project entry')
    require('stm32PCB/pcb/PCB1.epcb2' in archive.namelist(), 'Missing native PCB source')

report = {
    'revision': 'A3',
    'date': '2026-10-16',
    'board_components': len(components),
    'schematic_and_pcb_pin_assignments_checked': checked_pins,
    'connected_nets': len(nets),
    'unused_mcu_pins_brought_out': len(unused),
    'pcb_tracks': len(pcb['lines']),
    'pcb_vias': len(pcb['vias']),
    'ground_solid_regions': len(pcb['fills']),
    'board_size_mm': [100, 90],
    'socket_pitch_mm': 2.54,
    'module_row_center_distance_mm': 15,
    'connector_drills_from_gerber_mm': header_holes,
    'schematic_drc_violations': 0,
    'pcb_drc_violations': 0,
    'input_voltage_design_range_V': [7, 12],
    'output_current_design_target_A_total': 3,
    'pcb_routing_completed': True,
    'gerber_archive_verified': True,
    'module_mechanical_fit_verified': False,
    'full_module_3d_collision_check_completed': False,
    'power_load_and_thermal_test_completed': False,
    'hardware_tested': False,
    'fabrication_release_approved': False,
}
print(json.dumps(report, ensure_ascii=False, indent=2))
