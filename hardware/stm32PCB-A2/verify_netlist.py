import json
from pathlib import Path


root = Path(__file__).resolve().parent
native = json.loads((root / 'stm32PCB-A2.netlist.json').read_text())
drc = json.loads((root / 'drc.json').read_text())
components = {item['props']['Designator']: item for item in native['components'].values()}
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
    'J6': ['', '', '', '', 'S0', 'S1', 'S2', 'S3', 'S4', 'S5', 'PWMA', 'PWMB', 'S6', 'S7', 'BTN', '', '', '+3V3', 'GND', 'GND'],
    'J7': ['AIN1', 'AIN2', 'BIN1', 'BIN2', '', '', '', '', '', '', '', '', 'STBY', '', '', '', '', '+5V', 'GND', '+3V3'],
    'J8': ['VM', '+3V3', 'GND', 'AO1', 'AO2', 'BO2', 'BO1', 'GND'],
    'J9': ['PWMA', 'AIN2', 'AIN1', 'STBY', 'BIN1', 'BIN2', 'PWMB', 'GND'],
    'J10': ['+3V3', 'GND', 'S0', 'S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'S7'],
}
if set(components) != set(expected):
    raise ValueError('Unexpected or missing board components')
checked_pins = 0
unused_pins = 0
nets = set()
for designator, expected_nets in expected.items():
    component = components[designator]
    properties = component['props']
    if not properties.get('Footprint') or not properties.get('FootprintName'):
        raise ValueError(f'Missing footprint: {designator}')
    if properties['Add into BOM'] != 'yes' or properties['Convert to PCB'] != 'yes':
        raise ValueError(f'Unexpected manufacturing setting: {designator}')
    actual = component['pinInfoMap']
    if set(actual) != {str(number) for number in range(1, len(expected_nets) + 1)}:
        raise ValueError(f'Unexpected pin numbering: {designator}')
    for number, expected_net in enumerate(expected_nets, 1):
        actual_net = actual[str(number)]['net']
        if actual_net != expected_net:
            raise ValueError(f'{designator}-{number}: {actual_net!r} != {expected_net!r}')
        checked_pins += 1
        if actual_net:
            nets.add(actual_net)
        else:
            unused_pins += 1
if drc:
    raise ValueError('Archived schematic DRC contains violations')
report = {
    'project': 'stm32PCB',
    'revision': 'A2',
    'date': '2026-10-16',
    'board_components': len(components),
    'component_pin_assignments_checked': checked_pins,
    'module_socket_pin_positions_checked': sum(len(expected[designator]) for designator in ['J6', 'J7', 'J8', 'J9', 'J10']),
    'intentionally_unconnected_socket_pins': unused_pins,
    'connected_nets': len(nets),
    'all_board_components_have_footprints': True,
    'schematic_drc_errors': 0,
    'schematic_drc_warnings': 0,
    'power_nets_are_distinct': len({expected['J1'][0], expected['J2'][0], expected['J10'][0]}) == 3,
    'pcb_placement_completed': False,
    'pcb_routing_completed': False,
    'mechanical_fit_verified': False,
    'hardware_tested': False,
}
print(json.dumps(report, ensure_ascii=False, indent=2))
