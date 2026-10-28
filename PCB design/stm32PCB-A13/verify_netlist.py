import csv, json, pathlib, zipfile
root = pathlib.Path(__file__).resolve().parent
old = json.loads((root.parent/'stm32PCB-A10/stm32PCB-A10.netlist.json').read_text())
new = json.loads((root/'A13.netlist.json').read_text())
state = json.loads((root/'pcb-state.json').read_text())
def pins(j):
    return {c['props']['Designator']: {str(k):v['net'] for k,v in c['pinInfoMap'].items()} for c in j['components'].values() if c['props'].get('Convert to PCB') == 'yes'}
a,b = pins(old),pins(new)
pcb = {p['component']['designator']:{str(x['padNumber']):x['net'] for x in p['pads']} for p in state['parts']}
assert b == pcb, 'Schematic/PCB pin mismatch'
changes = [(r,k,a[r][k],b[r][k]) for r in a for k in a[r] if a[r][k] != b[r][k]]
assert sorted(changes) == sorted([('C1','1','VM','DC_RAW'),('C2','1','VM','DC_RAW'),('J8','1','VM','DC_RAW')])
assert set(b)-set(a) == {'J14','J15','J16'}
for ref in ('J14','J15','J16'): assert b[ref] == {'1':'+3V3','2':'GND'}
with zipfile.ZipFile(root/'A13-prototype-Gerber.zip') as z:
    d = z.read('Drill_PTH_Through.DRL').decode(); assert 'T06C1.10002' in d; d=d[d.index('T06\n'):]
    assert all(f'X{x}Y{y}' in d for x in ('87.63','90.17') for y in ('55.88','50.8','45.72'))
with (root/'A13-PickAndPlace.csv').open(encoding='utf-16') as f: rows=list(csv.DictReader(f,delimiter='\t'))
for ref,y in [('J14','55.88mm'),('J15','50.8mm'),('J16','45.72mm')]:
    r=next(r for r in rows if r['Designator']==ref); assert (r['Mid X'],r['Mid Y'],r['Layer']) == ('88.9mm',y,'T')
print(f'PASS: {len(b)} components, {sum(map(len,b.values()))} pins, 6 header drills and 3 placement records')
