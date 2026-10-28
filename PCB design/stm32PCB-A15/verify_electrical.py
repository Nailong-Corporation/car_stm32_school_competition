import json, pathlib
r=pathlib.Path(__file__).resolve().parent
A=json.loads((r.parent/'stm32PCB-A14/A14.netlist.json').read_text());B=json.loads((r/'A15.netlist.json').read_text());S=json.loads((r/'pcb-state.json').read_text())
def refs(j):return {c['props']['Designator']:c for c in j['components'].values() if c['props'].get('Convert to PCB')=='yes'}
a,b=refs(A),refs(B);assert a.keys()==b.keys();p={x['component']['designator']:{y['padNumber']:y['net'] for y in x['pads']} for x in S['parts']}
for ref in b:
 nets={k:v['net'] for k,v in b[ref]['pinInfoMap'].items()};assert nets==p[ref],ref
 if ref not in ['D1','R3']:assert nets=={k:v['net'] for k,v in a[ref]['pinInfoMap'].items()},ref
D=b['D1'];assert D['props']['Supplier Part']=='C22452'; assert {v['name']:v['net'] for v in D['pinInfoMap'].values()}=={'A':'DC_FUSED','K':'VIN_PROTECTED'}
R=b['R3'];assert R['props']['Supplier Part']=='C22797' and R['props']['Value']=='13kΩ';assert {v['net'] for v in R['pinInfoMap'].values()}=={'GND','BUCK_FB'}
report={'components':len(b),'pins':sum(len(c['pinInfoMap']) for c in b.values()),'pcb_schematic_match':True,'other_components_pin_networks_equal_A14':True,'D1_C22452':{'A':'DC_FUSED','K':'VIN_PROTECTED'},'R3_C22797_ohm':13000,'resistor_pin_net_swap_is_electrically_symmetric':True};(r/'electrical-verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
