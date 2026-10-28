import json,math,zipfile,re,pathlib
r=pathlib.Path(__file__).resolve().parent;a=json.loads((r.parent/'stm32PCB-A13/A13.netlist.json').read_text());b=json.loads((r/'A14.netlist.json').read_text());s=json.loads((r/'pcb-state.json').read_text());old=json.loads((r.parent/'stm32PCB-A13/pcb-state.json').read_text())
def net(j):return {c['props']['Designator']:{k:v['net'] for k,v in c['pinInfoMap'].items()} for c in j['components'].values() if c['props'].get('Convert to PCB')=='yes'}
def byref(state):return {p['component']['designator']:p for p in state['parts']}
a1,b1=net(a),net(b);assert a1==b1;parts=byref(s);base=byref(old);pcb={k:{p['padNumber']:p['net'] for p in v['pads']} for k,v in parts.items()};assert b1==pcb
j=parts['J10'];assert j['component']['supplierId']=='C492418' and j['component']['rotation']==180
for p in j['pads']:
 q=next(z for z in base['J10']['pads'] if z['padNumber']==p['padNumber']);assert math.dist((p['x'],p['y']),(q['x'],q['y']))*.0254<.005
assert [p['net'] for p in sorted(j['pads'],key=lambda x:x['x'])]==['S7','S6','S5','S4','S3','S2','S1','S0','GND','+3V3']
# Embedded library geometry: negative local Y is transformed by 180° into positive board Y (FRONT).
on=False;ys=[]
for line in (r/'stm32PCB/pcb/PCB1.epcb2').read_text().splitlines():
 try:x,y=line.split('||',1);m=json.loads(x);d=json.loads(y.rstrip('|'))
 except:continue
 if m['type']=='DOCHEAD':
  if on:break
  on=d.get('uuid')==j['component']['footprint']['uuid'];continue
 if on and m['type']=='POLY' and d.get('layerId')==13:
  path=d['path'];nums=path[:2]+path[3:];ys.extend(nums[1::2])
assert ys and min(ys)<-370
extent=j['component']['y']-min(ys);board_edge=90/.0254;overhang=(extent-board_edge)*.0254
report={'components':len(parts),'pins':sum(len(x) for x in b1.values()),'schematic_vs_A13_networks_equal':True,'pcb_pin_networks_match':True,'j10_part':'C492418','j10_rotation_degrees':180,'j10_pad_positions_unchanged':True,'j10_front_positive_board_y':True,'library_outline_overhang_board_mm':round(overhang,3),'physical_fit_verified':False}
(r/'j10-verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
