"""Measure all top-layer foreign copper around C1/C2, including native ground-fill holes.
Requires Shapely 2.x. Distances are copper-edge to copper-edge in millimetres.
"""
import json
from pathlib import Path
from shapely import affinity
from shapely.geometry import Point, LineString, Polygon, box
ROOT=Path(__file__).resolve().parent
SCALE=.0254

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
s=json.loads((ROOT/'pcb-state.json').read_text())
parts={p['component']['designator']:p for p in s['parts']}
items=[]
for n,p in parts.items():
 for q in p['pads']:
  if q['layer'] in [1,12]:items.append((n+'.'+q['padNumber'],q['net'],pad_shape(q)))
for l in s['lines']:
 if l['layer']==1:items.append(('track:'+l['primitiveId'],l['net'],LineString([(l['startX']*SCALE,l['startY']*SCALE),(l['endX']*SCALE,l['endY']*SCALE)]).buffer(l['lineWidth']*SCALE/2,quad_segs=32)))
for v in s['vias']:items.append(('via:'+v['primitiveId'],v['net'],Point(v['x']*SCALE,v['y']*SCALE).buffer(v['diameter']*SCALE/2,quad_segs=32)))
fill_ids={f['primitiveId']:f for f in s['fills']}
for line in (ROOT/'stm32PCB/pcb/PCB1.epcb2').read_text().splitlines():
 if '||' not in line:continue
 a,b=line.split('||',1);m=json.loads(a)
 if m.get('type')!='FILL' or m.get('id') not in fill_ids:continue
 f=fill_ids[m['id']]
 if f['layer']!=1:continue
 d=json.loads(b.rstrip('|'));rings=[]
 for r in d['path']:
  assert r[2]=='L'
  z=r[:2]+r[3:];rings.append([(z[i]*SCALE,z[i+1]*SCALE) for i in range(0,len(z),2)])
 items.append(('fill:'+m['id'],f['net'],Polygon(rings[0],rings[1:])))

report={'revision':'A14','units':'mm','minimum_required_cap_pad_to_foreign_copper':.25,'pads':{}}
for name in ['C1','C2']:
    for pad in parts[name]['pads']:
        shape=pad_shape(pad)
        nearest=min((shape.distance(g),identifier,net) for identifier,net,g in items if net and net!=pad['net'])
        report['pads'][name+'.'+pad['padNumber']]={'net':pad['net'],'minimum_clearance':round(nearest[0],6),'nearest_object':nearest[1],'nearest_net':nearest[2]}
        assert nearest[0]>=.25, (name,pad['padNumber'],nearest)
report['C2_GND_to_AIN1']=round(min(pad_shape(p).distance(g) for p in parts['C2']['pads'] if p['net']=='GND' for identifier,net,g in items if net=='AIN1'),6)
report['C1_DC_RAW_to_GND']=round(min(pad_shape(p).distance(g) for p in parts['C1']['pads'] if p['net']=='DC_RAW' for identifier,net,g in items if net=='GND'),6)
report['C1_C2_nearest_pad_gap']=round(min(pad_shape(a).distance(pad_shape(b)) for a in parts['C1']['pads'] for b in parts['C2']['pads']),6)
report['all_four_pads_pass']=True
(ROOT/'local-clearance-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
