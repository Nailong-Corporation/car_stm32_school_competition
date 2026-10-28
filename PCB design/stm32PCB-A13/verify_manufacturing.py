"""Check exported Gerber/drill/BOM/placement against final A13 editor data."""
import csv, io, json, math, re, zipfile, pathlib
R=pathlib.Path(__file__).resolve().parent
s=json.loads((R/'pcb-state.json').read_text()); parts={p['component']['designator']:p for p in s['parts']}
with zipfile.ZipFile(R/'A13-prototype-Gerber.zip') as z:
 assert z.testzip() is None
 need={'Gerber_TopLayer.GTL','Gerber_BottomLayer.GBL','Gerber_TopSilkscreenLayer.GTO','Gerber_TopSolderMaskLayer.GTS','Gerber_BottomSolderMaskLayer.GBS','Gerber_BoardOutlineLayer.GKO','Drill_PTH_Through.DRL','Drill_NPTH_Through.DRL','Drill_PTH_Through_Via.DRL'}
 assert need <= set(z.namelist())
 out=z.read('Gerber_BoardOutlineLayer.GKO').decode();xy=[(int(x)/100000,int(y)/100000) for x,y in re.findall(r'X(-?\d+)Y(-?\d+)D0[12]',out)];assert abs(max(x for x,y in xy)-min(x for x,y in xy)-100)<.005;assert abs(max(y for x,y in xy)-min(y for x,y in xy)-90)<.005
 npth=z.read('Drill_NPTH_Through.DRL').decode();assert 'T01C3.20000' in npth;holes={(float(x),float(y)) for x,y in re.findall(r'^X([\d.]+)Y([\d.]+)$',npth,re.M)};assert holes=={(4,4),(96,4),(4,86),(96,86)}
 def drill(file):
  tools={};sel=None;ans=[]
  for line in z.read(file).decode().splitlines():
   a=re.fullmatch(r'T(\d+)C([\d.]+)',line);b=re.fullmatch(r'T(\d+)',line);c=re.fullmatch(r'X([\d.]+)Y([\d.]+)',line)
   if a:tools[a[1]]=float(a[2])
   elif b:sel=b[1]
   elif c:ans.append((float(c[1]),float(c[2]),tools[sel]))
  return ans
 pth=drill('Drill_PTH_Through.DRL');via=drill('Drill_PTH_Through_Via.DRL')
 checked=0;matched={}
 for ref in ('J1','J2','J3','J4','J6','J7','J8','J9','J10','J11','J12','J14','J15','J16'):
  for p in parts[ref]['pads']:
   x,y=p['x']*.0254,p['y']*.0254; candidates=[(math.dist((x,y),(a,b)),dia) for a,b,dia in pth]; distance,dia=min(candidates);assert distance<.005 and 1<=dia<=1.25,(ref,p['padNumber'],distance,dia);checked+=1;matched[(ref,p['padNumber'])]=min(pth,key=lambda q:math.dist((x,y),q[:2]))
 for up,down,count,reverse in [('J6','J7',20,True),('J9','J8',8,False)]:
  for n in range(1,count+1):
   pa=next(p for p in parts[up]['pads'] if p['padNumber']==str(n));pb=next(p for p in parts[down]['pads'] if p['padNumber']==str(count+1-n if reverse else n));da=matched[(up,str(n))];db=matched[(down,str(count+1-n if reverse else n))];assert abs(da[1]-db[1]-15.24)<.00001 and abs(da[0]-db[0])<.005
 def flashes(file):return [(int(x)/100000,int(y)/100000) for x,y in re.findall(r'(?:G01)?X(-?\d+)Y(-?\d+)D03\*',z.read(file).decode())]
 top=flashes('Gerber_TopLayer.GTL');bottom=flashes('Gerber_BottomLayer.GBL');smd=0
 for ref,p in parts.items():
  for pad in p['pads']:
   if pad['layer'] not in (1,2):continue
   xy=(pad['x']*.0254,pad['y']*.0254);arr=top if pad['layer']==1 else bottom;assert min(math.dist(xy,q) for q in arr)<.005,(ref,pad['padNumber']);smd+=1
 # Newly moved C2 ground via appears in actual drill export and is tented.
 assert any(math.dist((38.481,29.845),(x,y))<.005 and abs(d-.508)<.005 for x,y,d in via)
 topmask=z.read('Gerber_TopSolderMaskLayer.GTS').decode(); assert 'X3848100Y2984500D03' not in topmask
with (R/'A13-PickAndPlace.csv').open(encoding='utf-16') as f: rows=list(csv.DictReader(f,delimiter='\t'))
assert len(rows)==len(parts) and {r['Designator'] for r in rows}==set(parts)
for r in rows:
 p=parts[r['Designator']]['component'];assert abs(float(r['Mid X'].removesuffix('mm'))-p['x']*.0254)<.005;assert abs(float(r['Mid Y'].removesuffix('mm'))-p['y']*.0254)<.005;assert abs((float(r['Rotation'])-p['rotation'])%360)<.005
with (R/'A13-BOM.csv').open(encoding='utf-16') as f: bom=list(csv.DictReader(f,delimiter='\t'))
seen=set()
for row in bom:
 refs=row['Designator'].split(',');assert len(refs)==int(row['Quantity'])
 for ref in refs:assert ref not in seen and row['Supplier Part']==parts[ref]['component']['supplierId'];seen.add(ref)
assert seen==set(parts)
with zipfile.ZipFile(R/'A13-native-project.zip') as z:
 files={str(p.relative_to(R)) for p in (R/'stm32PCB').rglob('*') if p.is_file()};assert files=={x for x in z.namelist() if not x.endswith('/')};assert all(z.read(x)==(R/x).read_bytes() for x in files)
report={'board_mm':[100,90],'npth_holes':4,'connector_drills_checked':checked,'row_spacing_mm':15.24,'smd_copper_flashes_checked':smd,'bom_components':len(seen),'placement_rows':len(rows),'source_zip_matches_disk':True,'c2_ground_via_mm':[38.481,29.845,.508]};(R/'manufacturing-verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
