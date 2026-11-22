from pathlib import Path
import numpy as np,json,base64, math,gzip,shutil,hashlib,sys,re
from scipy.spatial.transform import Rotation
p=Path(__file__).resolve().parent; web=p.parent.parent/'www'; repo=Path(sys.argv[1]);dest=repo/'docs/preliminary-design/releases/001';dest.mkdir(exist_ok=True,parents=True)
d=json.loads((web/'data.json').read_bytes()); ff=Path('C:/Users/ehart/Documents/FRC Native/2024/six-speaker-20261118/field-source/blue-speaker')
v=[];f=[]; fixtures={}
def beam(a,b,w,h):
    a,b=np.array(a,float),np.array(b,float);axis=b-a;axis/=np.linalg.norm(axis);basis=np.array([0,0,1]) if abs(axis[2])<.9 else np.array([0,1,0])
    side=np.cross(axis,basis);side*=w/2/np.linalg.norm(side);up=np.cross(axis,side);up*=h/2/np.linalg.norm(up)
    start=len(v)
    for c in [a,b]:
        for i,j in [(1,1),(-1,1),(-1,-1),(1,-1)]:v.append(c+i*side+j*up)
    for face in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]:f.extend([[start+face[k] for k in t] for t in [(0,1,2),(0,2,3)]])
def seal(name):
    vv=np.array(v,dtype='<f4');ff=np.array(f,dtype='<u4')
    fixtures[name]=dict(v=base64.b64encode(vv.tobytes()).decode(),f=base64.b64encode(ff.tobytes()).decode(),bounds=[(vv.min(0)/1000).tolist(),(vv.max(0)/1000).tolist()])
    v.clear();f.clear()
corner=np.array(d['corners'])
for i in range(4):beam(corner[i],corner[(i+1)%4],8,8)
for idx in (0,1):beam(corner[idx]+[-1000,0,0],corner[idx],14,14)
seal('speakerPanel')
faces=json.loads((ff/'large-planes.json').read_bytes())['faces']
for face in faces:
    # Display the two horizontal transport runs as dimensioned strips.
    if face['name']=='GE-24355' and face['normal'][2]>.999:
        pts=np.array(face['boundaryVerticesM'])*1000;lo,hi=pts.min(0),pts.max(0);mid=(lo+hi)/2
        if hi[0]-lo[0]>50 and hi[1]-lo[1]>10:beam([lo[0],mid[1],mid[2]-2],[hi[0],mid[1],mid[2]-2],hi[1]-lo[1],4)
seal('speakerRamp')
sensor=json.loads((ff/'opening-and-counter.json').read_bytes())
for row in sensor['sensorCaseEnds']:
    pts=1000*np.array(row['caseEndCornersM']);lo=pts.min(0);hi=pts.max(0);mid=(lo+hi)/2
    beam([lo[0],mid[1],mid[2]],[hi[0],mid[1],mid[2]],hi[1]-lo[1],4)
seal('sensors')
chain=next(c for c in d['concepts']['guided']['cases'] if c['key']=='guided-front')['chain'][0]
chain=np.asarray(chain);chain=chain-chain.mean(0)*[1,1,0]+[5760.46,4105.275,0]
length=chain[-1,1]-chain[0,1];center=np.array([5760.46-length/(2*math.sqrt(3)),4105.275,0]);others=[]
for angle in (0,120,240):
    ln=Rotation.from_euler('z',angle,degrees=True).apply(chain-center)+center
    for a,b in zip(ln,ln[1:]):beam(a,b,9,9)
    if angle==0:seal('workingChain')
    others.append(ln)
seal('otherChain')
for ln in others:
    for vtx in [ln[0],ln[-1]]:beam(vtx-[0,0,35],vtx+[0,0,15],35,35)
seal('anchors')
for ln in others:
    a,b=ln[0].copy(),ln[-1].copy()
    for pt in (a,b):beam([pt[0],pt[1],0],[pt[0],pt[1],2242],35,35)
    beam([a[0],a[1],2225],[b[0],b[1],2225],35,35)
seal('stageSteel')
d['fixtureMeshes']=fixtures
d['fixtureSource']=dict(scope='Project-drawn speaker opening, conveyor strips, sensor faces, stage envelope and chain centerlines. Dimensions use the 2024 field coordinate system. These display primitives illustrate the recorded field interaction.',nominalOpeningCornersMm=corner.tolist(),fieldRevision=sensor['source'],references=['2024 FIRST field drawings GE-24350, GE-24361','kickoff/stage-rules.json'])
# The public artifact carries the project-authored blockout and numerical traces. Keep the full private fixture tessellation out.
raw=json.dumps(d,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()
assert not re.search(rb'(?:(?<![A-Za-z])[A-Za-z]:[\\/]|file://|/Users/|AppData|localhost:8764)',raw,re.I)
(dest/'scene.json.gz').write_bytes(gzip.compress(raw,compresslevel=7,mtime=0))
for n in ('review.js','review.css','index.html','resources.js','presentation.md'):
    s=(web/n).read_text(encoding='utf8')
    if n=='index.html':
        s=s.replace('href="/','href="./').replace('src="/','src="./').replace('./?concept=field','../../../field-lab.html?concept=field')
        s=s.replace('<main>','<main><nav aria-label="Review releases"><a href="../../">All preliminary releases</a></nav>',1)
    if n=='review.js':
        old="const response=await fetch('/data.json');if(!response.ok)throw Error('Review data unavailable');data=await response.json();"
        assert s.count(old)==1
        s=s.replace(old,"const response=await fetch('./scene.json.gz');if(!response.ok)throw Error('Review data unavailable');const bytes=await response.arrayBuffer();const packed=new Uint8Array(bytes);data=await (packed[0]===31&&packed[1]===139?new Response(new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'))):new Response(bytes)).json();")
        s=re.sub(r" let stamp=await .*?1500\);","",s)
        assert "fetch('/version" not in s
        s=s.replace("init().catch","init().catch")
    (dest/n).write_text(s,encoding='utf8')
print('compressed scene',len((dest/'scene.json.gz').read_bytes()),'raw',len(raw))


