"""Build a private, source-tagged blue-field reference, in metres."""
from pathlib import Path
from collections import Counter
import hashlib, json, re
from OCP.TopAbs import TopAbs_REVERSED
from inspect_solids import *

root=base.parent
out=root/'blue-speaker'
out.mkdir(exist_ok=True)
occ=json.loads((base/'occurrences.json').read_text())
axes=np.array([[-1.,0.,0.],[0.,0.,1.],[0.,1.,0.]])
offset=np.array([8270.875,4105.275,-4.7752])
def convert(local,part):
    m=np.asarray(part['fieldCadFromPart'])
    return ((np.asarray(local)@m[:,:3].T+m[:,3])@axes.T+offset)/1000
def sha(path):
    with path.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def group(name):
    if name in ('GE-24307','GE-24308','GE-24309','GE-24306'):return 'interior ramp or wall'
    if name in ('GE-24391-01','GE-24392'):return 'collection ramp downstream'
    if name in ('GE-24355','Grip_Wheel-2in-MAXSpline','Compliant Wheel - 4in - MAXSpline','80 tooth RT-25 Belt'):return 'counter moving contact candidate'
    if name in ('GE-24352-01','GE-24362-01','GE-24363','GE-24364','GE-24365','GE-24371-01','GE-24372-01','GE-24374','GE-24375','GE-24390-01','GE-24393','42SMR'):return 'counter structure'
    if re.match(r'GE-243(0[1235]|1[0-9]|2[012]|31|94)',name):return 'opening, guard, or frame'
    return None
def mesh(filename):
    shape=read_shape(filename)
    m=BRepMesh_IncrementalMesh(shape,0.38,False,0.16,False)
    assert m.IsDone(),filename
    nodes=[];triangles=[];face_index=[];face_rows=[]
    for i,f in enumerate(faces(shape)):
        loc=TopLoc_Location()
        t=BRep_Tool.Triangulation_s(f,loc)
        if t is None:raise ValueError((filename,i))
        start=len(nodes)
        nodes.extend(t.Node(j).Transformed(loc.Transformation()).Coord() for j in range(1,t.NbNodes()+1))
        for j in range(1,t.NbTriangles()+1):
            inds=np.array(t.Triangle(j).Get())+start-1
            if f.Orientation()==TopAbs_REVERSED:inds=inds[[0,2,1]]
            triangles.append(inds);face_index.append(i)
        if filename.startswith(('GE-24307.','GE-24308.','GE-24309.','GE-24355.','GE-24364.','GE-24365.')):
            row=detail(f)
            if row['geom']=='GeomAbs_Plane' and row['area']>5_000:
                face_rows.append((i,row))
    return np.array(nodes),np.array(triangles,dtype='int32'),np.array(face_index,dtype='int32'),face_rows
def write_json(name,data):
    (out/name).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n',encoding='utf8')
memo={};counts=Counter();parts=[];observers=[];arrays={};planar=[]
for row in occ:
    if 'GE-24300_Briweld Field Blue' not in row['path'] or row['children']:continue
    n=row['path'][-1];kind=group(n)
    if not kind:continue
    filename=row.get('brepLocal',re.sub(r'[^A-Za-z0-9_.-]','_',n)+'.brep')
    if filename not in memo:memo[filename]=mesh(filename)
    v,t,fi,planes=memo[filename]
    w=convert(v,row);counts[n]+=1
    key='p'+str(len(parts)); arrays[key+'_v']=w;arrays[key+'_t']=t;arrays[key+'_face']=fi
    parts.append(dict(key=key,name=n,occurrence=counts[n],className=kind,path=row['path'],triangles=len(t),vertices=len(w),blueFieldBoundsM=[w.min(0).tolist(),w.max(0).tolist()]))
    for index,f in planes:
        pts=convert(f['vertices'],row);m=np.array(row['fieldCadFromPart'])
        planar.append(dict(key=key,name=n,occurrence=counts[n],face=index,areaM2=f['area']/1e6,centerM=convert([f['center']],row)[0].tolist(),normal=(axes@m[:,:3]@f['axis']).tolist(),boundaryVerticesM=pts.tolist()))
    if n=='42SMR':
        # Local y=0 is the facing, rectangular case end opposite its mate in the counter.
        corners=convert([[x,0,z] for x,z in [(-12.7,-15.748),(0,-15.748),(0,15.748),(-12.7,15.748)]],row)
        observers.append(dict(key=key,caseEndCornersM=corners.tolist(),caseCenterM=corners.mean(0).tolist()))
nlow=-.001050;nhigh=.458944;loy=5.020365;hiy=6.071290
corners=[[nlow,loy,1.984464],[nlow,hiy,1.984464],[nhigh,hiy,2.105733],[nhigh,loy,2.105733]]
source=dict(archive='CRESCENDO2024Field-STEP.zip',url='https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/CRESCENDO2024Field-STEP.zip',archiveSha256=sha(root/'CRESCENDO2024Field-STEP.zip'),stepMember='FE-2024.STEP',stepSha256=sha(root/'FE-2024.STEP'),speakerProduct='GE-24300_Briweld Field Blue')
frame=dict(units='m',name='FE-2024 blue field',fromStepCadMm='x=(8270.875-X)/1000; y=(4105.275+Z)/1000; z=(Y-4.7752)/1000',basis='nominal half field dimensions with actual CAD axes and measured carpet top')
np.savez_compressed(out/'named-panels.npz',**arrays)
write_json('panels.json',dict(schema='blue-speaker-panels/1',source=source,frame=frame,mesh='named-panels.npz',meshSha256=sha(out/'named-panels.npz'),columns={'<key>_v':'blue field XYZ, metres','<key>_t':'triangles indexing zero-based vertices','<key>_face':'original zero-based OCCT face index per triangle'},linearDeflectionMm=0.38,angularDeflectionRad=.16,parts=parts,limits=['selected occurrence roles only; dynamics and material parameters unmeasured','private FIRST geometry; do not redistribute','all bodies at saved fixed pose; moving surfaces require a separate powered model']))
write_json('large-planes.json',dict(frame=frame,faces=planar,note='Faces may have interior holes; vertices are unordered. Use named mesh triangles for trimmed boundaries.'))
write_json('opening-and-counter.json',dict(source=source,frame=frame,nominalOpeningCornersM=corners,openingBasis='GE-24312 inward side faces; GE-24305 top height; GE-24319 lower corner; low x inferred from 18.11 in horizontal run from GE-24300 sheet 2. Actual edge radii and assembly tolerance require separate collision checks.',sensorCaseEnds=observers,beamPairs=sorted([[v['key'] for v in observers if abs(v['caseCenterM'][0]-x)<.001] for x in [-1.136,-1.1074]]),uncertainty='Four simplified 42SMR cases. Pair opposing ends by x. Centerlines are geometric representatives. Actual active optical location, installed aim, electronics channel mapping and control algorithm are unresolved. A complete shadow across every ray joining a pair of facing active-end rectangles is a conservative geometric observation conditional on through-beam installation.'))
print('mesh',len(parts),'parts,',sum(p['triangles'] for p in parts),'triangles;',len(planar),'large planes;',len(observers),'sensor cases')
