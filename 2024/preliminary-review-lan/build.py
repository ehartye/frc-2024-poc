"""Build the owner LAN review from the exact preliminary-design checkpoint."""
import hashlib, itertools, json, sys
from collections import Counter
from pathlib import Path
import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GProp import GProp_GProps
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Trsf
here=Path(__file__).resolve().parent
repo=Path(sys.argv[1]); review=Path(sys.argv[2]); dest=here/'build-cache';dest.mkdir(exist_ok=True)
sys.path.insert(0,str(repo/'scripts'))
from showcase_contacts import load_solids, candidate_pairs
from mechanical_motion import bounds, common, transforms
from component_palette import resolve_palette
def read(path):return json.loads(path.read_bytes())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def verify(path,digest):
    if sha(path)!=digest:raise ValueError('Source changed: '+path.name)
result=read(review/'review-result.json');layout=read(review/'layout.json')
verify(review/'layout.json', result['planSha256'])
for label,ref in [('package.step',result['step']),('paths.json',result['motion']),('budgets.json',result['budgets']['source']),('design.json',result['design'])]:verify(review/label,ref['sha256'])
study=read(review/'paths.json');budget=read(review/'budgets.json')
groups=result['groups'];groupof={n:g for g,group in groups.items() for n in group['members']}
palette=resolve_palette(review.parents[4]);palette['bumper']='#356EB7'
track=next(Path(k) for k in read(review/'sources.json')['referenceFiles'] if k.endswith('ring.npy'))
verify(track,read(review/'sources.json')['referenceFiles'][str(track)]);raw=np.load(track)
goal=Path(r'R:\2024\six-speaker-20261118\field-source\blue-speaker\opening-and-counter.json')
corners=(1000*np.array(read(goal)['nominalOpeningCornersM'])).tolist()
def describe(n,g):
    if g=='Frame':return 'structure','Aluminum frame stock'
    if 'bumper' in g:return 'bumper','Plywood, foam and fabric assembly'
    if 'NOTE' in g:return 'game-piece','Foam'
    if 'loom' in n or 'corridor' in n:return 'reserved-space','Wire routing allowance'
    if 'motor' in n.lower() or g=='Controls':return 'electrical','Mixed purchased assembly'
    if g.startswith('Battery'):return 'electrical','Lead-acid pack and housing'
    if 'ladder' in n or 'battery blade' in n:return 'structure','Formed metal support (planning)'
    if 'tire' in n:return 'mechanism','Wheel and tread assembly'
    if 'Wheel station' in n:return 'mechanism','Polycarbonate core and TPU tread (planning)'
    if 'rack neck' in n:return 'mechanism','Rack/backer assembly (planning)'
    if 'Feeder carriage' in n:return 'mechanism','Mixed guide and carriage assembly'
    return 'mechanism','Aluminum stock (planning)'
def mesh(s):
    BRepMesh_IncrementalMesh(s,.08,False,.15,False)
    it=TopExp_Explorer(s,TopAbs_FACE); lookup={};verts=[];faces=[]
    while it.More():
        face=(getattr(TopoDS,"Face_s",None) or TopoDS.Face)(it.Current());loc=TopLoc_Location();tri=BRep_Tool.Triangulation_s(face,loc)
        if tri is None:raise ValueError('Unmeshed face')
        ids=[]
        for i in range(1,tri.NbNodes()+1):
            v=tuple(round(k,7) for k in tri.Node(i).Transformed(loc.Transformation()).Coord())
            if v not in lookup:lookup[v]=len(verts);verts.append(v)
            ids.append(lookup[v])
        for i in range(1,tri.NbTriangles()+1):
            f=[ids[j-1] for j in tri.Triangle(i).Get()]
            if face.Orientation()==TopAbs_REVERSED:f.reverse()
            faces.append(f)
        it.Next()
    points=np.array(verts);p=points[np.array(faces)]-points.mean(axis=0)
    vol=abs(float(np.einsum('ij,ij->i',p[:,0],np.cross(p[:,1],p[:,2])).sum()/6))
    edges=Counter(tuple(sorted((a,b))) for f in faces for a,b in zip(f,f[1:]+f[:1]))
    props=GProp_GProps();BRepGProp.VolumeProperties_s(s,props,True);volume=props.Mass()
    return dict(v=verts,f=faces,volume=volume,closed=all(v==2 for v in edges.values()),error=abs(vol-volume))
solids,surfaces=load_solids(review/'package.step')
assert not surfaces and set(solids)==set(groupof)
bodies={}
for name,s in solids.items():
    category,material=describe(name,groupof[name]);bodies[name]=dict(**mesh(s),category=category,material=material,group=groupof[name])
candidates=candidate_pairs({n:bounds(s) for n,s in solids.items()},0,[])
static=[];unresolved=[]
for a,b in sorted(candidates):
    try:
        v=common(solids[a],solids[b])
        if v>1e-6:static.append(dict(pair=[a,b],volume=v))
    except Exception:unresolved.append([a,b])
def matrix(initial,moves,t):
    pose=gp_Trsf()
    for step in itertools.chain(transforms(initial,1),transforms(moves,t)):pose.PreMultiply(step)
    return [round(pose.Value(i,j),11) for i in (1,2,3) for j in (1,2,3,4)]
cases=[];times=read(review/'sources.json')['knotsSeconds']
for index,(c,obs) in enumerate(zip(study['cases'],result['cases'],strict=True)):
    assert c['id']==obs['case']
    frames={};removed=set(obs['removed'])
    for n in bodies.keys()-removed:
        ini=c.get('initialPose',{}).get(n,[]);moves=c['moves'].get(n,[])
        if ini or moves:frames[n]=[matrix(ini,moves,i/24) for i in (range(25) if moves else [0])]
    ts=times[index:index+2] if index<len(times)-1 else [times[-1],times[-1]]
    positions=[]
    for t in np.linspace(*ts,25):
        row=raw[np.abs(raw[:,0]-t).argmin()]
        positions.append([round(1000*x,3) for x in row[1:3]])
    cases.append(dict(id=c['id'],stage=c['stage'],basis=c['basis'],gap=c['minGapMm'],status=obs['status'],frames=frames,removed=list(removed),results=obs['pairs'],positions=positions))
rings={}
for n,col in [('Floor NOTE',9),('Preload NOTE',16)]:
    center=(raw[0,col:col+3]-[raw[0,1],raw[0,2],0])*1000
    w,x,y,z=raw[0,col+3:col+7]/np.linalg.norm(raw[0,col+3:col+7])
    rot=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    rings[n]=dict(center=center.tolist(),rot=rot.tolist())
elecs=[]
drive=groups['Swerve']['members'];feed=['Feeder motor'];active=drive+feed+['Wrist motor on boom','Rear finger gearmotor','Shoulder motor and reducer allowance']+groups['Controls']['members']+[n for n in bodies if n.startswith('Motor station')]
for e in result['budgets']['electricalCases']:
    src=budget['electricalCases'][e['case']];lines=[]
    for label,amps in src['batterySideA'].items():
        members=drive if 'traction' in label else feed if label=='powered feeder' else [n for n in active if n not in drive+feed] if label.startswith('scorer,') else active
        if label=='idle feeder and scorer':members=feed+[n for n in bodies if n.startswith('Motor station')]
        if label.startswith('two bounded'):members=['Wrist motor on boom','Shoulder motor and reducer allowance']+groups['Controls']['members']
        lines.append(dict(name=label,amps=amps,members=members))
    elecs.append(dict(**e,lines=lines,resistance=src['resistanceOhm'][1]))
mass=budget['masses'];subs=read(review.parents[1]/'linked-side-drive/moving-mass.json')
details=[dict(name=s+' slide',members=groups[s.title()+' pad']['members'],kg=[v['massKg']]*2) for s,v in subs['sides'].items()]
for label,members in [('battery',['Battery and terminal case']),('finished bumper scenario',['Complete blue bumper envelope']),('one NOTE',['Floor NOTE'])]:
    details.append(dict(name=label,members=members,kg=budget['runningAdditions'][label]))
data=dict(issue=result['issue'],step=result['step']['sha256'],motion=result['motion']['sha256'],receipt=sha(review/'review-result.json'),bodies=bodies,palette=palette,cases=cases,mass=[{k:r[k] for k in ('id','kg','members')} for r in mass],details=details,power=elecs,inspection=result['budgets']['inspectionKg'],running=result['budgets']['runningKg'],status=result['status'],control=result['control'],static=static,unresolved=unresolved,narrow=len(candidates),rings=rings,corners=corners,fieldSource=sha(goal),open=result['missingInputs'])
from profiles import nominal
new,info=nominal(solids,bounds,track,raw,read)
from installed_profiles import install
new=install(review,here,track,new,bodies,info,load_solids,bounds,mesh,sha)
for n,v in new.items():
    bodies[n]['nominal']=dict(**mesh(v),basis=info[n])
refined={**solids,**new}
data['originalCount']=len(solids)
from mechanical_motion import distance
data['preloadFit']=dict(gap=distance(refined['Feeder carriage envelope'],refined['Preload NOTE']),common=common(refined['Feeder carriage envelope'],refined['Preload NOTE']))
local_candidates=candidate_pairs({n:bounds(s) for n,s in refined.items()},0,[])
data['refinedParked']=[]
for a,b in sorted(local_candidates):
    v=common(refined[a],refined[b]);
    if v>1e-6:data['refinedParked'].append(dict(pair=[a,b],volume=v))
data['refinedNarrow']=len(local_candidates)
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString as TS
from OCP.TDataStd import TDataStd_Name
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.IFSelect import IFSelect_RetDone
newdoc=TDocStd_Document(TS('BinXCAF'));table=XCAFDoc_DocumentTool.ShapeTool_s(newdoc.Main())
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
for n,v in refined.items():
    ex=TopExp_Explorer(v,TopAbs_SOLID);solid=ex.Current();ex.Next();assert not ex.More()
    placed=BRepBuilderAPI_Transform(solid.Located(TopLoc_Location()),solid.Location().Transformation(),True).Shape()
    TDataStd_Name.Set_s(table.AddShape(placed,False),TS(n))
writer=STEPCAFControl_Writer();assert writer.Transfer(newdoc,STEPControl_AsIs) and writer.Write(str(here/'supported-layout.step'))==IFSelect_RetDone
members=read(here/'upper-members.json')
for title,sides in [('upper-left travel',[0]),('upper-right travel',[1]),('both upper wheels under load',[0,1])]:
    move={n:[dict(kind='translation',travelMm=[5.1961524227,0,3])] for i in sides for n in members[i]}
    case=dict(id=title,stage='operating',basis='Independent guide stroke to 6 mm. Spring envelope remains at home length; compression/load and tolerance are open.',moves=move,minGapMm=.5,maxDepth=6)
    study['cases'].append(case);cases.append(dict(id=title,stage='operating',basis=case['basis'],gap=.5,status='pending',frames={n:[matrix([],o,i/24) for i in range(25)] for n,o in move.items()},removed=[],results=[],positions=cases[0]['positions']))
pinned=sha(here/'supported-layout.step');data['nominalSha']=pinned
(here/'supported-paths.json').write_text(json.dumps(dict(study,step=dict(file='supported-layout.step',sha256=pinned))))
allocation=next(Path(k) for k in budget['references'] if k.endswith('inspection-allocation.json'));verify(allocation,budget['references'][str(allocation)])
estimates=read(allocation)['cases'];mappings={'one planned shoulder':['Shoulder motor and reducer allowance'],'second fixed scorer':groups['Scorer']['members'],'two powered moving uppers':['Wheel station 2','Motor station 2','Wheel station 3','Motor station 3'],'two stationary upper mounting':['Wheel station 2','Motor station 2','Wheel station 3','Motor station 3'],'wrist drive, chain':['Wrist motor on boom'],'electrical devices excluding':groups['Controls']['members']+groups['Service ladder 1']['members']+groups['Service ladder 2']['members']+groups['Service ladder 3']['members'],'robot-side leads':['positive power lead corridor','negative power lead corridor']}
for label,value in estimates['high']['rowsKg'].items():
    for prefix,members in mappings.items():
        if label.startswith(prefix):details.append(dict(name=label,members=members,kg=[estimates['low']['rowsKg'][label],value]))
for n in ['Boom -203','Boom 203']:
    mass=bodies[n]['nominal']['volume']*2700/1e9;details.append(dict(name=n+' hollow aluminum estimate',members=[n],kg=[mass,mass]))
for e in elecs:
    for line in e['lines']:
        if 'traction' in line['name']:line['members']=[n for n in bodies if n.startswith('Swerve drive motor')]
for n in bodies:
    if n.startswith('Swerve drive motor'):details.append(dict(name=n+' catalog',members=[n],kg=[.425,.425]))
tmp=dest/'data.tmp';tmp.write_text(json.dumps(data,separators=(',',':'),allow_nan=False),encoding='utf8');tmp.replace(dest/'data.json')
print(len(bodies),'solids;',len(cases),'cases;',len(static),'parked overlaps;',len(unresolved),'unresolved;',sum(not b['closed'] for b in bodies.values()),'unclosed meshes')

