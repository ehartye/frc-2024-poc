from pathlib import Path
base=Path(__file__).resolve().parent
s=(base/'entry.py').read_text();exec(compile(s[:s.index('nominal=check(')],'hook fixture','exec'))
from showcase_contacts import load_solids
raw,_=load_solids(root/'follower-01/paired-follower.step')
d=design;roster=next(c['rig'] for c in d['cases'] if c['key']=='pickup-to-shot')
P=mat(125,185,0,501);ip=np.linalg.inv(P)
def label(n):
    while chr(194)+chr(183) in n:n=n.replace(chr(194)+chr(183),chr(183))
    return n
G={label(n):sh for n,sh in raw.items() if label(n)!='Chambered NOTE'}
field=Path('C:/Users/ehart/Documents/FRC Native/2024/six-speaker-20261118/field-source/whole-field-sliced-bounds.json')
rows=json.loads(field.read_bytes())['parts'];ob=[]
for part in rows:
    if part['path'][1]!='GE-24200_Blue':continue
    for a,b in part.get('tightBounds',[part['bounds']]):
        if a[0]<6.65 and b[0]>5.15 and a[1]<4.55 and b[1]>3.64 and a[2]<1.22:
            if part['path'][-1]=='GE-24205_hanging' and b[0]-a[0]<.1:continue
            ob.append((np.array(a)*1000,np.array(b)*1000,'/'.join(part['path'])))
cross=BRepAlgoAPI_Common(chain,box(cx-25,nom_y-470,700,50,940,260)).Shape();CB=np.array(bounds(cross));mins=1e6;hits=[];fieldPairs=set();checks=0;phase={}
def vis(m):return m[:3].round(7).ravel().tolist()
for idx,(dist,gate) in enumerate([(x,35) for x in np.linspace(238,180,15)]+[(180,z) for z in np.linspace(35,0,9)[1:]]):
    th,x=ik(dist,cz);pivot=mat(th,x+185,nom_y,501);frame=mat(0,x,nom_y,0)
    ext={}
    for n,b in G.items():
        role=roster[n];m=pivot@ip if role in ('arm','flap') else frame
        if n.startswith('Hook keeper'):m=pivot@mat(0,0,0,gate)@ip
        placed=place(b,m);bb=np.array(bounds(placed));ext[n]=bb
        for a,bnd,on in ob:
            if all(bb[3:]>=a)&all(bnd>=bb[:3]):fieldPairs.add((n,on))
        if any(bb[3:]<CB[:3]-3) or any(CB[3:]<bb[:3]-3):continue
        # Narrow with short local Y slab on the actual chain.
        mask=BRepAlgoAPI_Common(cross,box(cx-24,bb[1]-.1,690,48,max(.21,bb[4]-bb[1]+.2),290)).Shape()
        if vol(mask)<.001:continue
        v=common(placed,mask);sep=distance(placed,mask);checks+=1;mins=min(mins,sep)
        if v>.001:hits.append([idx,n,v])
    pos=[]
    for side in ('left','right'):
        nh='Chain hook · '+side;nk='Hook keeper · '+side
        k=place(G[nk],pivot@mat(0,0,0,gate)@ip);hook=place(G[nh],pivot@ip)
        pos.append(common(k,hook))
    phase[str(idx)]=dict(th=th,x=x,gate=gate,overlapKeeperToHook=pos)
    if idx%4==0:(p/'entry-progress.json').write_text(json.dumps(dict(pose=idx,chainCommon=hits,otherCandidates=list(fieldPairs),selfControls=pos)))
res=dict(chain=sha if False else json.loads((p/'source.json').read_bytes())['brep'],parent=d['step'],tested=len(G),exactChainChecks=checks,closestMm=mins,positiveChain=hits,otherFieldCandidates=list(fieldPairs),poses=phase)
(p/'root-entry.json').write_text(json.dumps(res,indent=2));print(len(hits),len(fieldPairs),checks,phase['20'])

