"""Replace ideal hook/chain point joints with unilateral contact and two finite spring-loaded gates."""
from pathlib import Path
import sys,math,json,builtins,numpy as np,hashlib
here=Path(__file__).resolve().parent;root=here.parent;method=root/'catch-layout-03'
label,members,dt,cg,mode=sys.argv[1:];members=int(members)
sources=json.loads((here/'collision-hook.json').read_bytes());chainCenters=json.loads((here/'rest-chain-centers.json').read_bytes())[str(members)]
sys.path.insert(0,str(method));realCompile=builtins.compile
def convert(src):
    src=src.replace('n=24',f'n={members}')
    a=src.index('span=');b=src.index('def fmt(',a)
    src=src[:a]+"""raw=np.array(chainCenters)/1000.
ys=raw[:,1]-4.10432;z=raw[:,2]
k1,k2=n//2-n//24,n//2+n//24
start=math.radians(75.38642106234)
cx=.185+.18*np.cos(start)-.232*np.sin(start)
nodes=np.column_stack([cx+raw[:,0]-(raw[k1,0]+raw[k2,0])/2,ys,z])
"""+src[b:]
    src=src.replace('mass=".085" contype="0" conaffinity="0"', 'mass="{.085*24/n}" contype="4" conaffinity="8"')
    # The complete native hook profile, decomposed into convex prisms. Finite normal contact can slide or release.
    asset='';hardware=''
    for i,part in enumerate(sources['parts']):
        low,high=part['y']
        verts=' '.join(str(x/1000) for y in (low,high) for xx,zz in part['polygon'] for x in (xx,y,zz))
        asset+=f'<mesh name="h{i}" vertex="{verts}" inertia="convex"/>'
    mask=0 if mode=='absent' else 8
    for side,yy in enumerate((-.10088,.10088)):
        for i in range(len(sources['parts'])):
            hardware+=f'<geom type="mesh" name="hook{side}_{i}" mesh="h{i}" pos="0 {yy} 0" contype="{mask}" conaffinity="4" mass="0" friction=".2 .001 .001" solref=".003 1"/>'
        hardware+=f'<body pos=".2035 {yy} .2405"><joint name="gate{side}" type="slide" axis="0 0 1" range="0 .044" limited="true" stiffness="180" damping="1" springref="-.010" solreflimit=".002 1"/><geom name="keeper{side}" type="box" size=".0025 .004 .0325" mass=".028" contype="{mask}" conaffinity="4" friction=".2 .001 .001" solref=".003 1"/></body>'
    src=src.replace('<worldbody>', '<asset>'+asset+'</asset><worldbody>').replace('<site name="h1"',hardware+'<site name="h1"')
    src=src.replace('fr=m.body', '''gates=[m.joint('gate'+str(i)) for i in range(2)]
for g in gates:d.qpos[g.qposadr[0]]=.044
fr=m.body''')
    a=src.index('    if t>=.25 and');b=src.index('    u=np.clip',a)
    src=src[:a]+"""    # Simulated open command releases at 100 ms. No body or site is attached to a hook.
    for j,g in enumerate(gates):
        opening=t<.1 or mode=='open'
        v=d.qvel[g.dofadr[0]]
        d.qfrc_applied[g.dofadr[0]]=np.clip(5000*(.044-d.qpos[g.qposadr[0]])-5*v+9.72,0,25) if opening else 0
    if t>=.13 and len(grasp)<2 and mode!='absent':
        rot=d.xmat[m.body('cassette').id].reshape(3,3);orig=d.xpos[m.body('cassette').id]
        inside=[rot.T@(d.site_xpos[chainids[k]]-orig) for k in (k1,k2)]
        if all(d.qpos[g.qposadr[0]]<.001 and np.linalg.norm(v[[0,2]]-[.18,.232])<.018 for g,v in zip(gates,inside)):
            grasp=[[t,j,v.tolist()] for j,v in enumerate(inside)]
"""+src[b:]
    src=src.replace('demand=3000*','target=target if len(grasp)==2 else start\n    demand=3000*')
    src=src.replace('cut=','cut=',1)
    src=src.replace('chainids=[','maxContacts=[0.,0.];loadedContacts=[0.,0.];proofSamples=0;maxDrift=0.;closures=[]\nchainids=[')
    idx=src.index('        corners=')
    src=src[:idx]+"""        cf=np.zeros(6);fside=np.zeros(2)
        for k in range(d.ncon):
            ct=d.contact[k];nms=[m.geom(gg).name or '' for gg in (ct.geom1,ct.geom2)]
            for sideidx in (0,1):
                if any(h.startswith(('hook'+str(sideidx)+'_','keeper'+str(sideidx))) for h in nms):
                    mj.mj_contactForce(m,d,k,cf);fside[sideidx]+=cf[0]
        maxContacts=np.maximum(maxContacts,fside)
        if t>=cut+.1:
            loadedContacts=np.asarray(loadedContacts)+fside;proofSamples+=1
        if i%round(.025/dt)==0:closures.append([t,*[d.qpos[g.qposadr[0]] for g in gates],*fside])
"""+src[idx:]
    src=src.replace("Path(sys.argv[1]+'.json').write_text","""r.update(physicalHookNormalPeakN=maxContacts.tolist(),meanDisabledSideNormalN=(np.array(loadedContacts)/max(1,proofSamples)).tolist(),gateTrace=closures,enabledHookEqualities=d.eq_active[1:3].tolist(),chainMembers=n,mode=mode,geometry=sources['source'])
assert not any(d.eq_active[1:3])
Path(sys.argv[1]+'.json').write_text""")
    src=src.replace("print({k:v for k,v in r.items() if k!='trace'})","print({k:v for k,v in r.items() if k not in ('trace','gateTrace')})")
    return src
def intercept(src,filename,*args,**kwargs):
    if filename=='unilateral stop experiment':
        src=convert(src);sys.argv[1]=str(here/label)
    return realCompile(src,filename,*args,**kwargs)
sys.argv=[str(method/'controlled.py'),label,dt,cg,'1','90','8']
env=dict(__file__=str(method/'controlled.py'),__name__='__main__',chainCenters=chainCenters,sources=sources,mode=mode,compile=intercept,math=math)
exec(realCompile((method/'controlled.py').read_bytes(),'controlled hook contact','exec'),env)

