"""Replay the attained release; hold robot history fixed while varying the field time step."""
from pathlib import Path
import sys,hashlib,json,math
from types import SimpleNamespace
from concurrent.futures import ProcessPoolExecutor
import numpy as np, mujoco as mj
p=Path(__file__).resolve().parent
src=p.parent/'mission-physics/shot.py'
bootstrap=src.read_text(encoding='utf8').split('def shot(')[0]
env={'__file__':str(src)};exec(bootstrap,env)
def trial(job):
    key,dt=job
    raw=p.parent/'split-field-02'/f'split-ready-{key}-center-low-96-6.88.json'
    doc=json.loads(raw.read_bytes());release=doc['release']; mass=.2413;major=.1524;minor=.0254
    ix=mass*(major**2/2+5*minor**2/8);iz=mass*(major**2+3*minor**2/4)
    model=mj.MjModel.from_xml_string(f'<mujoco><option timestep="{dt}" integrator="implicitfast" gravity="0 0 -9.80665"/><worldbody><body><freejoint/><inertial pos="0 0 0" mass="{mass}" diaginertia="{ix} {ix} {iz}"/></body></worldbody></mujoco>')
    d=mj.MjData(model);d.qpos[:]=release['q'];d.qvel[:]=release['v'];d.time=release['t'];mj.mj_forward(model,d)
    n=doc['n'];theta=np.arange(n)*math.tau/n;local=major*np.stack([np.cos(theta),np.sin(theta),np.zeros(n)],axis=1)
    f=env['SpeakerSurfaces'](env['field']);parameters=SimpleNamespace(stiffness=1.2e6,damping=450.,friction=.45,belt_speed=1.5)
    rows=[];firstContact=None;points=[]
    try:
        for step in range(round((7-d.time)/dt)+1):
            mj.mj_step1(model,d);R=d.xmat[1].reshape(3,3);q=d.qpos.copy();v=d.qvel.copy();w=np.zeros(6)
            if q[0]<.8:
                w,touch,_=env['field_wrench'](f,local@R.T+q[:3],q[:3],v[:3],R@v[3:],parameters)
                if touch and firstContact is None:firstContact=float(d.time)
            rows.append(dict(t=d.time,q=q.tolist()))
            if q[2]<.17:break
            d.xfrc_applied[1]=w;mj.mj_step2(model,d)
    finally:f.close()
    opening=env['opening_passage'](rows)
    ends={row['key']:np.array(row['caseEndCornersM']) for row in env['sensor']}
    groups=[env['beam_samples'](ends[a],ends[b]) for a,b in env['target']['beamPairs']]
    both=None;closest=-math.inf
    for r in rows:
        if not opening or r['t']<opening['fullyThroughByS']:continue
        q=np.array(r['q'])
        marg=min(env['shadow_margin'](q,b) for b in groups);closest=max(closest,float(marg))
        if marg>=0:both=r['t'];break
    # Independently compare prior saved pre-contact poses on their same simulation clocks.
    times=np.array([r['t'] for r in rows]);checks=[]
    for r in doc['trace']:
        if r['t']<=release['t'] or firstContact is None or r['t']>=firstContact-.001:continue
        i=np.argmin(abs(times-r['t']))
        if abs(times[i]-r['t'])<1.e-8:checks.append(float(np.linalg.norm(np.subtract(r['q'][:3],rows[i]['q'][:3]))*1000))
    return dict(key=key,dt=dt,source=hashlib.sha256(raw.read_bytes()).hexdigest(),release=release['t'],precontactChecks=len(checks),maxPrecontactErrorMm=max(checks,default=None),opening=opening,both=both,originalBoth=doc['both'],firstContact=firstContact,bestPairedMargin=closest if np.isfinite(closest) else None,last=rows[-1])
if __name__=='__main__':
    jobs=[(k,t) for t in [.00025,.000125] for k in ('R01','R02')]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for r in pool.map(trial,jobs):
            (p/f"replay-{r['key']}-{r['dt']}.json").write_text(json.dumps(r,indent=2));print(r['key'],r['dt'],r['maxPrecontactErrorMm'],r['both'],flush=True)

