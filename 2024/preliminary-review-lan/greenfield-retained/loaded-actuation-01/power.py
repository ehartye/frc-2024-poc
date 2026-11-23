"""Geometry-bounded upper demand during the actual loaded route and shared supply."""
from slide_demand import *
from smooth import table,times
doc=read(root.parent/'www/data.json');g=doc['concepts']['guided'];f=doc['concepts']['field']
assert sha(root/'guided-hoist-01/service-locked-robot.step')==g['step']
bodies={**doc['bodies'],**f['add'],**g['add']}
s=next(c for c in g['cases'] if c['key']=='guided-service')
omit=(set(f['remove'])-set(g['add']))|set(g['fixtureOnlyNames'])
rig={k:v for k,v in s['rig'].items() if k in bodies and k not in omit}
arm={k for k,v in rig.items() if v in ('arm','flap')};slide={k for k,v in rig.items() if v in ('slide','servicePin')}
# Storage meshes are already at the parked 125-degree pose.
ang=math.radians(125);rot=np.array([[math.cos(ang),0,-math.sin(ang)],[0,1,0],[math.sin(ang),0,math.cos(ang)]])
def extents(names):
    v=(np.vstack([bodies[n]['v'] for n in names])-[185,0,501])@rot/1000
    points=np.array([[x,y,z] for x in (v[:,0].min(),v[:,0].max()) for y in (v[:,1].min(),v[:,1].max()) for z in (v[:,2].min(),v[:,2].max())])
    return points
allrot=extents(arm)
rows=[]
for row in g['mass']:
    name=row['id'];m=row['kg'][1];ns=set(row['members'])
    if name=='Replacement traction tread and hardware allowance':ns={n for n in arm if 'tread' in n.lower()}
    if name=='Cassette drive':ns|={n for n in arm if n.startswith(('Launch Belt motor','Launch Belt drive shaft','Feed motor'))}
    if m<=0 or not ns&(arm|slide):continue
    vs=extents(ns&arm) if ns&arm else np.zeros((1,3))
    rows.append((name,m,vs,sorted(ns&(arm|slide))))
# This is a bounded held NOTE over the measured retained coordinate window, using its full outside radius.
note=np.array([[x,y,z] for x in (-.058,.528) for y in (-.228,.228) for z in (-.008,.116)])
rows.append(('Loaded NOTE',.2413,note,[]))
plan=read(p/'plan.json');driveMass=g['running'][1]
def run(leg,window,aux,extra,liftS=3.4):
    T=np.array(leg['loadedT']);dt=.001;t=np.arange(0,max(liftS+(T[0] if leg['id']=='middle' else 0),sum(T))+.002,dt)
    clock=t-(T[0] if leg['id']=='middle' else 0);profileClock=clock*(3.4/liftS);angle,x=law(profileClock);q=np.radians(angle);velq=np.gradient(q,dt);accq=np.gradient(velq,dt)
    if window:
        tab,_=table(a0,a1,x0);x=np.interp(profileClock,times,tab)
    vx=np.gradient(x,dt);ax=np.gradient(vx,dt)
    wheels=np.array([[xx,yy] for xx in [-.249,.249] for yy in [-.279,.279]])
    P=np.array(leg['loaded']);power=np.zeros(len(t));motorPeak=power.copy();axBody=power.copy();om=power.copy();alpha=power.copy();start=0.
    for i,D in enumerate(T):
        mask=(t>=start)&(t<start+D);u=(t[mask]-start)/D;start+=D
        b=u*u*(3-2*u);rate=6*u*(1-u)/D;acc=(6-12*u)/D**2;delta=P[i+1]-P[i]
        q0=leg['yaw'] if leg['id']=='middle' and i<2 else 0;dq=-q0 if leg['id']=='middle' and i==1 else 0
        yaw=q0+dq*b;co=np.cos(yaw);si=np.sin(yaw)
        axBody[mask]=(delta[0]*co+delta[1]*si)*acc;om[mask]=dq*rate;alpha[mask]=dq*acc
        loc=np.stack([co[:,None]*wheels[:,0]-si[:,None]*wheels[:,1],si[:,None]*wheels[:,0]+co[:,None]*wheels[:,1]],axis=-1)
        j=np.stack([-loc[:,:,1],loc[:,:,0]],axis=-1);vec=delta+dq*j;norm=np.linalg.norm(vec,axis=-1);direct=vec/np.maximum(norm[:,:,None],1e-10)
        force=driveMass/4*acc[:,None,None]*delta+(10./4)*alpha[mask,None,None]*j/np.sum(wheels*wheels,axis=1)[None,:,None]
        F=(force*direct).sum(axis=-1)+driveMass*9.80665*.025/4
        amps=F*.0508/(5.5*.82*(2.6/105));motorV=(12-12/105*1.8)/(5676*math.tau/60)*norm*rate[:,None]*5.5/.0508+amps*12/105
        power[mask]=np.maximum(0,amps*motorV).sum(axis=1)/.95;motorPeak[mask]=abs(motorV).max(axis=1)
    low=np.zeros(len(t));high=low.copy();fl=low.copy();fh=low.copy();cc=np.cos(q)[:,None];ss=np.sin(q)[:,None]
    for name,m,v,_ in rows+[('Additional mass from reserve',extra,allrot,[])]:
        if m<=0:continue
        X=cc*v[:,0]-ss*v[:,2];Z=ss*v[:,0]+cc*v[:,2]
        # Generalized force for pitch and slide includes vehicle translation/yaw, gravity and both accelerations.
        # A point on the slide-only stock contributes translation only. Its Y extent is bounded by half frame width.
        trans=(axBody+ax)[:,None]-alpha[:,None]*v[:,1]-om[:,None]**2*(x[:,None]+X)
        tau=m*(9.80665*X-trans*Z+accq[:,None]*(X**2+Z**2))
        force=m*(trans-accq[:,None]*Z-velq[:,None]**2*X)
        # Mixed stationary/moving budget rows can assign mass to fixed structure or anywhere in the enclosed group.
        low+=np.minimum(tau.min(axis=1),0);high+=np.maximum(tau.max(axis=1),0)
        fl+=np.minimum(force.min(axis=1),0);fh+=np.maximum(force.max(axis=1),0)
    # Point mass quadratic in local coordinates can have interior extrema. Add an interval bound for the radial inertia
    # and yaw cross-term instead of relying on vertex-enumeration extrema of a quadratic.
    # For pitch inertia r² is convex: chord overestimate <= (spanX²+spanZ²)/4; yaw X*Z uses the same Hessian bound.
    maxChord=sum(m*(np.ptp(v[:,0])**2+np.ptp(v[:,2])**2)/4 for _,m,v,_ in rows+[('reserve',extra,allrot,[])])
    amend=maxChord*(abs(accq)+om**2);low-=amend;high+=amend
    # 2 N m preliminary pivot drag, 15 N guide and screw friction independent of drivetrain rolling resistance.
    lowI=(low-2.+accq*.00004*250**2)/(250*.0198*.72);highI=(high+2.+accq*.00004*250**2)/(250*.0198*.72)
    pitchI=np.maximum(abs(lowI),abs(highI))
    slideForce=np.maximum(abs(fl),abs(fh))+15
    # Output is the required screw speed and torque. Use an explicit example motor as a sensitivity, without SKU credit.
    lead=.008;eta=.65;screwJ=2.e-5
    slideI=(slideForce*lead/(math.tau*eta)+abs(ax)*math.tau/lead*screwJ)/.0097
    pv=abs(velq)*250*.0202+pitchI*.115;sv=abs(vx)*math.tau/lead*.010+slideI*.13
    watts=power+(pitchI*pv+slideI*sv)/.95
    boundary=11.8-.027*aux;disc=boundary**2-4*.027*watts
    bus=np.where(disc>=0,(boundary+np.sqrt(np.maximum(0,disc)))/2,0)
    idx=int(np.argmin(bus));i=int(np.argmax(pitchI))
    return dict(piece=leg['id'],windowS=window*liftS/3.4,liftS=liftS,minSlideTerminalReserveV=float(min(bus-sv)),minPivotTerminalReserveV=float(min(bus-pv)),peakSlideMotorV=float(max(sv)),peakPivotMotorV=float(max(pv)),auxA=aux,additionalDistalKg=extra,minPitchCurrentA=float(min(lowI)),maxPitchCurrentA=float(max(highI)),maxPitchPhaseA=float(max(pitchI)),peakPitchAtS=t[i],peakPitchAngleDeg=angle[i],maxSlidePhaseA=float(max(slideI)),maxSlideForceN=float(max(slideForce)),noBusSolution=bool(min(disc)<0),weakBusMinV=float(min(bus)) if min(disc)>=0 else None,peakWatts=float(max(watts)),worstTimeS=t[idx],worstPitchV=pv[idx],worstSlideV=sv[idx],wheelPowerAtWorstW=power[idx],minimumMotorVoltageReserveV=float(min(bus-np.maximum.reduce((pv,sv,motorPeak)))) if min(disc)>=0 else None)
if __name__=='__main__':
    cases=[run(leg,window,aux,extra) for leg in plan['plan'] for window,aux,extra in [(0,25,0),(.16,25,0),(.16,25,2),(.16,92,0)]]
    data=dict(step=g['step'],plan=sha(p/'plan.json'),inputs=[dict(group=n,upperKg=m,boxBoundsM=[v.min(axis=0).tolist(),v.max(axis=0).tolist()],members=members) for n,m,v,members in rows],results=cases,scope='High grouped mass with worst-corner moment and force bounds. Zero guaranteed regeneration. Four 5.5:1 swerve drives, 250:1 shared pivot, and a sizing example for a direct 8 mm lead screw at 65% efficiency, 200 g cm² effective screw/motor inertia (2e-5 kg m²), 0.0097 Nm/A, 0.010 V s/rad, 0.13 ohm. Rotor/lead choice, drivetrain yaw inertia and rolling resistance, moving friction, mass locations, and true current/voltage require qualification. Negative margin on a bound is a design hold, not measured stall.')
    (p/'power.json').write_text(json.dumps(data,indent=2))
    for r in cases[::4]+cases[1::4]:print(r['piece'],r['windowS'],round(r['maxPitchPhaseA'],1),round(r['maxSlidePhaseA'],1),r['weakBusMinV'])

