"""Check translation AND yaw at each blocked wheel contact, with the 101.6 mm modeled wheels."""
from pathlib import Path
import sys,json,numpy as np,math,hashlib
p=Path(__file__).resolve().parent;source=p.parent/'six-piece-02/plan.json'
plan=json.loads(source.read_bytes())['plan'];mass=json.loads((p/'budget.json').read_bytes())['runningKg'][1]
radius=.0508;offset=np.array([[x,y] for x in [-.249,.249] for y in [-.279,.279]])
def J(v):return np.stack((-v[...,1],v[...,0]),axis=-1)
def segment(label,phase,i,row,ratio,aux):
    start,end=np.array(row[phase][i:i+2]);T=row[phase+'T'][i];diff=end-start
    h0=row['yaw'] if phase=='loaded' and label=='middle' and i<=1 else 0.
    dh=-h0 if phase=='loaded' and i==1 else 0.
    u=np.linspace(0,1,501);b=u*u*(3-2*u);rate=6*u*(1-u)/T;a=(6-12*u)/(T*T)
    yaw=h0+dh*b;omega=dh*rate;alpha=dh*a
    wheels=np.stack([offset[:,0]*np.cos(yaw[:,None])-offset[:,1]*np.sin(yaw[:,None]),offset[:,0]*np.sin(yaw[:,None])+offset[:,1]*np.cos(yaw[:,None])],axis=-1)
    # Direction remains defined through zero speed using the limiting path tangent.
    tangent=diff+dh*J(wheels);length=np.linalg.norm(tangent,axis=-1)
    direction=tangent/np.maximum(length[:,:,None],1.e-10);speed=length*rate[:,None]
    radialSq=np.sum(offset*offset,axis=1)
    total=(mass/4)*(a[:,None,None]*diff)+(10./4)*alpha[:,None,None]*J(wheels)/radialSq[None,:,None]
    force=(total*direction).sum(axis=-1)+mass*9.80665*.025/4
    peakMu=np.max(np.linalg.norm(total+direction*(mass*9.80665*.025/4),axis=-1)/(mass*9.80665/4))
    kt=2.6/105;R=12/105;ke=(12-R*1.8)/(5676*math.tau/60)
    phaseA=force*radius/(ratio*.82*kt);motorV=ke*speed*ratio/radius+phaseA*R
    watts=np.maximum(0,phaseA*motorV).sum(axis=1)/.95
    E=11.8-.027*aux;disc=E**2-4*.027*watts;bus=(E+np.sqrt(np.maximum(0,disc)))/2
    return dict(id=label,phase=phase,segment=i,ratio=ratio,auxA=aux,minimumV=float(bus.min()),reserveV=float(np.min(bus[:,None]-abs(motorV))),maxPhaseA=float(np.max(abs(phaseA))),mu=float(peakMu),passScreen=bool(disc.min()>0 and bus.min()>=8 and np.min(bus[:,None]-abs(motorV))>=0 and abs(phaseA).max()<=50 and peakMu<=.55))
rows=[segment(leg['id'],phase,i,leg,ratio,(24 if phase=='out' else aux)) for ratio in [5.5,6.75] for aux in [39,92] for leg in plan for phase in ['out','loaded'] for i in range(len(leg[phase+'T']))]
groups=[]
for gear in (5.5,6.75):
    for current in (39,92):
        cases=[r for r in rows if r['ratio']==gear and r['auxA'] in (24,current)]
        groups.append(dict(gear=gear,loadedAuxA=current,minBusV=min(r['minimumV'] for r in cases),maxPhaseA=max(r['maxPhaseA'] for r in cases),worstVoltageReserve=min(r['reserveV'] for r in cases),failed=[r['id']+' '+r['phase']+' '+str(r['segment']) for r in cases if not r['passScreen']]))
out=dict(source=hashlib.sha256(source.read_bytes()).hexdigest(),runningKg=mass,tireRadiusM=radius,contactCentersM=offset.tolist(),groups=groups,rows=rows,scope='101.6 mm nominal blocked tire. Four equal normal loads, 10 kg m² yaw inertia, equal planar force and yaw-moment allocation, braking energy dissipated, weak pack 11.8 V/27 mOhm. Ratios are sensitivity inputs. Module selection, steering transient and dynamic wheel loads remain open.')
(p/'four-wheel-duty.json').write_text(json.dumps(out,indent=2));print(groups)

