"""Native sampled-force feasibility and independent contact kinematics checks."""
import argparse,copy,hashlib,json,pathlib,sys
import numpy as np
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
p=argparse.ArgumentParser()
for name in ['project','source','binding','base-contract','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
p.add_argument('--gap',type=float,default=1e-12);p.add_argument('--iterations',type=int,default=1000)
a=p.parse_args();assert not a.output.exists();sys.path[:0]=[str(a.project/f'sim/{x}') for x in ['pressure','integrated','drivetrain','actuation','acquisition']]
from native_static_lift import NativeStaticLift
from native_dry_friction import contact_jacobian,solve_disks
from patch_registration import rotation
load=lambda p:json.loads(p.read_bytes());source=load(a.source);dc=load(a.project/'sim/drivetrain/config.json');nc=load(a.project/'sim/acquisition/free-planar-config.json')['model'];cad=load(a.project/'cad/assembly/intake_components.json');ac=load(a.project/'sim/actuation/config.json');m=NativeStaticLift(source['final'],dc,nc,cad,load(a.binding),ac,load(a.base_contract));s=m.s;pressure=m.contact(s,{})['pressure'];j,basis=contact_jacobian(pressure,nc['radiusM']);v=np.array([.13,-.07,.21,-.03,.11,-.17,.4,-.6]);rn=pressure['noteLeverWorldM'];rc=pressure['cradleLeverWorldM'];eps=1e-7
# Differentiate the actual rotation convention, independently of Jacobian code.
def moved(sign):
 nrot=np.array(rotation(0.,float(sign*eps*v[5])));crot=np.array(rotation(0.,float(sign*eps*v[2])));note=rn@nrot.T+sign*eps*np.array([v[3],0.,v[4]]);cradle=rc@crot.T+sign*eps*np.array([v[0],0.,v[1]])+sign*eps*nc['radiusM']*v[6+pressure['motor']][:,None]*pressure['tangentWorld'];return note-cradle
fd=(moved(1)-moved(-1))/(2*eps);observed=np.einsum('nij,nj->ni',basis,fd);predicted=np.einsum('nik,k->ni',j,v);err=float(np.max(abs(observed-predicted)));assert err<1e-8
mass=m.mass(s);weight=nc['pieceMassKg']*9.80665;offset=s['note']['p'][0]-s['chassis']['x']-.178;force=np.zeros(8);force[1]=weight;force[2]=weight*offset;force[4]=-weight;dt=.0005;free=dt*np.linalg.solve(mass,force);cases=[]
for mu in [0.,.1,.2,.3,.32,.4,.6]:
 r=solve_disks(j,mass,free,dt*mu*pressure['normalN'],gap_tolerance=a.gap,max_iterations=a.iterations);impulses=np.einsum('ni,nij->nj',r['impulse'],basis);bodyforce=np.sum(impulses,axis=0)/dt;residual=force+j.reshape(-1,8).T@r['impulse'].ravel()/dt;maximum_acc=float(np.max(abs(np.linalg.solve(mass,residual))));cases.append(dict(mu=mu,verticalSupportN=float(bodyforce[2]),weightN=weight,maximumGeneralizedAcceleration=maximum_acc,equilibriumWitness=maximum_acc<1e-5,maximumGeneralizedForceResidual=float(np.max(abs(residual))),maximumConeExcessNs=r['maximumConeExcessNs'],gapJ=r['gapJ'],iterations=r['iterations']))
assert not cases[0]['equilibriumWitness'] and not cases[1]['equilibriumWitness']
out=dict(schema='native-static-support-witness/1',inputs={label:sha(path) for label,path in [('producer',pathlib.Path(__file__)),('friction',pathlib.Path('native_dry_friction.py')),('model',pathlib.Path('native_static_lift.py')),('sourceReceipt',a.source),('binding',a.binding),('baseContract',a.base_contract)]},solverGapToleranceJ=a.gap,solverMaxIterations=a.iterations,contactCount=pressure['contacts'],kinematicFiniteDifferenceMaximumMps=err,staticCases=cases,nativeMovingMassBound=False,acceptance=False,limits=['Prescribed zero-velocity equilibrium feasibility at the attained geometry; not a new trajectory or calibrated static coefficient','Cradle gravity is balanced by an ideal test effort plus the explicitly reported NOTE load/moment; actual finite actuation is tested in separate dynamic runs','All eight generalized velocities include finite free pinch rotor inertia; no rotor lock or NOTE attachment','Only the existing normal pressure field sets friction capacities'])
a.output.write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(out,indent=2))
