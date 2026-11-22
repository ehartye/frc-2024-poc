"""Four-module inverse drive and weak-battery screen on the commissioned route ramps."""
from pathlib import Path
import json,math,numpy as np
p=Path(__file__).resolve().parent;leg=json.loads((p/'plan.json').read_bytes())['plan']
cad=json.loads((p.parent/'follower-01/review-data.json').read_bytes())
module=json.loads(Path('C:/Users/ehart/repos/frc-2024-poc/cad/drivetrain/maxswerve-layout.json').read_bytes())['module']
m=cad['running'][1];r=module['wheelDiameterMm']/2000;gear=module['driveReduction']
R=12/105;kt=2.6/105;ke=(12-R*1.8)/(5676*math.tau/60)
runs=[]
def check(k,phase,i,T,L,turn):
    data=[];u=np.linspace(0,1,1001)
    if turn:L=abs(turn)*.345
    for h in u:
        v=L*6*h*(1-h)/T;a=L*(6-12*h)/(T*T)
        F=(10./.345**2 if turn else m)*a+m*9.81*.025
        I=F*r/(4*gear*.82*kt);w=v*gear/r;V=ke*w+R*I;watts=max(0,4*V*I/.95)
        other=39 if phase=='loaded' else 24
        E=11.8-.027*other;disc=E*E-4*.027*watts;bus=(E+math.sqrt(max(0,disc)))/2
        data.append([bus,abs(I),abs(V),bus-abs(V),abs(F)/(m*9.81),disc])
    data=np.array(data);return dict(id=k,phase=phase,segment=i,timeS=T,lengthM=L,minBusV=float(min(data[:,0])),maxPhaseA=float(max(data[:,1])),voltageReserveV=float(min(data[:,3])),requiredFriction=float(max(data[:,4])),passScreen=bool(min(data[:,0])>=8 and min(data[:,3])>=0 and max(data[:,1])<=50 and max(data[:,4])<=.55 and min(data[:,5])>0))
for row in leg:
    for phase in ('out','loaded'):
        path=np.array(row[phase]);dur=row[phase+'T']
        for i,T in enumerate(dur):
            runs.append(check(row['id'],phase,i,T,np.linalg.norm(path[i+1]-path[i]),row['yaw'] if phase=='loaded' and row['id']=='middle' and i==1 else 0))
(p/'route-power.json').write_text(json.dumps(dict(upperRunningKg=m,module=module,trials=runs,assumptions='Weak pack 11.8 V open, 27 mΩ. 39/24 A concurrent loaded/empty, 82% gear efficiency, 95% controller, 0.025 rolling resistance, yaw inertia 10 kg m² at 0.345 m, mu 0.55.'),indent=2))
print(len(runs),min(v['minBusV'] for v in runs),min(v['voltageReserveV'] for v in runs),[v for v in runs if not v['passScreen']])

