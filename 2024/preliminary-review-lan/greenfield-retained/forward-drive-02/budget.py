from pathlib import Path
import json,numpy as np,math,hashlib
p=Path(__file__).resolve().parent;root=p.parent
frame=json.loads((root/'frame-pocket-01/result.json').read_bytes());fit=json.loads((p/'result.json').read_bytes())
inc=sum(v*(7.85e-6 if ('shaft' in k or 'axle' in k) else 2.70e-6) for k,v in fit['netVolumeMm3'].items())
extra=[inc+.015,1.15*inc+.05]
inspection=np.array(frame['combinedInspectionKg'])+extra
old=json.loads((root/'follower-01/review-data.json').read_bytes());running=inspection+np.array(old['running'])-np.array(old['inspection'])
# One sill takes the entire applied effective hanging force and pitch couple; others can redistribute.
# This is a signed, isolated member screen with simple end supports, not a joint or complete frame test.
xs=np.linspace(-322,322,1289);feet=np.array([-125.5,269.5])
cuts=[(a,b) for n,a,b in frame['cuts'] if n=='Chassis sill · left']
pocket=np.zeros_like(xs,dtype=bool)
for a,b in cuts:pocket|=(xs>=a)&(xs<=b)
B,H=24.,30.;t=3.;h=23.;I=(B*H**3-np.where(pocket,(B-t)*h**3,0))/12
Q=(B*H**2-np.where(pocket,(B-t)*h**2,0))/8
studies=[]
grav=running[1]*9.80665
for factor in (1.,2.):
    W=factor*grav
    for cg in [-40,40]:
        for couple in [-215,215]:
            force=(W*(cg-feet[0])+1000*couple)/(feet[1]-feet[0]);f=np.array([W-force,force])
            right=sum(f*(feet+322))/644;left=W-right
            M=left*(xs+322)-np.maximum(xs[:,None]-feet,0)@f
            shear=left-(xs[:,None]>=feet)@f
            stress=M*15/I;webTau=shear*Q/(I*np.where(pocket,t,B))
            rot=np.r_[0,np.cumsum((M[:-1]/I[:-1]+M[1:]/I[1:])*.25/69000)]
            dz=np.r_[0,np.cumsum((rot[:-1]+rot[1:])*.25)]
            dz-=dz[-1]*(xs+322)/644
            studies.append(dict(effectiveG=factor,cgXmm=cg,pitchMomentNm=couple,footForcesN=f.tolist(),endReactionsN=[left,right],maxBendingMPa=float(max(abs(stress))),maxWebShearMPa=float(max(abs(webTau))),maxVerticalDeflectionMm=float(max(abs(dz)))))
out=dict(parent=fit['step'],chassisRange=frame['newChassisKg'],machinedChassisKg=frame['newFrameSolidKg'],motorBracketAndShaftDeltaKg=inc,gripAndWireIncrementKg=extra,inspection=inspection.tolist(),running=running.tolist(),highMarginKg=float(old['limits']['inspectionKg']-inspection[1]),sillStudies=studies,scope='All five machined chassis members plus estimated hardware, purchased parts and wiring. Sill screen assumes simple supports at both frame end faces, 69 GPa, all vertical load and pitch moment on one sill. End fixity, fatigue, joint slip, chassis torsion and compliance are open.')
(p/'budget.json').write_text(json.dumps(out,indent=2));print(inspection,out['highMarginKg'],max(v['maxBendingMPa'] for v in studies),max(v['maxVerticalDeflectionMm'] for v in studies))

