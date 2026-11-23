"""Replay the commissioning load cases with a bounded, power-loss-applied pivot brake."""
from pathlib import Path
import sys,json,numpy as np
p=Path(__file__).resolve().parent
label,dt,cg,cap,lag=sys.argv[1:];cap=float(cap);lag=float(lag)
src=(p.parent/'mission-physics/climb.py').read_text()
def edit(a,b):
    global src
    assert src.count(a)==1,a
    src=src.replace(a,b)
edit('axis="0 -1 0" damping=".5"', 'axis="0 -1 0" damping=".5" frictionloss=".000001"')
edit('for i in range(int(9/dt)+1):','''minimum=100.;maxGround=0.;drift=0.;peakBrake=0.;angleCut=None;maxMotorDisabled=0.
for i in range(int(9/dt)+1):
    phase=np.clip((d.time-cut-lag)/.03,0,1)
    m.dof_frictionloss[vi]=max(1e-6,cap*phase)
''')
a=src.index('    if t>=cut and hold');b=src.index('    d.qfrc_applied',a)
src=src[:a]+src[b:]
edit('    if i%round(.025/dt)==0:', '''    if i%round(.025/dt)==0 or t>=cut:''')
edit('        trace.append(dict','''        if t>=cut:
            if angleCut is None:angleCut=angle
            drift=max(drift,abs(np.rad2deg(angle-angleCut)))
            maxGround=max(maxGround,ground);minimum=min(minimum,zmin)
            maxMotorDisabled=max(maxMotorDisabled,abs(torque))
            ids=np.flatnonzero((d.efc_type==int(mj.mjtConstraint.mjCNSTR_FRICTION_DOF))&(d.efc_id==vi))
            if len(ids):peakBrake=max(peakBrake,float(np.max(np.abs(d.efc_force[ids]))))
        if i%round(.025/dt)==0:trace.append(dict''')
edit('hold=bool(d.eq_active[3])','hold=bool(phase>=1 and cap>0)')
edit("Path(sys.argv[1]+'.json').write_text", """r.update(method='finite spring-applied friction brake',capacityNm=cap,delayS=lag,rampS=.03,
    peakDisabledBrakeNm=peakBrake,peakDisabledMotorNm=maxMotorDisabled,driftDeg=drift,
    maxFloorAfterCutN=maxGround,minFloorAfterCutM=minimum,
    passScreen=bool(len(grasp)==2 and maxGround<.01 and minimum>=.010 and drift<3 and maxMotorDisabled==0))
Path(sys.argv[1]+'.json').write_text""")
sys.argv=[__file__,str(p/label),dt,'1','0',cg]
exec(compile(src,'bounded climb','exec'))

