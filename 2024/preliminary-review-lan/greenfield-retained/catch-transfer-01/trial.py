"""Bounded preliminary pawl/sector load transfer; geometry and strength remain open."""
from pathlib import Path
import hashlib,json,sys
p=Path(__file__).resolve().parent
label,dt,cg,present,cap=sys.argv[1:];cap=float(cap);present=int(present)
source=p.parent/'mission-physics/climb.py';src=source.read_text()
def edit(a,b):
    global src
    assert src.count(a)==1
    src=src.replace(a,b)
edit('for i in range(int(9/dt)+1):','''armed=False;clearTime=0.;confirmedTime=0.;confirmAt=None;armAt=None;contactAt=None;cutAngle=None;drift=0.;maxFloor=0.;minFloor=100.;peakCatch=0.;maxMotor=0.;capTime=0.;preTorque=None
for i in range(int(9/dt)+1):''')
a=src.index('    if t>=cut and hold');b=src.index('    d.qfrc_applied',a)
src=src[:a]+'''    clearTime=clearTime+dt if angle>=np.deg2rad(160) and len(grasp)==2 else 0.
    if present and not armed and clearTime>=.040:
        armed=True;armAt=t
    # Test ramp-down from 3.1 s with the SAME forced cutoff in both enabled and jammed-open cases.
    # A failed seat proof would forbid elective motor disable, but cannot prevent loss of power.
    torque*=1.-.98*np.clip((t-3.10)/.45,0,1)
    sink=np.deg2rad(158)-angle
    reaction=float(np.clip(12000.*sink-18*d.qvel[vi],0,cap)) if armed and sink>0 else 0.
    if reaction>0 and contactAt is None:contactAt=t
    if reaction>=cap-1e-8 and armed:capTime+=dt
    peakCatch=max(peakCatch,reaction)
    proof=t<cut and reaction>=60 and abs(torque)<=8 and abs(d.qvel[vi])<np.deg2rad(3)
    confirmedTime=confirmedTime+dt if proof else 0.
    if confirmedTime>=.1 and confirmAt is None:confirmAt=t
    if t<cut:preTorque=reaction
    if t>=cut:
        if cutAngle is None:cutAngle=angle
        drift=max(drift,abs(np.rad2deg(angle-cutAngle)));maxMotor=max(maxMotor,abs(torque))
''' +src[b:]
edit('d.qfrc_applied[vi]=torque;','d.qfrc_applied[vi]=torque+reaction;')
edit('if i%round(.025/dt)==0:', 'if i%round(.025/dt)==0 or t>=cut:')
edit('        trace.append(dict','''        if t>=cut:minFloor=min(minFloor,zmin);maxFloor=max(maxFloor,ground)
        if i%round(.025/dt)==0:trace.append(dict''')
edit('hold=bool(d.eq_active[3])','hold=bool(reaction>0),catchNm=reaction,armed=armed,confirmed=confirmAt is not None')
edit("Path(sys.argv[1]+'.json').write_text", """r.update(sourceHash=sourceHash,capNm=cap,armAt=armAt,seatAt=contactAt,confirmAt=confirmAt,
    measuredDriftDeg=drift,maxFloorN=maxFloor,minFloorMm=minFloor*1000,maxMotorNmAfterCut=maxMotor,
    catchNmAtCut=preTorque,peakCatchNm=peakCatch,secondsAtLimit=capTime,
    passed=bool(len(grasp)==2 and confirmAt is not None and preTorque>=60 and maxFloor<.01 and minFloor>=.01 and drift<3 and maxMotor==0))
Path(sys.argv[1]+'.json').write_text""")
sys.argv=[__file__,str(p/label),dt,'1','0',cg];sourceHash=hashlib.sha256(source.read_bytes()).hexdigest()
exec(compile(src,'unilateral stop experiment','exec'))

