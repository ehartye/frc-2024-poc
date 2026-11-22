"""Drive the same free NOTE until the rear seat beam is continuously blocked; cap additional time at 1.5 s."""
from pathlib import Path
import json,sys,math
p=Path(__file__).resolve().parent;ref=p.parent/'split-field-02';plan=p/'plan.json'
assert not plan.exists() or plan.read_bytes()==(ref/'plan.json').read_bytes()
if not plan.exists():plan.write_bytes((ref/'plan.json').read_bytes())
driver=(ref/'carry.py').read_text(encoding='utf8')
def interlock(src):
    def edit(a,b):
        nonlocal src
        assert src.count(a)==1,(a,src.count(a))
        src=src.replace(a,b)
    edit('    for step in range(round(endTime/dt)+1):','''    delay=0.;proof=0.;seatTime=None;firstRay=None;maxDepth=0.;failedGate=False
    for step in range(round((endTime+1.5)/dt)+2):''')
    edit('        u=max(0.,min(1.,(t-startLift)/3.4))','''        if t>endTime+delay+.000001:break
        if t>=2.82 and seatTime is None:
            delay=max(0.,t+dt-2.82)
            if delay>=1.5:
                failedGate=True
                break
        u=max(0.,min(1.,(t-startLift-delay)/3.4))''')
    edit('if t>=2.82:commanded=0.', 'if seatTime is not None and t>=2.82+delay:commanded=0.')
    edit('bodyPos,yaw,accField,om,al=pathAt(t)','bodyPos,yaw,accField,om,al=pathAt(t-delay)')
    edit('        # All velocities in the cassette frame', '''        # Segment crosses the rear of the seat, at a fixed axial and vertical position. The entire free ring is used.
        beam2=(local[:,0]-.055)**2+(local[:,2]-.0504)**2+np.maximum(abs(local[:,1])-.050,0.)**2
        depth=.0254-float(np.sqrt(beam2.min()));maxDepth=max(maxDepth,depth)
        lit=depth>.001 and not blind
        if lit and firstRay is None:firstRay=t
        proof=proof+dt if lit else 0.
        if t>=2.82 and proof>=.040 and seatTime is None:seatTime=t
        # All velocities in the cassette frame''')
    edit('return dict(endZoneSpeeds=',"return dict(seatDelay=delay,beamProofS=seatTime,beamFirst=firstRay,beamPeakDepthM=maxDepth,failedGate=failedGate,doneAt=t,endZoneSpeeds=")
    return src
blind=sys.argv.pop(4)=='blind'

exec(driver.split("exec(compile(src,",1)[0])
setup, finish=src.split("loaded=(sources/'loaded.py').read_text()",1)
exec(compile(setup,'original split drive','exec'))
original_split=split_intake
def split_intake(source):
    return interlock(original_split(source))
loaded=(sources/'loaded.py').read_text()
loaded=loaded.replace('startLift=startLift,finish=endTime',"startLift=startLift+r['seatDelay'],finish=r['doneAt']")
loaded=loaded.replace("held=bool(np.all","held=bool(not r['failedGate'] and np.all")
loaded=loaded.replace("sys.argv=[__file__,attempt+", "sys.argv=[__file__,('blind-' if blind else '')+attempt+")
exec(compile(finish,'seat confirmed carry','exec'))
