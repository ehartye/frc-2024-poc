"""Preserve the free floor NOTE through the real return route, ending at the single 122 degree firing pose."""
from pathlib import Path
import sys
p=Path(__file__).resolve().parent;root=p.parent
sys.path[:0]=[str(root/'follower-01'),str(root/'loaded-actuation-01')]
from smooth import times,table
text=(root/'seat-interlock-01/recovery.py').read_text()
def sub(old,new):
    global text
    assert text.count(old)==1,old
    text=text.replace(old,new)
sub("assert not plan.exists() or plan.read_bytes()==(ref/'plan.json').read_bytes()","assert plan.exists()")
sub("if not plan.exists():plan.write_bytes((ref/'plan.json').read_bytes())","assert all(r['loaded'][-1]==[1.39,5.871] for r in json.loads(plan.read_bytes())['plan'])")
sub('(local[:,0]-.055)','(local[:,0]-.060)');sub('abs(local[:,1])-.050','abs(local[:,1])-.225')
sub('return interlock(original_split(source))','''s=interlock(original_split(source))
    assert s.count('125-deploy')==2 and s.count('count=38;')==1 and s.count('mass=".07" priority')==1
    s=s.replace('(t-startLift-delay)/3.4','(t-startLift-delay)/4.8').replace('rate=6*u*(1-u)/3.4','rate=6*u*(1-u)/4.8')
    assert '(t-startLift-delay)/4.8' in s and 'rate=6*u*(1-u)/4.8' in s
    marker="        tj=jid('truck');"
    assert s.count(marker)==1 and s.count('    for step in range')==1
    s=s.replace('    for step in range',"    tabX,tabV=table(deploy,122,g['pickPivot'][0])"+chr(10)+'    for step in range')
    calc="        profileTime=(t-startLift-delay)*3.4/4.8;X=float(np.interp(profileTime,times,tabX));delta=1000*(X-robotX);vx=float(np.interp(profileTime,times,tabV))*3.4/4.8"
    s=s.replace(marker,calc+chr(10)+marker)
    return s.replace('125-deploy','122-deploy').replace('count=38;','count=76;').replace('mass=".07" priority','mass=".035" priority')''')
sub("exec(compile(finish,'seat confirmed carry','exec'))","""marker="loc.write_text(json.dumps(r,separators=(',',':')));print(key"
assert loaded.count(marker)==1
assert loaded.count('startLift+3.4')==1
loaded=loaded.replace('startLift+3.4','startLift+4.8')
loaded=loaded.replace(marker,"after=[v for v in r['trajectory'] if v[0]>=2.82+r['seatDelay']];r.update(originalHeld=r['held'],held=bool(after) and not r['failedGate'] and all(.12<v[8]<.35 and .018<v[9]<.09 and abs(v[2])<.05 for v in after),endPitchDegrees=122,liftDurationS=4.8,slideSmoothingS=.16*4.8/3.4);"+marker)
exec(compile(finish,'seat confirmed carry','exec'))""")
exec(compile(text,'lifted cassette at revised approach','exec'))

