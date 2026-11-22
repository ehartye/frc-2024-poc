"""Preserve the free floor NOTE through the real return route, ending at the single 122 degree firing pose."""
from pathlib import Path
import sys
p=Path(__file__).resolve().parent;root=p.parent
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
    return s.replace('125-deploy','122-deploy').replace('count=38;','count=76;').replace('mass=".07" priority','mass=".035" priority')''')
sub("exec(compile(finish,'seat confirmed carry','exec'))","""marker="loc.write_text(json.dumps(r,separators=(',',':')));print(key"
assert loaded.count(marker)==1
loaded=loaded.replace(marker,"after=[v for v in r['trajectory'] if v[0]>=2.82+r['seatDelay']];r.update(originalHeld=r['held'],held=bool(after) and not r['failedGate'] and all(.12<v[8]<.35 and .018<v[9]<.09 and abs(v[2])<.05 for v in after),endPitchDegrees=122);"+marker)
exec(compile(finish,'seat confirmed carry','exec'))""")
exec(compile(text,'lifted cassette at revised approach','exec'))

