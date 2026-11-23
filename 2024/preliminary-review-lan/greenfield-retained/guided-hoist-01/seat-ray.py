"""Test the proposed acquisition sensor in occupied cassette coordinates using ray and sensor-body envelopes."""
from pathlib import Path
p=Path(__file__).resolve().parent
src=(p/'inboard.py').read_text(encoding='utf8');exec(compile(src.split('partsAll={')[0],'installed hoist geometry','exec'))
stock={**members,**parts};bb={n:np.array(bounds(s)) for n,s in stock.items()}
out={}
for x,z in [(55,50.4),(55,66),(55,75),(60,50.4)]:
    checks={}
    for n,v in [('old ray',cyl((x,-50,z),Y,.5,100)),('transverse ray',cyl((x,-225,z),Y,.5,450)),('left sensor',box(x-3,225,z-4,6,12,8)),('right sensor',box(x-3,-237,z-4,6,12,8))]:
        a=np.array(bounds(v));hits={}
        for k,s in stock.items():
            if candidate(a,bb[k]):
                h=common(v,s)
                if h>.005:hits[k]=h
        checks[n]=hits
    out[str([x,z])]=checks
save(p/'seat-ray.json',out);print(out)
