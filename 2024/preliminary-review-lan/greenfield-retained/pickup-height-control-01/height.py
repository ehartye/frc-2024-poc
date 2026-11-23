"""Find the usable Profile A tip-height interval on the finite-motor pickup model, with a floor-referenced setup."""
from pathlib import Path
import sys,json,math
p=Path(__file__).resolve().parent;src=p.parent/'bounded-risk-01/pickup.py'
label,n,target,error,height,y,mu,amp=sys.argv[1:];target,error,height=map(float,(target,error,height))
g=json.loads((p.parent/'pickup-lip/layout.json').read_bytes());prof=g['profiles']['Replaceable pickup lip']
def tip(a):return g['pickPivot'][2]+height+min(x*math.sin(math.radians(a))+z*math.cos(math.radians(a)) for x,z in prof)
lo=-55.;hi=-45.
for _ in range(50):
    mid=(lo+hi)/2
    if tip(mid)<target+error:lo=mid
    else:hi=mid
sys.argv=[__file__,label,n,str(hi-g['pickAngle']),'0',str(height),y,mu,amp]
exec(compile(src.read_text(),'height-referenced free pickup','exec'))

