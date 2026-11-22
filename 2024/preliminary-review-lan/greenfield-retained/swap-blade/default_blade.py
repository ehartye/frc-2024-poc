"""Select the owner's default blade and bound pitch error to first flat-floor contact."""
from pathlib import Path
import hashlib,json,math
p=Path(__file__).resolve().parent
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def select(d):
    j=json.loads((p.parent/'pickup-lip/layout.json').read_bytes())
    x0,_,z0=j['pickPivot']
    contour=j['profiles']['Replaceable pickup lip']
    def floor_height(extra):
        a=math.radians(j['pickAngle']-extra)
        return min(z0+x*math.sin(a)+z*math.cos(a) for x,z in contour)
    lo,hi=0.,1.
    assert floor_height(lo)>0 and floor_height(hi)<0
    for _ in range(55):
        mid=(lo+hi)/2
        if floor_height(mid)>0:lo=mid
        else:hi=mid
    decision=dict(default='A',label='Blunt UHMW',geometry='A-blunt.step',sha256=sha(p/'A-blunt.step'),
        spare='B-short-raised.step',assembly=d['step'],nominalFloorMm=floor_height(0),
        floorAtHalfDegreeDownMm=floor_height(.5),additionalNoseDownDegreesToTouch=hi,
        scope='Unloaded rigid edge, level floor, deployed pivot height held fixed; physical carpet and loaded deflection unmeasured.')
    (p/'blade-selection.json').write_text(json.dumps(decision,indent=2))
    d['defaultPickupBlade']=decision
    d['issue']='Shared cassette · Profile A pickup lip'
    d['focus']='Profile A is the default. Its 3 mm tip clearance is the first floor test. The power-off climb and weak-battery shot remain under review.'
    d['bodies']['Replaceable pickup lip']['rationale']='Default Profile A: blunt UHMW edge on a four-screw replaceable seat. The lower-grip simulation seated the NOTE. Profile B is a fit-checked spare for testing on the same mount.'
    d['notes']=[f'Profile A is the default. At fixed pivot height, {hi:.3f}° of extra nose-down pitch uses the nominal {floor_height(0):.1f} mm floor clearance. Measure loaded pitch, wear and carpet variation on the physical nose.' if x.startswith('Blade trial A is installed.') else x for x in d['notes']]
if __name__=='__main__':
    path=p/'review-data.json'; live=p.parent.parent/'www/data.json'
    assert path.read_bytes()==live.read_bytes()
    d=json.loads(path.read_bytes())
    assert sha(p/'shared-cassette-swappable.step')==d['step']
    select(d)
    payload=json.dumps(d,separators=(',',':')).encode()
    for target in [path,live]:
        tmp=target.with_suffix('.next');tmp.write_bytes(payload);tmp.replace(target)

