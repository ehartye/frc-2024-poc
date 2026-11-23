"""Check the replacement service-index disk in its received shaft frame before accepting battery service."""
from pathlib import Path
import numpy as np
p=Path(__file__).resolve().parent
exec(compile((p/'service.py').read_text().split('out={}')[0],'installed stock','exec'))
disk=original['Service index disk'];pin=original['Battery service pin'];bb=np.array(bounds(pin))
center=(bb[:3]+bb[3:])/2
# Transfer the actual truck-pin center back through the selected −40 degree rotator pose, then forward to saved park.
t=mat(-40,185,0,501)@ip;insert=offset(0,-12);peg=place(pin,insert)
axis=np.linalg.inv(t)@np.r_[center,1.];drill=cyl((axis[0],225.5,axis[2]),Y,3.35,12);new=cut(disk,drill)
def sample(angle):
    tf=mat(angle,185,0,501)@ip
    return dict(oldCommonMm3=common(place(disk,tf),peg),newCommonMm3=common(place(new,tf),peg),radialGapMm=distance(place(new,tf),peg))
poses={a:sample(a) for a in [-44,-40,-36,80]}
size=math.pi*3.35**2*11
assert abs((vol(disk)-vol(new))-size)<.02*size and poses[-40]['oldCommonMm3']>200 and poses[-40]['newCommonMm3']<.001 and min(poses[a]['newCommonMm3'] for a in [-44,-36])>50
G.clear();G.update(original);G.update({k:place(v,park) for k,v in parts.items()});G['Service index disk']=new
write_step(p/'service-index-guide.step')
reopened={name(k):v for k,v in load_solids(p/'service-index-guide.step')[0].items()}
assert len(reopened)==232 and abs(vol(reopened['Service index disk'])-vol(new))<.01
newMesh=viewmesh(reopened['Service index disk']);save(p/'service-index-mesh.json',newMesh)
out=dict(parent=digest(p/'inboard-guide.step'),whole=digest(p/'service-index-guide.step'),solids=len(reopened),cutMm3=vol(disk)-vol(new),releasedKg=(vol(disk)-vol(new))*7850e-9,nominalInsertedMm=10,poses=poses,axis=list(axis[:3]),scope='Straight 370 mm disconnected battery sweep clears by 5 mm at the −40 degree service index. Manual slide restraint, pin strength, operator handling and disconnected harness still require proof. Matched angle shifts use the real 6.35 mm pin.')
save(p/'service-pin.json',out);print(out)
