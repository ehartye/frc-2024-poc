"""Early packaging screen for a stationary-pitch cassette with the hook group on a retracting slide."""
from pathlib import Path
import json
p=Path(__file__).resolve().parent
src=(p/'compare-hangs.py').read_text()
src=src.replace("    if not ('Tag14'"+src.split("    if not ('Tag14'",1)[1].split(":continue",1)[0]+":continue","")
exec(compile(src.split('def trial(')[0],'original field and robot members','exec'))
r=read(root/'chain-contact-02/front48.json')['trace'][0]
tr=transforms(r,0,0)
moving={n for n in bodyNames if n.startswith(('Chain hook','Keeper drive','Keeper pinion','Keeper return','Keeper motor','Common keeper'))}
assert len(moving)==7
# Positive slide retracts the hook carrier along the cassette longitudinal axis.
# To keep the chain attachment in place, the ideal chassis follows the opposite displacement.
axis=np.array([math.cos(math.radians(r['angle'])),0, math.sin(math.radians(r['angle']))])
parked={n:place(stock[n],tr[scene['rig'][n]]) for n in bodyNames}
geom={n:verts[n]@tr[scene['rig'][n]][:3,:3].T+tr[scene['rig'][n]][:3,3] for n in bodyNames}
shapeBounds={n:np.array([v.min(axis=0)-.3,v.max(axis=0)+.3]) for n,v in geom.items()}
def touch(bb,bc):return np.all(bb[1]>=bc[0]) and np.all(bb[0]<=bc[1])
def moved(n,xyz):
    t=np.eye(4);t[:3,3]=xyz
    return place(parked[n],t)
conflicts=[];fieldCounts=[];relative=[]
for ds in (0,25,50,75,100):
    delta=ds*axis
    rivals=0
    for n in bodyNames:
        dx=np.zeros(3) if n in moving else delta
        bb=shapeBounds[n]+dx
        for label,s,b in catalog:
            if not touch(bb,b):continue
            rivals+=1;ob=moved(n,dx);g=distance(s,ob)
            if g<.001:
                v=common(s,ob)
                if v>.1:conflicts.append(dict(stroke=ds,article=n,fixed=label,volume=v))
    fieldCounts.append(rivals)
    for n in moving:
        for k in set(bodyNames)-moving:
            if not touch(shapeBounds[n]-delta,shapeBounds[k]):continue
            ob=moved(n,-delta);g=distance(ob,parked[k])
            if g<.001:
                v=common(ob,parked[k])
                if v>.1:relative.append(dict(stroke=ds,article=n,fixed=k,volume=v))
    out=dict(source=read(p/'access.json')['layout'],field=registry.get('source'),installed=len(parked),staticFieldLeaves=len(catalog),hookGroup=sorted(moving),strokeMm=ds,idealLiftMm=delta[2],fieldCandidateCounts=fieldCounts,fieldContacts=conflicts,internalContacts=relative,scope='Ideal constant-pitch motion, at five stroke samples. The frame translates along the cassette axis and hook seats remain fixed. Any new stock intersection vetoes the unmodified slide layout. Capture dynamics, actual CG tilt, rail/slide design, mass and motor remain open.')
    (p/'guided-hooks.json').write_text(json.dumps(out,indent=2))
print(len(conflicts),len(relative),fieldCounts)
