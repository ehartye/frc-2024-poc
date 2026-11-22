"""Test the changed stock against chassis, service poses and the actual pitch sweep from floor intake to scoring."""
from pathlib import Path
p=Path(__file__).resolve().parent
src=(p/'stop-slot.py').read_text(encoding='utf8');exec(compile(src.split('partsAll=')[0],'corrected cable and clutch parts','exec'))
from stroke import scheduled
curves={};invalid=[];nearPairs=0;testStates=0
targetRig=active['rig'];neutral={k:parts.get(k,members.get(k)) for k in {*parts,*members}}
originalOnly={k:s for k,s in original.items() if k in active['rig'] and k not in active['removed'] and k not in neutral and k!='Chambered NOTE'}
from sweep import place as fast
def B(s):return np.array(bounds(s))
def shiftB(b,t):
    c=(b[:3]+b[3:])/2;h=(b[3:]-b[:3])/2;pos=t[:3,:3]@c+t[:3,3];half=abs(t[:3,:3])@h
    return np.r_[pos-half,pos+half]
fixedB={k:B(s) for k,s in originalOnly.items()};localB={k:B(s) for k,s in parts.items()};unique=set();pitchStop=[]
def inspect(angle,stroke=0,dep=0,release=0,dogDepth=0,shaftDeg=0):
    global nearPairs,testStates
    testStates+=1;arm=mat(angle,scheduled(angle),0,501);slide=offset(scheduled(angle)-185)
    fixed={}
    for n,s in originalOnly.items():
        t=slide if targetRig[n]=='slide' else np.eye(4)
        if n=='Catch dog':t=t@offset(0,-dogDepth)
        fixed[n]=(s,t,shiftB(fixedB[n],t))
    for n,s in parts.items():
        v=role[n];t=arm
        if v=='carriage':t=t@offset(-stroke)
        elif v=='clutch':t=t@offset(0,-release)@mat(shaftDeg,0,0,0)
        elif v=='drum':t=t@mat(shaftDeg,0,0,0)
        elif v=='stop':t=t@offset(0,-dep if n.endswith('left') else dep)
        elif v=='cord':s=placedAt(n,stroke,dep,release)
        b=shiftB(B(s) if v=='cord' else localB[n],t)
        for other,(stationary,tf,bb) in fixed.items():
            if not candidate(b,bb):continue
            rel=np.linalg.inv(tf)@t;key=(n,other,tuple(rel[:3].round(5).flatten()),stroke if v=='cord' else 0)
            if key in unique:continue
            unique.add(key);nearPairs+=1;actual=fast(s,rel)
            volx=common(actual,stationary)
            if volx>.01:invalid.append([angle,stroke,dep,release,n,other,volx])
            if n=='Capture-angle dog bushing' and other=='Catch dog' and dogDepth>0:pitchStop.append([angle,dogDepth,distance(actual,stationary),volx])
for a in np.unique(np.r_[np.arange(-50,126,2),-49.7445989264,75.38642106234,91.29157,125.]):inspect(float(a))
for stroke in (0,55,110,165,220,230):
    inspect(75.38642106234,stroke,8 if stroke==230 else 0,4.2,10.8,180*stroke/29.5/math.pi)
for a in [74.38642106234,76.38642106234]:inspect(a,0,0,4.2,10.8)
# Label intentional angle-shift controls separately.
production=[row for row in invalid if all(abs(row[0]-v)>1e-7 for v in (74.38642106234,76.38642106234))]
out=dict(parent=digest(source),package=digest(p/'stop-slot.step'),poses=testStates,comparisons=nearPairs,intersections=production,pitchStop=pitchStop,controlContacts=[row for row in invalid if row not in production],scope='Sampled tilt/slide and supplied six hoist states. The gate and rotating cassette internal comparisons and field service are separate.')
save(p/'whole-screen.json',out);print(len(production),nearPairs,[(a,b,c,d,n,f,round(v,2)) for a,b,c,d,n,f,v in production[:9]])

