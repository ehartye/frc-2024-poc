"""Close the unmodeled chain/battery and slide-motor contacts on the same moving chassis."""
from pathlib import Path
import sys,math,json
p=Path(__file__).resolve().parent;base=p.parent/'chain-contact-02'
label,ramp,cg,mode=sys.argv[1:5];ramp=float(ramp)
def augment(src):
    def guard(a,b):
        nonlocal src
        assert src.count(a)==1,(a,src.count(a))
        src=src.replace(a,b)
    # Control the actual extended source after hook geometry, free body and joint logic have been installed.
    guard('(t-.55)/2.4',f'(t-.55)/{ramp}')
    tag='<body name="cassette"'
    guard(tag,'<geom name="batteryCase" type="box" pos="-.0625 -.180 .2375" size=".0925 .040 .0865" contype="8" conaffinity="4" solref=".003 1" mass="0"/><geom name="slideMotor" type="cylinder" fromto="-.102 .275 .428 -.049 .275 .428" size=".014" contype="8" conaffinity="4" solref=".003 1" mass="0"/>'+tag)
    guard('chainids=[',"neighborPeak={'batteryCase':0.,'slideMotor':0.};neighborImpulse=dict.fromkeys(neighborPeak,0.);firstHit={};\nchainids=[")
    guard('    u=np.clip','''    for contactId in range(d.ncon):
        ct=d.contact[contactId]
        nm=[m.geom(gg).name or '' for gg in (ct.geom1,ct.geom2)]
        for key in neighborPeak.keys() & set(nm):
            cf=np.zeros(6);mj.mj_contactForce(m,d,contactId,cf)
            neighborPeak[key]=max(neighborPeak[key],float(cf[0]))
            neighborImpulse[key]+=float(cf[0])*dt
            if cf[0]>.01:firstHit.setdefault(key,t)
    u=np.clip''')
    guard('r.update(physicalHookNormalPeakN=' if 'r.update(physicalHookNormalPeakN=' in src else 'r.update(totalExplicitMassKg=',"r.update(hoistRampS="+str(ramp)+",neighborPeakN=neighborPeak,neighborImpulseNs=neighborImpulse,firstNeighborContactS=firstHit);\n"+('r.update(physicalHookNormalPeakN=' if 'r.update(physicalHookNormalPeakN=' in src else 'r.update(totalExplicitMassKg='))
    return src
s=(base/'common_gate.py').read_text(encoding='utf8')
s=s.replace("exec(compile(source,'shared rack and spring','exec'))","source=source.replace('src=convert(src);','src=augment(convert(src));');exec(compile(source,'shared rack and spring','exec'))")
sys.argv=[str(base/'common_gate.py'),str(p/label),'48','.00025',cg,mode]
exec(compile(s,'source-bound keeper dynamics with actual neighbors','exec'),dict(__file__=str(base/'common_gate.py'),__name__='__main__',augment=augment))

