"""Replay the measured ramp and charge the gross frame-mounted slide package during the free hang."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import tempfile, json, sys, subprocess
p=Path(__file__).resolve().parent;root=p.parent;hoist=root/'guided-hoist-01'
if len(sys.argv)==1:
    def call(args):
        name,_,_=args
        with (p/(name+'.log')).open('w') as f:
            subprocess.run([sys.executable,__file__,*args],stdout=f,stderr=f,check=True)
    with ThreadPoolExecutor(max_workers=3) as ex:list(ex.map(call,[('climb-control','0.04','0'),('climb-gross-front','0.04','1'),('climb-gross-rear','-0.04','1')]))
    source=json.loads((hoist/'return-front.json').read_bytes());control=json.loads((p/'climb-control.json').read_bytes())
    assert control['trace']==source['trace']
    summary={}
    for name in ['climb-control','climb-gross-front','climb-gross-rear']:
        r=json.loads((p/(name+'.json')).read_bytes())
        summary[name]={k:r[k] for k in ['modeledRobotMass','chargedMass','disabledMinSideN','captured','latched','weakBusMin','maxCombinedCableN','peakPitchLockNm','disabledMinTireOrBumperM','maxDisabledFloorN','neighborN']}
    (p/'climb-gross-result.json').write_text(json.dumps(summary,indent=2))
    print(summary)
else:
    label,cg,charge=sys.argv[1:]
    mesh=json.loads((root/'mounted-slide-02/display-mesh.json').read_bytes())
    stock={n:[list(map(min,zip(*v['v']))),list(map(max,zip(*v['v'])))] for n,v in mesh.items()}
    stock['Slide drive motor']=[[244.4,238,391],[301,312,465]]
    mass=json.loads((root/'mounted-slide-02/gross-power.json').read_bytes())['extraFrameKg'] if charge=='1' else 0
    insert="""if chargeMass:
    extra=ET.SubElement(frame,'body',name='serviceMass',pos='.270 .275 .428')
    ET.SubElement(extra,'inertial',pos='0 0 0',mass=str(chargeMass),diaginertia='.0004 .0004 .0004')
    for i,(name,(lo,hi)) in enumerate(stockBounds.items()):
        # AABB occupied envelopes of the added hardware on the frame can collide with the chain.
        center=(np.array(lo)+hi)*.0005; half=(np.array(hi)-lo)*.0005
        ET.SubElement(frame,'geom',type='box',name='serviceStock'+str(i),pos=' '.join(map(str,center)),size=' '.join(map(str,half)),contype='8',conaffinity='4',mass='0')
"""
    src=(hoist/'line-return.py').read_text()
    patch=(hoist/'winch.py').read_text()
    native=(root/'endgame-route-01/guided-equipped.py').read_text().replace('m=mj.MjModel.from_xml_string(ET.tostring(tree',insert+'m=mj.MjModel.from_xml_string(ET.tostring(tree')
    native=native.replace("out=dict(strokeM=stroke","out=dict(strokeM=stroke",1)
    native=native.replace("(p/(tag+'.json')).write_text","out.update(chargedMass=chargeMass,modeledRobotMass=float(m.body_subtreemass[m.body('frame').id]));(outFolder/(tag+'.json')).write_text")
    patch=patch.replace("src=(srcdir/'guided-equipped.py').read_text()","src=baseNative")
    src=src.replace("code=(p/'winch.py').read_text(encoding='utf8')","code=baseWinch")
    sys.argv=[str(hoist/'line-return.py'),label,cg,'normal','6.5']
    exec(compile(src,'gross slide service sensitivity','exec'),dict(__file__=str(hoist/'line-return.py'),baseWinch=patch,baseNative=native,chargeMass=mass,stockBounds=stock,outFolder=p))

