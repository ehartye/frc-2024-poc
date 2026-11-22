"""One guided climb with the gross slide package charged, at a stated centre-of-gravity offset, solver step,
hoist ramp and catch mode. The trial body is counted-reset-power-02/loaded_climb.py; only the four inputs are
read from the command line: LABEL CG_M DT_S RAMP_S MODE (released: 0.04 or -0.04, .00025, 6.5, normal)."""
from pathlib import Path
import json, sys
p=Path(__file__).resolve().parent;root=p.parent;hoist=root/'guided-hoist-01'
label,cg,DT,RAMP,MODE=sys.argv[1:];charge='1'
if True:
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
    assert native.count("'48','.00025',cg")==1
    native=native.replace("'48','.00025',cg","'48',"+repr(DT)+",cg")
    native=native.replace("(p/(tag+'.json')).write_text","out.update(chargedMass=chargeMass,modeledRobotMass=float(m.body_subtreemass[m.body('frame').id]));(outFolder/(tag+'.json')).write_text")
    patch=patch.replace("src=(srcdir/'guided-equipped.py').read_text()","src=baseNative")
    src=src.replace("code=(p/'winch.py').read_text(encoding='utf8')","code=baseWinch")
    sys.argv=[str(hoist/'line-return.py'),label,cg,MODE,RAMP]
    exec(compile(src,'gross slide service sensitivity','exec'),dict(__file__=str(hoist/'line-return.py'),baseWinch=patch,baseNative=native,chargeMass=mass,stockBounds=stock,outFolder=p))

