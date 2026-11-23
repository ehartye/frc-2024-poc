from pathlib import Path
import json
p=Path(__file__).resolve().parent
r=json.loads((p/'layout.json').read_bytes())
src=(p.parent/'mission-physics/shot.py').read_text(encoding='utf8')
src=src.replace('.3995 0 .084','.4575 0 .096').replace('box(.4255,0,.084','box(.4835,0,.096')
src=src.replace("    bead=''",'''    stock+='<geom name="flapAxle" type="cylinder" pos=".4575 0 .096" size=".002 .226" euler="90 0 0" contype="2" conaffinity="1"/>'
    assets=''
    for fieldname,half,key in [('Replaceable pickup lip',.217,'lip'),('Pickup lip crossbar',.226,'lipback')]:
        pts=r['profiles'][fieldname];verts=' '.join(str(v) for y in (-half,half) for x,z in pts for v in (x/1000,y,z/1000))
        assets+=f'<mesh name="{key}" vertex="{verts}"/>'
        stock+=f'<geom name="{key}" mesh="{key}" type="mesh" contype="2" conaffinity="1"/>'
    bead=\'\' ''').replace('<worldbody><body pos=', '<asset>{assets}</asset><worldbody><body pos=')
src=src.replace('    belts=np.zeros(2);','    changedContacts=set();\n    belts=np.zeros(2);').replace('mj.mj_step1(model,d);q=d.qpos.copy();',"""mj.mj_step1(model,d)
        for ct in d.contact:
            for gg in (ct.geom1,ct.geom2):
                nm=model.geom(gg).name
                if nm in ('lip','lipback','camOpenedFlap','flapBarrel','flapAxle'):changedContacts.add(nm)
        q=d.qpos.copy();""").replace('result=dict(name=name','result=dict(changedContacts=sorted(changedContacts),name=name')
exec(compile(src,'p5-shot','exec'))

