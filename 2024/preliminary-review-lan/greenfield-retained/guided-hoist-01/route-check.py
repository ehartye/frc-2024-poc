from pathlib import Path
p=Path(__file__).resolve().parent;root=p.parent
s=(root/'tread-underlay-01/recheck-route.py').read_text()
idx='cs=dict(cs,rig=rig);design'
s=s.replace(idx,"changed=read(p/'mesh.json');variant=dict(variant,add={**variant['add'],**changed},step=read(p/'exchange.json')['whole']);rig.update({n:'arm' for n in changed});"+idx)
exec(compile(s,'rerun original route with revised stock','exec'))
