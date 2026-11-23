from pathlib import Path
import json
p=Path(__file__).resolve().parent;reference=p.parent/'six-piece-02';layout=p.parent/'split-cycle-01'
script=(reference/'route.py').read_text().replace('1.53,5.55','1.54,5.55')
marker="vs={role:"
patch="""variant=read(layout/'package.json');mesh=read(layout/'meshes.json');design['step']=variant['layout']
for n in variant['removed']:
    design['bodies'].pop(n);cs['rig'].pop(n,None)
design['bodies'].update(mesh);cs['rig'].update({n:'arm' for n in mesh})
"""
assert script.count(marker)==1
exec(compile(script.replace(marker,patch+marker),'route on split solid package','exec'))
