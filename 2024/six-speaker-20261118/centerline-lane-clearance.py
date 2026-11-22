import sys
from pathlib import Path
import numpy as np
import json
sys.path.insert(0,str(Path.cwd()/'sim/pressure'))
from field_route_clearance import SURFACES,GAME_PIECES
base=Path(__file__).parent
parts=json.loads((base/'field-source/reproduced-installed-field.json').read_bytes())['parts']
ob=np.asarray([x['bounds'] for x in parts if x['path'][1] not in SURFACES|GAME_PIECES and x['bounds'][0][2]<.9 and x['bounds'][1][2]>.03])
for y in np.arange(7.7,3.2,-.15):
    pts=np.c_[np.arange(1.4,8.55,.15),np.full_like(np.arange(1.4,8.55,.15),y)]
    gap=np.maximum(np.maximum(ob[None,:,0,:2]-pts[:,None,:],pts[:,None,:]-ob[None,:,1,:2]),0)
    clear=np.linalg.norm(gap,axis=2).min(axis=1)
    print(f'{y:.2f}', ''.join(' ' if n>.75 else '.' if n>.56 else '#' for n in clear))
