from pathlib import Path
import json,sys,numpy as np
p=Path(__file__).resolve().parent
sys.path.insert(0,str(p.parent))
from layout import *
from showcase_contacts import load_solids
src=p.parent/'catch-layout-03/catch-install-trial.step'; stocks,_=load_solids(src)
names={n.replace(chr(194),''):sh for n,sh in stocks.items()}
park=mat(125,185,0,501);pickup=mat(-50,169.3625892825837,0,501)
toPick=pickup@np.linalg.inv(park)
if __name__=='__main__':
    for n,sh in names.items():
        if 'extension' in n.lower() or 'nose' in n.lower():
            print(n,np.round(bounds(place(sh,toPick)),1))
    for x in [596,600,603,606]:
        for r in [12,15,18]:
            z=r-2.6
            gap=cyl((x,226,z),Y,4.5,4)
            area=common(gap,place(names['Cassette side beam extension · left'],toPick))/(4*np.pi*4.5**2)
            print(x,r,round(area,3))

