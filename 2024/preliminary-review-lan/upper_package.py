"""Bring in root-frame preliminary support proposals with explicit independent moving membership."""
from pathlib import Path
import json,math
from OCP.gp import gp_Trsf,gp_Ax2,gp_Pnt,gp_Dir
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
def products(review,track,add,load,sha,found,info):
    qa=track.parents[2];ledger=json.loads((review.parent/'candidate-occupancy.json').read_bytes())['sources']
    q=qa/'powered-upper-50cf55f3'
    reflection=gp_Trsf();reflection.SetMirror(gp_Ax2(gp_Pnt(0,0,0),gp_Dir(0,1,0)))
    rosters=[[],[]]; bodyrefs={}; projection={}
    def take(label,s,moving=False,material='Metal assembly (planning)',right=True):
        for side in range(2 if right else 1):
            v=BRepBuilderAPI_Transform(s,reflection,True).Shape() if side else s
            ex=TopExp_Explorer(v,TopAbs_SOLID);assert ex.More();v=ex.Current();ex.Next();assert not ex.More()
            n='Upper '+('right' if side else 'left')+' · '+label
            add(n,v,'mechanism' if moving else 'structure','Upper '+('right' if side else 'left')+' scorer',material,
                ('Opposite-hand preliminary package from the measured left study. ' if side else 'Robot-frame preliminary mounting study. ')+('Moves with the independent wheel slide.' if moving else 'Stationary part of the load path.'))
            if moving:rosters[side].append(n)
    def registered(p):
        if str(p) in ledger:assert sha(p)==ledger[str(p)]
        return load(p)[0]
    for name,v in registered(q/'01-station-frame.step').items():
        if name.startswith('rail ') and 'cut length' in name or name=='shop-made adjustable spring seat M12x1':take(name,v)
    for name,v in registered(q/'02-four-block-carriage.step').items():
        if name.startswith('one piece receiver') or name.startswith('block ') and name.endswith('[1.1.1]'):take(name,v,True)
    important={'integral outer wheel bearing carrier','formed motor yoke','removable inner cheek','inner REV-21-1916','outer REV-21-1916','smooth output shaft successor','SJC-25C-8 output hub','SJC-25C-14 motor hub','SJC-25 BL sleeve spider','shaft end keeper'}
    for name,v in registered(q/'03-detachable-powered-trial.step').items():
        if name in important:take(name,v,True)
    stops=registered(qa/'upper-stop-layout-50cf55f3'/'backing-relieved.step')
    for n,v in stops.items():
        if n=='cartridge pockets integral with existing guide backing' or n.endswith('nose'):take(n,v)
    spring=registered(qa/'powered-upper-station-20261121'/'spring-seat-nominal-0.step')['spring dimensional envelope']
    take('spring occupied envelope at home',spring,False,'Spring steel · home envelope')
    for side in range(2):
        rosters[side]+= ['Wheel station '+str(2+side),'Motor station '+str(2+side)]
    right=json.loads((review.parents[1]/'output-key/right-web-seat.json').read_bytes())['trial'];rightpath=Path(right['file']);assert sha(rightpath)==right['sha256']
    for n,v in load(rightpath)[0].items():
        if n in ('cut right web','integral outer wheel bearing carrier','formed motor yoke','removable inner cheek','turned and keyed shaft'):
            add('Fixed right · '+n,v,'structure' if 'shaft' not in n else 'mechanism','Fixed right scorer','Machined metal (planning)','Placed opposite-hand carrier and web study. Preliminary fit, load verification open.')
    # Four wheel profiles use a clearance bore around the 1/2-inch hex envelope.
    for i in range(4):
        n='Wheel station '+str(i);v=found[n]
        from mechanical_motion import bounds
        bb=bounds(v);cx=(bb[0]+bb[3])/2;cz=(bb[2]+bb[5])/2
        bore=BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(cx,bb[1]-1,cz),gp_Dir(0,1,0)),6.35/math.cos(math.pi/6),bb[4]-bb[1]+2).Shape()
        found[n]=BRepAlgoAPI_Cut(v,bore).Shape();info[n]+=' Bore uses the circumscribed 1/2-inch hex envelope; exact flank engagement is a separate check.'
    return rosters
