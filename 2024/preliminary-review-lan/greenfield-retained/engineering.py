from travel import *
from math import atan2,degrees,hypot
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
def check_limit():
    neutral=mat(0,*pivot)@inverse
    ext=np.array([bounds(place(G[n],neutral)) for n in moving]);lo=ext[:,:3].min(axis=0)-pivot;hi=ext[:,3:].max(axis=0)-pivot
    assert lo[0]>=-18.0001 and hi[0]<=643.001 and lo[2]>=-44.001 and hi[2]<=67.001,(lo,hi)
    # The slide schedule explicitly subtracts the support function of this bounding rectangle.
    return dict(sectionBounds=[lo.tolist(),hi.tolist()],frameFront=340,allowedFront=644.8,maxProgrammedFront=634,continuousReserve=10.8,minimumSlideX=634-hypot(643,67),maximumSlideX=185,lockedForwardOverrun=185+637-644.8)
def service(theta):
    ms=mat(theta,pivot_x(theta),0,pivot[2])@inverse
    ps=mat(0,pivot_x(theta)-185,0,0)
    pack=G['Battery and terminal case'];a=bounds(pack);sweep=box(a[0],a[1],a[2],a[3]-a[0],a[4]-a[1],a[5]-a[2]+370)
    hits=[];closest=1000;pair=''
    for n,s in G.items():
        if n in [held,'Pack lead','Distribution lead','Battery and terminal case','Electronics deck']:continue
        sh=posegeom(s,ms if n in moving else ps if n in slide else identity);d=distance(sh,sweep)
        if d<closest:closest=d;pair=n
        if d<.001:
            v=common(sh,sweep)
            if v>.001:hits.append([n,v])
    return dict(theta=theta,hits=hits,closest=pair,clearance=closest,scope='370 mm continuous straight box pull, disconnected cables, empty robot, pack support deck excluded')
def approach():
    shs={n:posegeom(s,poses(cases[2],0)[group(n)]) for n,s in G.items() if n!=held}
    ring=lambda dx:place(torus,mat(0,-dx,0,0)@m0)
    # Close before the ring moves through the nearest candidate. 2 mm increments and bisect first occupied interval.
    trace=[]
    for n in ['Nose pickup roller','NOTE runner · left','NOTE runner · right','Opposed traction belt · left','Blue bumper set']:
        stock=shs[n];v=0;gap=distance(ring(0),stock);first=None
        for dx in range(2,102,2):
            if not close(bounds(ring(dx)),bounds(stock)):continue
            v=common(ring(dx),stock)
            if v>.001:
                a,b=dx-2,dx
                for _ in range(9):
                    m=(a+b)/2
                    if common(ring(m),stock)>.001:b=m
                    else:a=m
                first=b;break
        trace.append(dict(body=n,initialGap=gap,firstOverlapAtApproachMm=first))
    powered=shs['Nose pickup roller'];drivegap=[distance(place(torus,floorpose(t/24)[0]),powered) for t in range(25)]
    return dict(approach=trace,poweredRollerGapOnImposedTip=[min(drivegap),max(drivegap)],meaning='A free piece needs a driven contact across the tip. The animation prescribes orientation and cannot validate that handoff.')
if __name__=='__main__':
    result=dict(reach=check_limit(),service=[service(park),service(80)],entry=approach())
    dump(HERE/'engineering.json',result);print(result)

