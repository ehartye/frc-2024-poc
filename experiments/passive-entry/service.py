"""Conservative swept component bounds against the hash-checked robot mesh, in mm."""
import json
import math
import sys
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/"registered-custody"))
from contact import meshes
D=json.loads((HERE/"part.json").read_bytes())
GROUPS=dict(meshes())
TARGET=lambda side:"R7 "+("538" if side==1 else "536")+" intake candidate original cradle side weldment "+str(side)


def primitives(side):
    def box(label,x0,y0,z0,x1,y1,z1):
        ylo,yhi=sorted((side*y0,side*y1))
        return label,np.array((x0,ylo,z0,x1,yhi,z1),float)
    boxes=[box("low beam",275,211.4,4,505,217.75,15),
           box("mount web",275,211.4,14,311,217.75,41.275),
           box("tongue",275,203.825,38.1,311,217.75,41.275)]
    for x in D["mountXmm"]:
        boxes.extend([box("mount head",x-2.75,D["mountYmm"]-2.75,41.275,x+2.75,D["mountYmm"]+2.75,44.275),
                      box("mount shank",x-1.5,D["mountYmm"]-1.5,33.275,x+1.5,D["mountYmm"]+1.5,41.275)])
    for i,(x,y) in enumerate((D["railPathMm"][0],D["railPathMm"][-1])):
        boxes.append(box("return leg",x-2,y-2,6.5,x+2,218,10.5))
        boxes.append(box("set screw",x-1.5,213.1,9.8,x+1.5,216.1,15.8))
    for a,b in zip(D["railPathMm"],D["railPathMm"][1:]):
        # Subdivide inclined leg to avoid attributing the full flare to every x.
        for q in range(12):
            p=np.array(a)+(np.array(b)-a)*q/12
            r=np.array(a)+(np.array(b)-a)*(q+1)/12
            boxes.append(box("sleeve",p[0]-7,p[1]-7,1.5,r[0]+7,r[1]+7,15.5))
    return boxes


TRIBOUNDS={id(ts):(ts.min(axis=1),ts.max(axis=1)) for ts in GROUPS.values()}

def hit(ts,box,pad=0.):
    low,high=box[:3]-pad,box[3:]+pad
    tl,th=TRIBOUNDS[id(ts)]
    tri=ts[(th>=low).all(axis=1)&(tl<=high).all(axis=1)]
    if not len(tri):return False
    v=tri-(high+low)/2
    half=(high-low)/2
    edges=np.roll(v,-1,axis=1)-v
    axes=[np.cross(edges[:,0],edges[:,1])]
    for k in range(3):
        for direction in np.eye(3):
            axes.append(np.cross(edges[:,k],direction))
    active=np.ones(len(v),dtype=bool)
    for axis in axes:
        projection=np.einsum("ij,ikj->ik",axis,v)
        radius=abs(axis)@half
        active &= (projection.min(axis=1)<=radius+1e-7)&(projection.max(axis=1)>=-radius-1e-7)
    return bool(active.any())


# Group AABBs prune triangles before applying the exact triangle/box axes.
CACHED={name:(ts.min(axis=(0,1)),ts.max(axis=(0,1)),ts) for name,ts in GROUPS.items()}
def sweep(boxes,side,mode,pad=0.):
    seen=set()
    for label,b in boxes:
        a=b.copy()
        if mode=="installed":
            # Original moving host and candidate rise together.
            pass
        elif mode in ("removeOut","removeNose","removeDrop","removeExit","verticalControl"):
            if label.startswith("mount " ) and label!="mount web":continue
            if mode=="removeOut":a[:3]+=[0,min(0,side*2),17];a[3:]+=[0,max(0,side*2),17]
            elif mode=="removeNose":a[:3]+=[0,side*2,17];a[3:]+=[70,side*2,17]
            elif mode=="removeDrop":a[:3]+=[70,side*2,12];a[3:]+=[70,side*2,17]
            elif mode=="removeExit":a[:3]+=[70,side*2,12];a[3:]+=[280,side*2,12]
            else:a[5]+=120
        for name,(lo,hi,ts) in CACHED.items():
            check=a.copy()
            moving=(name.startswith("R7 cradle /") or name.startswith("R7 carriage /")
                    or name in ("534","535") or name in CRADLE)
            if mode=="installed" and not moving:check[5]+=D["liftMm"]
            if mode in ("installed","verticalControl") and name==TARGET(side) and label in ("tongue","mount shank"):
                continue
            if np.any(hi<check[:3]-pad) or np.any(lo>check[3:]+pad): continue
            if hit(ts,check,pad):seen.add((label,name))
    return [list(pair) for pair in sorted(seen)]


CRADLE=json.loads((HERE.parent/"registered-custody/cradle-bounds.json").read_bytes())
def wheels(boxes,side,shift=0):
    # Continuous steering; each native triangle's radial maximum is at a vertex.
    # The radial bound is conservative across that triangle's vertical interval.
    lowest=(1e5,"","")
    for name,ts in GROUPS.items():
        if not (("carrier /" in name or "wheel /" in name) and ("Front " in name or "Rear " in name)):continue
        center=np.array([260 if "Front" in name else -260,260 if "left" in name else -260])
        lo=ts[:,:,2].min(axis=1);hi=ts[:,:,2].max(axis=1)
        radius=np.linalg.norm(ts[:,:,:2]-center,axis=2).max(axis=1)
        for label,b in boxes:
            low=b[:2]+[0,side*shift];high=b[3:5]+[0,side*shift]
            ring=radius[(hi>=b[2])&(lo<=b[5]+D["liftMm"])]
            if ring.size:
                gap=float(np.linalg.norm(np.maximum(0,np.maximum(low-center,center-high))))-float(ring.max())
                lowest=min(lowest,(gap,label,name))
    return [round(lowest[0],3),*lowest[1:]]


def witnesses(side):
    # Vertical line through each pilot tap. Face triangles at each station
    # establish the upper surface and uninterrupted available drill depth.
    ts=GROUPS[TARGET(side)]
    rows=[]
    for x in D["mountXmm"]:
        y=side*D["mountYmm"]
        heights=[]
        for a,b,c in ts:
            edge=np.column_stack((b[:2]-a[:2],c[:2]-a[:2]))
            if abs(np.linalg.det(edge))<1e-9:continue
            u,v=np.linalg.solve(edge,[x-a[0],y-a[1]])
            if min(u,v)>=-1e-9 and u+v<=1+1e-9:heights.append(a[2]+u*(b[2]-a[2])+v*(c[2]-a[2]))
        rows.append([x,*sorted({round(float(z),3) for z in heights})])
    return rows


def wheel_control(side):
    # A tire vertex strictly inside a solid part of the outboard-shifted web.
    center=np.array([260,side*260])
    wheel="Front "+("left" if side==1 else "right")+" wheel / R7 tire"
    vertices=GROUPS[wheel].reshape(-1,3)
    for deg in range(0,360,5):
        a=math.radians(deg)
        rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
        xy=(vertices[:,:2]-center)@rot.T+center
        inside=(xy[:,0]>275.2)&(xy[:,0]<310.8)&(side*xy[:,1]>246.6)&(side*xy[:,1]<252.55)&(vertices[:,2]>15.2)&(vertices[:,2]<38)
        if np.any(inside):
            i=np.flatnonzero(inside)[0]
            return {"angleDeg":deg,"article":wheel,"pointMm":[*xy[i].round(3),round(vertices[i,2],3)]}
    raise ValueError("Missing displaced-web control")


def role_room():
    # All pose records from the matched free-piece runs must fit this measured envelope.
    # The projection of a tilted torus stays inside its outer-radius disk.
    cxlo,cxhi,ymax,zmax=405.,752.,6.5,57.0
    out={}
    for name,tri in GROUPS.items():
        low,high=tri.min(axis=1),tri.max(axis=1)
        dx=np.maximum(0,np.maximum(low[:,0]-cxhi,cxlo-high[:,0]))
        dy=np.maximum(0,np.maximum(low[:,1]-ymax,-ymax-high[:,1]))
        radial=np.hypot(dx,dy)-177.8
        # The fixed and moving parent meshes start parked. Moving parts rise;
        # using their parked lower bounds is conservative for this screen.
        sep=np.maximum(radial,low[:,2]-zmax)
        out[name]=round(float(sep.min()),3)
    return {"centerXmm":[cxlo,cxhi],"maxAbsYmm":ymax,"topMm":zmax,
            "minParentMm":min(out.values()),"limitingParent":min(out,key=out.get),
            "backingMm":min(v for k,v in out.items() if k.startswith(("R7 504 ","R7 519 "))),
            "treadsMm":min(out["534"],out["535"]),"wheelParkedMm":min(v for k,v in out.items() if "wheel /" in k),
            "rearReturnMm":round(math.hypot(cxlo-350,181-ymax)-177.8-2,3),
            "frontReturnMm":round(193-ymax-177.8-2,3)}


def analyze():
    record={}
    for side in (-1,1):
        boxes=primitives(side)
        tools=[("driver",np.array([x-3.25,side*D["mountYmm"]-3.25,44.28,x+3.25,side*D["mountYmm"]+3.25,124.28])) for x in D["mountXmm"]]
        taps=[("pilot",np.array([x-1.25,side*D["mountYmm"]-1.25,30.9,x+1.25,side*D["mountYmm"]+1.25,38.1])) for x in D["mountXmm"]]
        tap_hits=[pair for pair in sweep(taps,side,"drill",.2) if pair[1]!=TARGET(side)]
        bare=[b for b in boxes if b[0] not in ("mount shank","mount head")]
        rise=[]
        for name,b in bare:
            lifted=b.copy();lifted[5]+=17
            # Host's mating tongue touches only at the initial face.
            rise.extend(sweep([(name,lifted)],side,"verticalLift"))
        short=[box+np.array([0,0,3,280,0,3]) for n,box in bare if n=="tongue"]
        service_boxes=[(label,b+np.array([0,min(0,side*2),0,280,max(0,side*2),17])) for label,b in bare]
        record[str(side)]= {"installed":sweep(boxes,side,"installed",.2),
                          "driver":sweep(tools,side,"driver",.2),"pilotAgainstOtherBodies":tap_hits,
                          "riseRaw":rise,"frontExtraction":{m:sweep(bare,side,m,.2) for m in ("removeOut","removeNose","removeDrop","removeExit")},
                          "shortRiseHostControl":hit(GROUPS[TARGET(side)],short[0],.2),
                          "verticalControl":sweep(bare,side,"verticalControl"),
                          "wheel360":wheels(boxes,side),"serviceWheel360":wheels(service_boxes,side),
                          "outboardWheelControl":wheel_control(side),
                          "hostRayZmm":witnesses(side)}
    record["carriedNote"]=role_room()
    return record


if __name__=="__main__":
    (HERE/"service.json").write_bytes((json.dumps(analyze(),indent=2)+"\n").encode())
