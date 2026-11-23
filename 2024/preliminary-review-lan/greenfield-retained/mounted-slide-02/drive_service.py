"""Fit the two radial clamp fasteners and four removable motor-lead reservations to the current cartridge."""
from pathlib import Path
import math,sys,json,hashlib,itertools,numpy as np
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'assembly-recheck-01'))
from sweep import load_solids,name,bounds,place,mat,common,distance
from layout import box,cyl,fuse,cut,G,write_step,X
from OCP.gp import gp_Pnt,gp_Dir,gp_Ax2,gp_Circ
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge,BRepBuilderAPI_MakeFace,BRepBuilderAPI_MakeWire
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakePipe
from OCP.OCP.collections import Array1_gp_Pnt as TColgp_Array1OfPnt
from OCP.Geom import Geom_BezierCurve
def objs(f):return {name(k):s for k,s in load_solids(f)[0].items()}
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
fx=p/'slide-cartridge.step';parent=root/'guided-hoist-01/service-locked-robot.step'
assert sha(fx)==json.loads((p/'fit.json').read_bytes())['fixture']
H=objs(fx);old=H.pop('Trial coupling blank');motor=H.pop('SOLID')
R=objs(parent);del R['Slide drive motor'];del R['Slide bearing mount 220']
stock=R|H
def peg(x,y,z,r,L):return cyl((x,y,z),(0,0,1),r,L)
def shift(s,xyz):return place(s,mat(0,*xyz))
B={k:np.array(bounds(s)) for k,s in stock.items()}
def check(subj,ob):
    hits=[];tests=0
    for k,s in subj.items():
        b=np.array(bounds(s))
        for j,v in ob.items():
            c=B[j] if j in B else np.array(bounds(v))
            if np.any((b[:3]>c[3:]+.0001)|(c[:3]>b[3:]+.0001)):continue
            tests+=1;n=common(s,v)
            if n>.001:hits.append([k,j,n])
    return dict(subjects=len(subj),obstacles=len(ob),narrow=tests,positive=hits)
# The aligned clamp screws are accessible together in a single service clock. The bore registers both shafts.
hub=cut(old,box(233.4,275,427.65,18.2,10,.7));screws={};tools={}
for k,x in [('spindle',238.4),('motor',247.6)]:
    y=281.05
    for z,r,l in [(418,1.025,10),(428,1.4,2.00),(430.00,2.45,8.0)]:hub=cut(hub,peg(x,y,z,r,l))
    screws[k]=fuse(peg(x,y,422.00,1.25,8),peg(x,y,430.,2.25,2.5));tools[k]=peg(x,y,432.50,1.65,75)
checks={};checks['tools at service clock']=check(tools,stock|{'hub':hub})
checks['coupler parked']=check({'hub':hub,**screws},stock)
# Entire rotating hardware stays inside the union of these cylinders; include both occupied screw heads.
env=cyl((233.5,275,428),X,math.hypot(6.05+2.25,4.50),18)
checks['rotating envelope']=check({'rotor':env},{k:v for k,v in stock.items() if k!='Spindle'})
D=np.array([0.,1.,1.])/math.sqrt(2);V=np.array([0.,1.,-1.])/math.sqrt(2);Xvec=np.array(X,dtype=float)
cent=np.array([261.,275.,428.])
def smoothTube(a,e,r):
    ar=TColgp_Array1OfPnt(1,4);L=(e-a)@D
    for j,k in enumerate([a,a+D*L/3,e-D*L/3,e],1):ar.SetValue(j,gp_Pnt(*k))
    w=BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(Geom_BezierCurve(ar)).Edge()).Wire()
    f=BRepBuilderAPI_MakeFace(BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(*a),gp_Dir(*D)),r)).Edge()).Wire()).Face()
    return BRepOffsetAPI_MakePipe(w,f).Shape()
# These positions are the centers of the four flat wire-end faces on the purchased file.
exits=np.array([[16.4835,24.2615],[19.312,21.433],[21.787,18.9585],[24.262,16.484]])
leads={};plugs={};sockets={};finger={};guide=[];wireRows=[]
for i,(y,z) in enumerate(exits):
    a=cent+[0,y,z];e=cent+D*90+V*(-18+12*i)
    r=1.75 if i==0 else 1.625
    leads[str(i)]=smoothTube(a,e,r)
    plugR=7. if i==0 else 4.5
    plugs[str(i)]=cyl(e,D,plugR,22)
    sockets[str(i)]=cyl(e+22*D,D,plugR,21)
    finger[str(i)]=cyl(e+22*D,D,10,48)
    lat=abs((e-a)@V);L=(e-a)@D
    wireRows.append(dict(start=a.tolist(),end=e.tolist(),radius=r,minBendRadiusBoundMm=L*L/(6*lat),connectorReservationDiameterMm=2*plugR))
# Each disconnected chassis socket is pulled 24 mm along the lead tangent and held to that side.
bundle={**{'lead '+k:v for k,v in leads.items()},**{'plug '+k:v for k,v in plugs.items()}}
checks['routed cables and plugs']=check(bundle|{'socket '+k:v for k,v in sockets.items()},stock|{'hub':hub}|{'clamp '+k:v for k,v in screws.items()})
checks['wires against each other']=[check({a:sa},{b:sb}) for (a,sa),(b,sb) in itertools.combinations(bundle.items(),2)]
checks['raised clamp drivers alongside leads']=check(tools,bundle)
checks['disconnect reach vs fixed installed stock']=check(finger,stock)
checks['disconnect cylinder stroke']=check({'socket '+k:cyl(np.array(wireRows[int(k)]['end'])+22*D,D,(7 if k=='0' else 4.5),45) for k in sockets},stock)
park={k:shift(s,24*D) for k,s in sockets.items()}
# Conservative extruded X slabs bound every point of each motor-side wire as it leaves the robot.
swept={};endpoints={}
for k,s in bundle.items():
    b=np.array(bounds(s));swept[k]=box(*b[:3],b[3]-b[0]+45,b[4]-b[1],b[5]-b[2])
checks['45 mm draw after permanent socket is parked']=check(swept,stock|{'parked socket '+k:v for k,v in park.items()})
checks['draw with connected plugs left in place']={}
for k,plug in plugs.items():
    # Axial draw is transverse to the cable. Connected pins resist the first sideways move.
    pin=cyl(np.array(wireRows[int(k)]['end'])+22*D,D,1.65,6)
    host=cut(sockets[k],cyl(np.array(wireRows[int(k)]['end'])+21.95*D,D,1.80,6.1))
    checks['draw with connected plugs left in place'][k]=[common(pin,host),common(shift(pin,[1,0,0]),host)]
maximumTorque=(38.921-1.4)*.0097;thresholds=[dict(mu=mu,requiredNormalN=maximumTorque/(.001585*mu)) for mu in [.10,.15,.20,.25]]
r=dict(parent=sha(parent),cartridge=sha(fx),phaseWireGauge='16 AWG from REV-21-1651-DS',wireFaces='The four outgoing planar faces in exits.log are at source X16.5. The previous keepout began at installed X282 and omitted these stubs.',path=wireRows,checks=checks,torqueNm=maximumTorque,motorShaftFrictionThresholds=thresholds,scope='Clamp and pins are preliminary machining and service envelopes. Connector housings, hand tooling, strain relief, actual leads, clamp preload/slip, allowable total runout and thrust bearing remain to validate.')
G.clear();G.update({'Double pinch hub':hub,**{'M2.5 pinch '+k:v for k,v in screws.items()},**bundle,**{'socket '+k:v for k,v in sockets.items()}})
out=p/'drive-service.step';write_step(out);r['step']=sha(out);(p/'drive-service.json').write_text(json.dumps(r,indent=2))
for k,v in checks.items():print(k,v['positive'] if isinstance(v,dict) and 'positive' in v else v)


