"""Check the two installed hook throats against the purchased field chain in its parked shape."""
from sections import *
sys=__import__('sys');sys.path.insert(0,str(root));from layout import box,mat,place,cyl,cut,fuse,Y,common,distance,viewmesh,bounds
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Shape
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
def vol(s):g=GProp_GProps();BRepGProp.VolumeProperties_s(s,g);return g.Mass()
design=json.loads((root/'follower-01/review-data.json').read_bytes());chain=TopoDS_Shape();assert BRepTools.Read_s(chain,str(p/'working-chain.brep'),BRep_Builder())
designPose=next(c for c in design['cases'] if c['key']=='climb')
from math import pi,asin,hypot,atan2,sin,cos,degrees
ss=designPose['chain'][0][11];nom_x=designPose['positions'][0][0];nom_y=designPose['positions'][0][1]
sec=[slab(nom_y+y,4.01) for y in [-100.88,100.88]];pts=np.vstack(sec);cx,cz=((pts.max(0)+pts.min(0))/2)[[0,2]]
def ik(s,z):
    th=pi-asin((z-501)/hypot(s,232))-atan2(232,s);return degrees(th),cx-185-s*cos(th)+232*sin(th)
def readhook(side):
    from showcase_contacts import load_solids
    src=root/'follower-01/paired-follower.step';G,_=load_solids(src)
    return {n.replace(chr(194),''):place(sh,np.linalg.inv(mat(125,185,0,501))) for n,sh in G.items() if 'Chain hook' in n or 'Hook keeper' in n}
hooks=readhook('both');parts={}
for side,yy in [('left',100.88),('right',-100.88)]:
    chainPart=BRepAlgoAPI_Common(chain,box(cx-40,nom_y+yy-4.25,690,80,8.5,85)).Shape()
    assert vol(chainPart)>50
    parts[side]=chainPart
def check(s,z,openMm=0,dx=0,dy=0):
    th,x=ik(s,z);pose=mat(th,x+185+dx,nom_y+dy,501);rows={}
    for side in parts:
        for label in ['Chain hook','Hook keeper']:
            sh=hooks[label+' · '+side];m=pose if label=='Chain hook' else pose@mat(0,0,0,openMm)
            b=place(sh,m);part=parts[side];rows[label+' '+side]=[common(part,b),distance(part,b)]
    return dict(localChainCenterX=s,deg=th,chassisX=x,contacts=rows)
nominal=check(180,ss[2],dx=nom_x-ik(180,ss[2])[1]);new=check(180,cz)
result=dict(parent=design['step'],chain=json.loads((p/'source.json').read_bytes()),assumedAxisZmm=ss[2],measuredMidRangeZmm=cz,measuredMidRangeXmm=cx,
    old=nominal,registered=new,openApproach=[check(float(x),cz,35) for x in np.linspace(238,180,15)],jammedKeeper=[check(float(x),cz,0) for x in (210,203,196,190)])
(p/'entry-result.json').write_text(json.dumps(result,indent=2))
print('centers',ss[2],cz,cx,'old',nominal['contacts'],'new',new['contacts'])


