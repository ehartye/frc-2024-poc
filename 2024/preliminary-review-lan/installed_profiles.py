"""Install the measured receiver and register motor and stock profiles in robot coordinates."""
import json,hashlib
from pathlib import Path
import numpy as np
from OCP.BRep import BRep_Builder
from OCP.BRepTools import BRepTools
from OCP.TopoDS import TopoDS_Shape
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt,gp_Trsf
def install(review,root,track,found,bodies,info,load,bounds,mesh,sha):
    source=json.loads((root/'motors.json').read_bytes())
    index_dir=track.parents[1]/'stator-index';index=json.loads((index_dir/'index.json').read_bytes())
    roster=json.loads((review.parent/'issued-occupancy.json').read_bytes())
    baseline={r['key']:r['bounds'] for r in roster['rows']}
    stock={'Runner right','Runner left','Saddle uphill','Saddle downhill','Feeder cross tie uphill','Feeder cross tie downhill','Guide backing beam','crossmember','strut -1','strut 1','front beam','rear beam','upper cross tie support','Pickup foot -1','Pickup foot 1','integral outer wheel bearing carrier','formed motor yoke','upper web -1','machined uphill web','sill 1','sill -1','Front perimeter tube','captured arm -1','captured arm 1','turned and keyed shaft','outer REV-21-1916','inner REV-21-1916','removable inner cheek','shaft end keeper'}
    def add(n,solid,role,group,material,basis):
        found[n]=solid;info[n]=basis
        if n not in bodies:bodies[n]=dict(**mesh(solid),category=role,group=group,material=material,added=True)
    for row in index['rows']:
        n=row['key']
        if n not in stock:continue
        p=index_dir/row['file'];assert sha(p)==row['sha256'];s=TopoDS_Shape();assert BRepTools.Read_s(s,str(p),BRep_Builder())
        assert np.max(np.abs(np.array(bounds(s))-baseline[n]))<.01
        group='Swing supports' if n.startswith('Pickup') else 'Receiver and scorer supports' if n.startswith(('Runner','Saddle','Feeder','Guide','formed','machined','integral','upper')) else 'Frame'
        add(n,s,'structure',group,'Aluminum stock (planning)','Placed project stock from the indexed source; world bounds agree with the issued structure.')
    feeder=review.parent/'shared-feeder-stop.step';assert sha(feeder)==json.loads((review.parent/'shared-stop-check.json').read_bytes())['step']
    n='Feeder carriage envelope';found[n]=next(iter(load(feeder)[0].values()));info[n]='One-piece moving feeder spine with flag and clearance relief; follows the same carriage stroke.'
    def cyl(o,u,r,l):return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*o),gp_Dir(*u)),r,l).Shape()
    def model(row,neo):
        p=np.array(row['origin']);axis=np.array(row['axis']);lo=np.array(row['box'][:3]);hi=np.array(row['box'][3:]);ind=np.abs(axis).argmax()
        z=[(v-p[ind])/axis[ind] for v in (lo[ind],hi[ind])];start,end=sorted(z)
        if neo:
            # Register motor face from coaxial 9.525 mm pilot. Output shaft points away from the main barrel.
            pilot=next(v for v in row['spans'] if abs(v[2]-9.525)<.01);shaft=next(v for v in row['spans'] if abs(v[2]-4)<.01 and v[1]-v[0]>8)
            positive=(shaft[0]+shaft[1])<(pilot[0]+pilot[1]);face=pilot[1] if positive else pilot[0];r=29.99
            spans=[(start,face,4),(face,end,r)] if positive else [(start,face,r),(face,end,4)]
        else:
            main=[v for v in row['spans'] if abs(v[2]-17.5)<.1];left=min(v[0] for v in main);right=max(v[1] for v in main)
            spans=[(start,left,1.5875),(left,right,17.5),(right,end,6.5)]
        s=None
        for a,b,r in spans:
            if b-a<.01:continue
            piece=cyl(p+axis*a,axis,r,b-a);s=piece if s is None else BRepAlgoAPI_Fuse(s,piece).Shape()
        return s
    for name,v in source['rows'].items():
        if name.startswith(('Front ','Rear ')):
            corner=name.split(' REV')[0];is_drive=name.endswith('1650');n='Swerve '+('drive' if is_drive else 'steer')+' motor · '+corner
            add(n,model(v,is_drive),'electrical','Swerve','NEO brushless motor' if is_drive else 'NEO 550 brushless motor','Case and shaft envelope on the verified purchased motor axis. Leads and housing details omitted.')
    package=track.parents[2]/'four-station-package-20261121';axes=json.loads(Path(json.loads((package/'physical-basis.json').read_bytes())['files']['directedAxes']['path']).read_bytes())['stations']
    mats=[np.array([*axes[str(i)]['matrix'],0,0,0,1]).reshape(4,4) for i in range(4)]
    fixed=model(source['rows']['NEO motor'],True)
    for i in range(4):
        mat=mats[i]@np.linalg.inv(mats[0]);t=gp_Trsf();t.SetValues(*mat[:3].ravel())
        n='Motor station '+str(i);found[n]=fixed.Moved(TopLoc_Location(t));info[n]='Stepped NEO case and shaft on the registered motor station axis. Flexible lead and casing detail remain outside this profile.'
    from tube import lead
    route=track.parents[2]/'battery-electrical-bay-20261121'/'pair.json'
    checks=json.loads((review/'sources.json').read_bytes())['referenceFiles'];assert sha(route)==checks[str(route)]
    wires=json.loads(route.read_bytes())
    for sign in ('positive','negative'):
        n=sign+' power lead corridor';found[n]=lead(wires[sign]['waypoints']);info[n]='Round routing allowance, Ø9.75 mm (Ø7.75 mm lead plus 1 mm radial margin); 50 mm centerline bend radius on the recorded route.'
    from upper_package import products
    members=products(review,track,add,load,sha,found,info)
    (root/'upper-members.json').write_text(json.dumps(members))
    return found
