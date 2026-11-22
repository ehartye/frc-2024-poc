"""Produce dimensioned nominal solids in the same initial robot frame as the recorded motion."""
import numpy as np
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeCylinder,BRepPrimAPI_MakeTorus
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Ax2,gp_Dir,gp_Pnt,gp_Trsf
def nominal(solids,bounds,track,raw,read):
    updated={};info={}
    def cylinder(p,axis,r,length):return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*p),gp_Dir(*axis)),r,length).Shape()
    for n,s in solids.items():
        if n.endswith(' tire'):
            box=np.array(bounds(s));dim=box[3:]-box[:3];axis=np.eye(3)[np.argmin(dim)];r=38.1;width=dim.min();c=(box[3:]+box[:3])/2
            updated[n]=cylinder(c-width/2*axis,axis,r,width);info[n]='Wheel outer profile · Ø76.2 mm, axial width from placed tire bounds. Internal construction omitted.'
    package=track.parents[2]/'four-station-package-20261121'/'physical-basis.json'
    for i,center in enumerate(read(package)['axesMm']):
        n='Wheel station '+str(i);updated[n]=cylinder(np.array(center)-[0,12.7,0],[0,1,0],53.34,25.4)
        info[n]='Operating tread profile · Ø106.68 × 25.4 mm, at the recorded station axis. Upper slide shown at zero stroke. Hub details omitted.'
    boomangle=np.load(track.with_name('joints.npy'))[0,1]
    pivot=[236.3692822684,0,337.90380493]
    # Same R_y(-angle) used by the package's boom pose.
    rotation=gp_Trsf();c=np.cos(boomangle);s=np.sin(boomangle)
    rotation.SetValues(c,0,-s,pivot[0],0,1,0,0,s,0,c,pivot[2])
    for y in [-203,203]:
        def box(dx,dy,dz):return BRepPrimAPI_MakeBox(gp_Pnt(-16-dx,y-dy/2,-dz/2),gp_Pnt(409.2419313+16+dx,y+dy/2,dz/2)).Shape()
        tube=BRepAlgoAPI_Cut(box(0,25.4,38.1),box(1,19.05,31.75)).Shape()
        for x in (0,409.2419313):
            hole=cylinder([x,y-13,0],[0,1,0],14.2875,26)
            # Shoulder has a broached drive socket, do not pretend it is a round bearing bore.
            if x>0:tube=BRepAlgoAPI_Cut(tube,hole).Shape()
        n='Boom '+str(y);updated[n]=tube.Moved(TopLoc_Location(rotation));info[n]='Hollow 25.4 × 38.1 mm boom · 3.175 mm wall, distal Ø28.575 opening. Simplified shoulder end. Stock size and endpoints from the retained swing design.'
    for n,col in [('Floor NOTE',9),('Preload NOTE',16)]:
        ctr=1000*(raw[0,col:col+3]-[raw[0,1],raw[0,2],0])
        w,x,y,z=raw[0,col+3:col+7]/np.linalg.norm(raw[0,col+3:col+7])
        normal=[2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)]
        updated[n]=BRepPrimAPI_MakeTorus(gp_Ax2(gp_Pnt(*ctr),gp_Dir(*normal)),152.4,25.4).Shape()
        info[n]='Nominal foam torus · 355.6 mm OD, 254 mm ID, 50.8 mm section.'
    return updated,info
