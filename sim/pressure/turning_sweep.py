"""Conservative native-frame envelopes of a freely rotating torus on an attained turn."""
import json
from pathlib import Path

import numpy as np


class NativeTurnPath:
    def __init__(self, folder):
        result=json.loads((folder/"result.json").read_bytes())
        self.dt=result["dt"]
        rows=[result["initialQ"]]
        with (folder/"trace.jsonl").open() as trace:
            for i,line in enumerate(trace,1):
                row=json.loads(line)
                if abs(row["t"]-i*self.dt)>1e-9:raise ValueError("Nonuniform trace")
                rows.append(row["q"])
        if result["outcome"]!="HORIZON" or abs(result["end"]-(len(rows)-1)*self.dt)>1e-8:
            raise ValueError("Incomplete native path")
        q=np.asarray(rows)
        quat=q[:,8:12]
        if np.max(np.abs(np.linalg.norm(quat,axis=1)-1))>1e-9:raise ValueError("Nonunit NOTE orientation")
        w,x,y,z=quat.T
        normal=np.column_stack((2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)))
        p=1000*q[:,5:8].copy();p[:,:2]-=1000*q[:,:2]
        c,s=np.cos(q[:,2]),np.sin(q[:,2])
        def robot(a):return np.column_stack((c*a[:,0]+s*a[:,1],-s*a[:,0]+c*a[:,1],a[:,2]))
        fixed=robot(p);nr=robot(normal)
        car=fixed.copy();car[:,2]-=1000*q[:,3]
        c,s=np.cos(q[:,4]),np.sin(q[:,4])
        def inv_pitch(a):return np.column_stack((c*a[:,0]+s*a[:,2],a[:,1],-s*a[:,0]+c*a[:,2]))
        pivot=np.array([178.,0.,25.4])
        self.frames={"fixed":(fixed,nr),"carriage":(car,nr),"cradle":(pivot+inv_pitch(car-pivot),inv_pitch(nr))}
        # Bound deviation from each small segment's endpoint chord by sup|f''|/8
        # for linear world translation and constant-angular-speed quaternion interpolation.
        dyaw=np.abs(np.diff(q[:,2])); dpitch=np.abs(np.diff(q[:,4]))
        norm=lambda a:np.linalg.norm(a,axis=1)
        xy=np.maximum(norm(p[:-1,:2]),norm(p[1:,:2]))
        chassis_curve=(2*dyaw*norm(np.diff(p[:,:2],axis=0))+dyaw**2*xy)/8
        moving=p.copy();moving[:,2]-=1000*q[:,3]+25.4
        speed=norm(np.diff(moving,axis=0))+dyaw*xy
        reach=178.+np.maximum(norm(moving[:-1]),norm(moving[1:]))
        cradle_curve=chassis_curve+(2*dpitch*speed+dpitch**2*reach)/8
        # Quaternion sign does not change orientation. Use chord length for accuracy near zero.
        a,b=quat[:-1],quat[1:]
        qchord=np.minimum(norm(b-a),norm(b+a))
        drot=4*np.arcsin(np.minimum(1.,qchord/2))
        self.errors={g:(cradle_curve if g=="cradle" else chassis_curve,
                    (drot+dyaw+(dpitch if g=="cradle" else 0.))**2/8) for g in self.frames}
        self.length=len(q)-1

    def interval(self,first,last):
        mid=(first+last)//2;span=slice(first,last+1);frames={}
        for group,(centers,normals) in self.frames.items():
            ce,ne=self.errors[group]
            ce=float(max(ce[first:last]));ne=float(max(ne[first:last]))
            c,n=centers[mid],normals[mid]
            delta=np.max(np.linalg.norm(centers[span]-c,axis=1))+ce
            # A rigid rotation mapping one unit normal to another moves a
            # circle point by at most R times the chord between the two normals.
            turn=np.max(np.linalg.norm(normals[span]-n,axis=1))+ne
            frames[group]=dict(center=c.tolist(),normal=n.tolist(),padMm=float(delta+152.4*turn+.002))
        cy=self.frames["fixed"][0][span,1];error=float(max(self.errors["fixed"][0][first:last]))
        return dict(first=first,last=last,begin=first*self.dt,end=last*self.dt,frames=frames,
                    yBoundsMm=[float(min(cy)-177.802-error),float(max(cy)+177.802+error)])

    def intervals(self,ranges,stride):
        return [self.interval(i,min(i+stride,b)) for a,b in ranges for i in range(a,b,stride)]
