"""Belt contact force integrated along each occupied arc of the free ring.
Coefficients are per meter of centerline. Match the previous normal force at a 6 mm centered squeeze.
"""
import numpy as np,math
def belt_patch(diff,pts,center,v,omega,R,speed,dt,parameters):
    k,damping,stick,mu,cap=parameters
    ds=np.linalg.norm(diff,axis=1);depth=.0434-ds;sel=depth>0
    if not np.any(sel):return np.zeros(6),0.,0.,0.,0.
    spacing=.1524*math.tau/len(ds);direc=diff[sel]@R.T/ds[sel,None]
    levers=pts[sel]-center-direc*.0254;velocity=v+np.cross(omega,levers)
    normal=np.maximum(0,k*depth[sel]-damping*np.sum(velocity*direc,axis=1))*spacing
    tangents=R[:,0]-direc*np.outer(direc@R[:,0],np.ones(3))
    tangents/=np.maximum(.01,np.linalg.norm(tangents,axis=1))[:,None]
    slip=speed-np.sum(velocity*tangents,axis=1)
    ft=np.clip(stick*slip*spacing,-mu*normal,mu*normal)
    ft*=min(1.,cap/max(1e-8,np.sum(abs(ft))))
    F=direc*normal[:,None]+tangents*ft[:,None]
    wr=np.r_[F.sum(axis=0),np.cross(levers,F).sum(axis=0)]
    return wr,float(normal.sum()),float(ft.sum()),float(spacing*len(ft)),float(np.sum(abs(ft*slip)))
def coefficients(n=2048):
    phi=np.arange(n)*math.tau/n;pen=np.maximum(0,.0434-abs(.1898-.1524*np.sin(phi)))
    length=.1524*math.tau/n;arc=(pen>0).sum()*length
    return [27./(length*sum(pen)),5./arc,45./arc,.78,18.]
if __name__=='__main__':
    print(coefficients())

