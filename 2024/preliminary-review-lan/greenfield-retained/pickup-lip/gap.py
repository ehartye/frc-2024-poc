import numpy as np
from scipy.spatial.transform import Rotation
from math import radians as rad
N=2048
ang=np.arange(N)*2*np.pi/N
ring=np.stack([152.4*np.cos(ang),152.4*np.sin(ang),np.zeros(N)],axis=1)
allowance=152.4*np.pi/N
def mat(angle,x=0,y=0,z=0):
    m=np.eye(4);m[:3,:3]=Rotation.from_euler('y',-angle,degrees=True).as_matrix();m[:3,3]=[x,y,z];return m
def quat(q,xyz):
    m=np.eye(4);m[:3,:3]=Rotation.from_quat([*q[1:],q[0]]).as_matrix();m[:3,3]=xyz;return m
def clearance(piece,flap):
    rel=np.linalg.inv(flap)@piece;v=ring@rel[:3,:3].T+rel[:3,3];box=np.maximum(abs(v+[0,0,26])-[1.5,37,26],0)
    boxgap=np.sqrt((box*box).sum(axis=1)).min()
    rad=np.sqrt(v[:,0]**2+v[:,2]**2)
    def cyl(r,w):
        return np.hypot(np.maximum(rad-r,0),np.maximum(abs(v[:,1])-w,0)).min()
    return np.array([boxgap,cyl(3,37),cyl(2,226)])-25.4-allowance

