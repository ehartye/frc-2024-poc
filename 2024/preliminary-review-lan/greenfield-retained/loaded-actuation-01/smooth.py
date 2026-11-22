"""Finite-acceleration slide command for a 3.4 s lift with unchanged endpoint holds."""
import numpy as np
from scipy.ndimage import convolve1d
from stroke import scheduled
times=np.arange(-.25,3.65+.00025,.00025)
def table(deploy,final,x0):
    t=times;u=np.clip(t/3.4,0,1);b=u*u*(3-2*u);a=deploy+(final-deploy)*b
    original=(scheduled(a)+(x0-scheduled(deploy))*(1-b))*.001
    kernel=np.arange(-320,321)*.00025
    w=1+np.cos(2*np.pi*kernel/.16)
    smooth=convolve1d(original,w/w.sum(),mode='nearest')
    z=np.clip(np.minimum(t,3.4-t)/.20,0,1);h=z**3*(10-15*z+6*z*z)
    x=original+h*(smooth-original)
    return x,np.gradient(x,.00025)

