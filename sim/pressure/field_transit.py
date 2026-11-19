"""Monotone lookahead along registered differential-drive field waypoints."""
import numpy as np


class TransitPath:
    """Steer toward a point ahead on successive field legs, slowing for the station."""

    def __init__(self, points, lookahead):
        xy=np.asarray(points,dtype=float)
        delta=np.diff(xy,axis=0)
        lengths=np.linalg.norm(delta,axis=1)
        keep=lengths>1e-9
        self.starts=xy[:-1][keep]
        self.lengths=lengths[keep]
        self.axes=delta[keep]/self.lengths[:,None]
        self.ends=np.cumsum(self.lengths)
        self.station=xy[-1]
        self.lookahead=lookahead
        self.progress=0.

    def target(self, position):
        if not len(self.ends):
            return self.station,np.linalg.norm(self.station-position)
        starts=self.ends-self.lengths
        # Search locally in path order, including both legs of a corner.
        # A later crossing of the same field position cannot skip a loop.
        stop=np.minimum(self.ends,self.progress+self.lookahead)
        start=np.maximum(starts,self.progress)
        legs=np.flatnonzero(stop>=start)
        along=np.sum((position-self.starts[legs])*self.axes[legs],axis=1)+starts[legs]
        along=np.clip(along,start[legs],stop[legs])
        near=self.starts[legs]+(along-starts[legs])[:,None]*self.axes[legs]
        self.progress=along[np.argmin(np.linalg.norm(near-position,axis=1))]
        forward=min(self.progress+self.lookahead,self.ends[-1])
        i=np.searchsorted(self.ends,forward)
        target=self.starts[i]+(forward-starts[i])*self.axes[i]
        remaining=max(self.ends[-1]-self.progress,np.linalg.norm(self.station-position))
        return target,remaining
