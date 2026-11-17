"""Wider positive-side support candidate; does not resolve source spline/retention."""
import servo_opening_fasteners as fast
s=fast.s;h=fast.h;old=fast.old

def local_parts(theta,travel_mm=None):
 parts=[(n,p,k) for n,p,k in fast.local_parts(theta,travel_mm) if n!='relocated servo original support']
 stem=h.box(62,208,116,16,16,91)
 arm=h.box(62,208,199,57,16,8)
 parts.append(('wide original servo support',s.fuse(stem,arm),'original-stock'))
 return parts
