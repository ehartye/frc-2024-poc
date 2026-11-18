"""Source-supported M3 horn attachment, candidate center retention; no fit waiver."""
import servo_opening_relocation as base
s=base.s;h=base.h;old=base.old
OFFSETS=[(-4,-6.928203230275509),(4,6.928203230275509)]
def local_parts(theta,travel_mm=None):
 parts=[(n,p,k) for n,p,k in base.local_parts(theta,travel_mm) if n!='original horn flange neck eccentric cam' and not n.startswith('horn adapter M2')]
 flange=h.cyl((133,196.25,185),(1,0,0),10,3)
 for dy,dz in OFFSETS:flange=s.cut(flange,h.cyl((132,196.25+dy,185+dz),(1,0,0),1.65,5))
 neck=h.cyl((136,196.25,185),(1,0,0),4.5,7)
 cam=h.cyl((143,197,185),(1,0,0),6,6)
 body=s.fuse(s.fuse(flange,neck),cam)
 body=s.cut(body,h.cyl((132,196.25,185),(1,0,0),3.3,18))
 parts.append(('original M3 horn adapter and bored cam',base.spin(body,theta),'original-cam'))
 for dy,dz in OFFSETS:
  shank=h.cyl((128,196.25+dy,185+dz),(1,0,0),1.5,8)
  head=h.cyl((136,196.25+dy,185+dz),(1,0,0),3.2,3)
  parts.append(('M3x8 horn screw major-envelope '+str(dy),base.spin(s.fuse(shank,head),theta),'threaded-fastener-envelope'))
 # Candidate M3x8; native geometry bounds support packaging, not thread class or torque.
 screw=s.fuse(h.cyl((125,196.25,185),(1,0,0),1.5,8),h.cyl((133,196.25,185),(1,0,0),3.2,3))
 parts.append(('candidate M3x8 center retention envelope',screw,'threaded-fastener-envelope'))
 return parts
