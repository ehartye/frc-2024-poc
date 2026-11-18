"""Fixed continuous 2024 bumper crosssection candidate; not rules acceptance."""
import math
import a2_primitives as h
CONFIG={'backingThicknessMm':19.05,'backingHeightMm':127,'noodleDiameterMm':63.5,'topMm':185.5,'clothAllowanceMm':1.5,'maximumTopMm':190.5,'source':'https://firstfrc.blob.core.windows.net/frc2024/Manual/HTML/2024GameManual.htm','sections':['R401','R402','R408','R410'],'status':'Nominal informed crosssection; cloth,corner construction,fasteners and backing supports unqualified'}
def underpass_margin(pitch_deg,c=CONFIG):
 # Lowest floor-tangent torus at fixed attitude: vertical full extent2*(R*sin(pitch)+r).
 piece_height=2*(152.4*abs(math.sin(math.radians(pitch_deg)))+25.4)
 return c['topMm']-c['backingHeightMm']-c['clothAllowanceMm']-piece_height

def build(length,width,c=CONFIG):
 parts=[];t=c['backingThicknessMm'];height=c['backingHeightMm'];r=c['noodleDiameterMm']/2;bottom=c['topMm']-height
 for side in [-1,1]:
  y=side*width/2-(t if side<0 else 0)
  parts.append((f'fixed bumper side backing {side}',h.box(-length/2-t,y,bottom,length+2*t,t,height),'bumper'))
  cy=side*(width/2+t+r)
  for row in [0,1]:parts.append((f'fixed bumper side noodle {side} {row}',h.cyl((-length/2-t,cy,bottom+r+2*r*row),(1,0,0),r,length+2*t),'bumper'))
 for side in [-1,1]:
  x=side*length/2-(t if side<0 else 0)
  parts.append((f'fixed bumper end backing {side}',h.box(x,-width/2,bottom,t,width,height),'bumper'))
  cx=side*(length/2+t+r)
  for row in [0,1]:parts.append((f'fixed bumper end noodle {side} {row}',h.cyl((cx,-width/2-t,bottom+r+2*r*row),(0,1,0),r,width+2*t),'bumper'))
 parts.append(('original rigid front backing support proposal',h.box(length/2-3.175,-width/2,bottom,3.175,width,height),'support'))
 return parts
