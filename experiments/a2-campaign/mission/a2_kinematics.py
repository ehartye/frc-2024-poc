"""Shared, validated A2 prescribed kinematics. No actuator or retention physics."""
import math

def validate(i):
 required=['pivot','armLength','minimumLength','floorAngleDeg','stowAngleDeg','rollerRadius','rollerWidth','armSection','maximumHeight','designClearance','maximumForwardCenter','retentionRollerOffset','retentionRollerRadius','shaftRadius','shaftWidth','armYOrigins','sleeveSection','sleeveLength','linkYOrigins','linkSection','linkNormalOffset']
 for key in required:
  v=i[key];values=v if isinstance(v,list) else [v]
  if any(not isinstance(x,(int,float)) or isinstance(x,bool) or not math.isfinite(x) for x in values):raise ValueError('nonfinite '+key)
 for k in ['armLength','minimumLength','rollerRadius','rollerWidth','maximumHeight','retentionRollerRadius','shaftRadius','shaftWidth','sleeveLength']:
  if i[k]<=0:raise ValueError('nonpositive '+k)
 for k in ['armSection','sleeveSection','linkSection']:
  if len(i[k])!=2 or min(i[k])<=0:raise ValueError(k)
 if len(i['pivot'])!=3 or len(i['retentionRollerOffset'])!=3:raise ValueError('vector length')
 if i['minimumLength']>i['armLength'] or i['floorAngleDeg']>i['stowAngleDeg']:raise ValueError('bounds')
 if i['designClearance']<0:raise ValueError('clearance')

def pose(i,degree):
 validate(i);a=math.radians(degree);co,si=math.cos(a),math.sin(a);px,py,pz=i['pivot']
 # Circle heights are exact; positive-angle arm/link offsets contribute to the cap.
 extra=max(i['rollerRadius'],i['retentionRollerOffset'][2]+i['retentionRollerRadius'],i['shaftRadius'],abs(co)*i['armSection'][1]/2,abs(co)*i['sleeveSection'][1]/2,co*(i['linkNormalOffset']+i['linkSection'][1]),co*i['linkNormalOffset'])
 caps=[i['armLength']]
 if co>1e-12:caps.append((i['maximumForwardCenter']-px)/co)
 if si>1e-12:caps.append((i['maximumHeight']-i['designClearance']-pz-extra)/si)
 L=min(caps)
 if L<i['minimumLength']-1e-9:raise ValueError('requested pose exceeds telescope travel')
 return {'angleDeg':degree,'angleRad':a,'length':L,'tip':[px+L*co,py,pz+L*si]}

def moving_bounds(i,p):
 """Named bounds for every moving CAD part, including auxiliary rollers/links."""
 co,si=math.cos(p['angleRad']),math.sin(p['angleRad']);px,py,pz=i['pivot'];L=p['length'];tip=p['tip'];out=[]
 def rect(name,x0,x1,z0,z1,y0,width):
  pts=[(px+x*co-z*si,pz+x*si+z*co) for x in [x0,x1] for z in [z0,z1]]
  out.append({'id':name,'min':[min(x for x,z in pts),py+y0,min(z for x,z in pts)],'max':[max(x for x,z in pts),py+y0+width,max(z for x,z in pts)]})
 for n,y in enumerate(i['armYOrigins']):
  rect('arm'+str(n),0,L,-i['armSection'][1]/2,i['armSection'][1]/2,y,i['armSection'][0])
  rect('sleeve'+str(n),0,i['sleeveLength'],-i['sleeveSection'][1]/2,i['sleeveSection'][1]/2,y-(i['sleeveSection'][0]-i['armSection'][0])/2,i['sleeveSection'][0])
 for n,y in enumerate(i['linkYOrigins']):rect('link'+str(n),0,L,i['linkNormalOffset'],i['linkNormalOffset']+i['linkSection'][1],y,i['linkSection'][0])
 for name,r,w,off in [('capture',i['rollerRadius'],i['rollerWidth'],[0,0,0]),('retention',i['retentionRollerRadius'],i['rollerWidth'],i['retentionRollerOffset']),('shaft',i['shaftRadius'],i['shaftWidth'],[0,0,0])]:
  c=[tip[j]+off[j] for j in range(3)];out.append({'id':name,'min':[c[0]-r,c[1]-w/2,c[2]-r],'max':[c[0]+r,c[1]+w/2,c[2]+r]})
 return out
