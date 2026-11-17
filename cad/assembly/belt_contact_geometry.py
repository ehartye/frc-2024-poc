"""Pure-math geometry of the authored smooth side-belt capsule annulus. No force law."""
import math

def geometry(config):
 p=config['pinch'];pitch_radius=p['teeth']*p['pitch']/(2*math.pi);centers=(p['beltTeethHypothesis']-p['teeth'])*p['pitch']/2
 if centers<=0 or pitch_radius<=1 or p['beltWidth']<=0 or p['beltRadialThicknessHypothesis']!=2:raise ValueError('unsupported authored belt dimensions')
 return {'rearX':p['beltCenterX']-centers/2,'frontX':p['beltCenterX']+centers/2,'axisYAbs':p['innerFaceYAbs']+pitch_radius+1,'outerRadius':pitch_radius+1,'innerRadius':pitch_radius-1,'halfHeight':p['beltWidth']/2,'floorOrigin':[config['stageCenterX'],0,config['floorCenterZ']]}

def query(point,side,g):
 if side not in [-1,1] or len(point)!=3 or not all(math.isfinite(v) for v in point):raise ValueError('finite point and signed side required')
 x,y,z=point;axisx=min(g['frontX'],max(g['rearX'],x));axisy=side*g['axisYAbs'];dx=x-axisx;dy=y-axisy;radius=math.hypot(dx,dy)
 # Exact signed distance to an annular capsule extruded over finite belt height.
 planar=max(radius-g['outerRadius'],g['innerRadius']-radius);height=abs(z)-g['halfHeight'];sdf=math.hypot(max(planar,0),max(height,0))+min(max(planar,height),0)
 if radius==0:return {'signedDistanceToMaterialMm':sdf,'outerSidewall':None}
 nx,ny=dx/radius,dy/radius;normal=[nx,ny,0];nearest=[axisx+g['outerRadius']*nx,axisy+g['outerRadius']*ny,min(g['halfHeight'],max(-g['halfHeight'],z))]
 return {'signedDistanceToMaterialMm':sdf,'outerSidewall':{'pointCradleMm':nearest,'outwardNormal':[nx,ny,0],'acquisitionUnitTangent':[side*ny,-side*nx,0],'handoffUnitTangent':[-side*ny,side*nx,0],'signedRadialGapMm':radius-g['outerRadius'],'withinAxialHeight':height<=0,'noteFacingHalf':side*ny<=0,'feature':'straight' if g['rearX']<=x<=g['frontX'] else 'end-wrap','warning':'sidewall radial gap is not full-solid penetration when height is exceeded or point lies in loop cavity; no contact force inferred'}}

def inner_face_at(x,z,side,g):
 """NOTE-facing outer wall at a longitudinal coordinate, including finite round ends."""
 if side not in [-1,1] or not all(math.isfinite(v) for v in [x,z]):raise ValueError('invalid query')
 axisx=min(g['frontX'],max(g['rearX'],x));dx=x-axisx
 if abs(dx)>g['outerRadius'] or abs(z)>g['halfHeight']:return None
 y=side*(g['axisYAbs']-math.sqrt(max(0,g['outerRadius']**2-dx**2)))
 return query([x,y,z],side,g)['outerSidewall']

def coplanar_note_contact(center,major_radius,tube_radius,side,g):
 """Exact nearest outer-wall geometry for a strictly outside, coplanar major ring.

 The single resulting contact is geometric only, not a distributed force model.
 """
 if len(center)!=3 or not all(math.isfinite(v) for v in center) or major_radius<=0 or tube_radius<=0:raise ValueError('invalid NOTE geometry')
 if abs(center[2])>1e-9:return {'supported':False,'reason':'NOTE major ring is not at belt midheight'}
 x,y,z=center;axisx=min(g['frontX'],max(g['rearX'],x));axisy=side*g['axisYAbs'];d=math.hypot(x-axisx,y-axisy)
 if d<=major_radius+g['outerRadius']:return {'supported':False,'reason':'major ring reaches capsule material; outer-wall formula unavailable'}
 ux,uy=(x-axisx)/d,(y-axisy)/d;ring=[x-major_radius*ux,y-major_radius*uy,0];wall=query(ring,side,g)['outerSidewall']
 if not wall or not wall['noteFacingHalf'] or not wall['withinAxialHeight']:return {'supported':False,'reason':'unsupported belt feature'}
 return {'supported':True,'penetrationMm':major_radius+tube_radius+g['outerRadius']-d,'ringCenterCradleMm':ring,'surfacePointCradleMm':wall['pointCradleMm'],'normalOnNote':wall['outwardNormal'],'acquisitionUnitTangent':wall['acquisitionUnitTangent'],'handoffUnitTangent':wall['handoffUnitTangent'],'contactLeverFromNoteCenterMm':[wall['pointCradleMm'][i]-center[i] for i in range(3)],'feature':wall['feature'],'forceLaw':None}
