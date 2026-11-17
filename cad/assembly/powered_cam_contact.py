"""Frictionless cam normal-contact port. No servo dynamics or u prescription."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class CamGeometry:
 eccentricity_m: float = .00075
 radius_m: float = .006
 center_y_m: float = .19625
 center_z_m: float = .185
 contact_x_m: float = .146
 follower_zero_y_m: float = .203

def scalar(name,value):
 if isinstance(value,bool) or not isinstance(value,(int,float)):raise ValueError(name+' must be finite real scalar')
 try:v=float(value)
 except (OverflowError,ValueError) as ex:raise ValueError(name+' not representable') from ex
 if not math.isfinite(v):raise ValueError(name+' must be finite')
 return v

def evaluate(u_m,u_rate_m_s,theta_rad,theta_rate_rad_s,*,stiffness_n_m,damping_n_s_m,geometry=CamGeometry()):
 u,ud,t,td,k,c=[scalar(n,v) for n,v in [('u',u_m),('udot',u_rate_m_s),('theta',theta_rad),('thetadot',theta_rate_rad_s),('stiffness',stiffness_n_m),('damping',damping_n_s_m)]]
 g=geometry
 if not isinstance(g,CamGeometry):raise ValueError('geometry type')
 vals=[scalar(n,getattr(g,n)) for n in g.__dataclass_fields__]
 e=g.eccentricity_m
 if e<=0 or g.radius_m<=0 or k<=0 or c<0:raise ValueError('nonpositive geometry/stiffness or negative damping')
 if not math.isclose(g.follower_zero_y_m-g.center_y_m-g.radius_m,e,rel_tol=0,abs_tol=1e-12):raise ValueError('inconsistent zero-gap geometry')
 if not 0<=u<=2*e or not 0<=t<=math.pi:raise ValueError('outside studied cassette/angle domain; state must not be reset')
 st=0.0 if t in (0,math.pi) else math.sin(t);ct=math.cos(t)
 gap=e*(1-ct)-u;dtheta=e*st;rate=dtheta*td-ud;penetration=max(0.0,-gap)
 # Explicit onset convention: no dashpot force until positive penetration.
 closing=max(0.0,-rate) if penetration>0 else 0.0
 try:
  elastic=k*penetration;dissipative=c*closing;normal=elastic+dissipative
  potential=.5*k*penetration**2;potential_rate=-elastic*rate
  heat=c*closing**2;qu=-normal;qt=normal*dtheta
  pu=qu*ud;pt=qt*td;residual=pu+pt+potential_rate+heat
 except OverflowError as ex:raise ValueError('derived overflow') from ex
 pc=(g.contact_x_m,g.center_y_m+e*ct+g.radius_m,g.center_z_m+e*st)
 pf=(g.contact_x_m,g.follower_zero_y_m-u,pc[2])
 numbers=[gap,dtheta,rate,penetration,normal,potential,potential_rate,heat,qu,qt,pu,pt,residual,*pc,*pf]
 if not all(math.isfinite(v) for v in numbers):raise ValueError('nonfinite derived quantity')
 return {'gap_m':gap,'gap_rate_m_s':rate,'gap_jacobian':(-1.0,dtheta),'penetration_m':penetration,'normal_force_n':normal,'cassette_generalized_force_n':qu,'cam_generalized_torque_nm':qt,'potential_j':potential,'potential_rate_w':potential_rate,'dissipation_w':heat,'cassette_power_w':pu,'cam_power_w':pt,'power_identity_residual_w':residual,'cam_point_local_m':pc,'follower_point_local_m':pf,'cam_force_local_n':(0.0,-normal,0.0),'follower_force_local_n':(0.0,normal,0.0),'mode':'compressed' if penetration>0 else ('touching' if gap==0 else 'separated'),'positive_latch':False}

def command_target(pulse_us,*,verified_default_270_mode,center_cam_angle_rad,direction):
 """Source PWM mapping plus explicit assembly clocking; never measured position."""
 p=scalar('pulse',pulse_us);center=scalar('clocking center',center_cam_angle_rad);sign=scalar('direction',direction)
 if verified_default_270_mode is not True:raise ValueError('servo mode unverified; continuous/custom mappings differ')
 if not 500<=p<=2500 or sign not in (-1,1):raise ValueError('pulse/direction domain')
 target=center+sign*(p-1500)*(1.5*math.pi)/2000
 if not math.isfinite(target):raise ValueError('nonfinite target')
 return {'requested_cam_angle_rad':target,'within_studied_cam_domain':0<=target<=math.pi,'actual_angle_known':False,'positive_latch':False}
