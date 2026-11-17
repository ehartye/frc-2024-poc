"""Quasistatic shared DC supply for pure subsystem electrical demand queries.
Not a battery transient, controller topology, thermal or wiring validation.
"""
import math
class SupplyDomainError(ValueError):pass

def finite(value,name,minimum=None):
 if type(value) not in (int,float) or not math.isfinite(value) or minimum is not None and value<minimum:raise ValueError(name)
 return value

def solve_supply(ports,open_circuit_v,resistance_ohm,minimum_v=.1,maximum_v=16.,allow_battery_regeneration=False):
 """ports: named pure callables(busV)->{busPowerW,...}; no state integration here.
 Select highest positive-slope root of V+R*P(V)/V-Voc=0.
 Lower unstable constant-power roots are reported, never selected silently.
 """
 finite(open_circuit_v,'open circuit',.1);finite(resistance_ohm,'resistance',0);finite(minimum_v,'minimum voltage',.001);finite(maximum_v,'maximum voltage',minimum_v)
 if maximum_v<=minimum_v or not minimum_v<=open_circuit_v<=maximum_v or type(allow_battery_regeneration)is not bool:raise ValueError('supply domain')
 if not isinstance(ports,dict) or not all(isinstance(k,str) and callable(v) for k,v in ports.items()):raise ValueError('named pure ports')
 def query(v):
  outputs={k:f(v) for k,f in ports.items()}
  for k,o in outputs.items():finite(o['busPowerW'],k+' power')
  power=sum(o['busPowerW'] for o in outputs.values());current=power/v
  return v+resistance_ohm*current-open_circuit_v,power,current,outputs
 roots=[]
 if resistance_ohm==0:roots=[open_circuit_v]
 else:
  grid=[minimum_v+(maximum_v-minimum_v)*i/512 for i in range(513)]
  prior_v=grid[0];prior=query(prior_v)[0]
  for v in grid[1:]:
   value=query(v)[0]
   if abs(prior)<1e-11:roots.append(prior_v)
   if prior*value<0:
    lo,hi=prior_v,v;flo=prior
    for _ in range(60):
     mid=(lo+hi)/2;fm=query(mid)[0]
     if flo*fm<=0:hi=mid
     else:lo=mid;flo=fm
    roots.append((lo+hi)/2)
   prior_v,prior=v,value
  if abs(prior)<1e-11:roots.append(prior_v)
 roots=sorted({round(r,11) for r in roots});stable=[]
 for v in roots:
  eps=max(1e-6,v*1e-6);lo=max(minimum_v,v-eps);hi=min(maximum_v,v+eps)
  slope=(query(hi)[0]-query(lo)[0])/(hi-lo)
  if slope>0:stable.append(v)
 if not stable:raise SupplyDomainError('No supported positive-slope supply equilibrium')
 if len(stable)>1:raise SupplyDomainError('Multiple positive-slope equilibria require a transient model')
 v=stable[0];residual,power,current,outputs=query(v)
 if current< -1e-9 and not allow_battery_regeneration:raise SupplyDomainError('Battery regeneration unmodeled')
 source_power=open_circuit_v*current;heat=resistance_ohm*current*current
 return dict(busV=v,busCurrentA=current,busPowerW=power,sourcePowerW=source_power,sourceHeatW=heat,powerResidualW=source_power-power-heat,voltageResidualV=residual,ports=outputs,allEquilibriumCandidatesV=roots,regenerating=current<0,acceptance=False)
