"""Nominal component law only. Stops, guide friction and NOTE contact solved externally."""
import math

def spring_state(u_m, seat_adjustment_m=0, rate_fraction=0, free_length_m=.00953):
 if not all(math.isfinite(v) for v in [u_m,seat_adjustment_m,rate_fraction,free_length_m]):raise ValueError('nonfinite')
 if not 0<=u_m<=.0015 or abs(seat_adjustment_m)>.001 or abs(rate_fraction)>.1:raise ValueError('outside studied parameter domain')
 length=.007+u_m-seat_adjustment_m
 solid_upper=.00366*1.05
 if length<=solid_upper:raise ValueError('possible coil bind')
 compression=max(0,free_length_m-length);k=2890*(1+rate_fraction)
 return {'installedLengthM':length,'pairInwardForceN':2*k*compression,'storedEnergyJ':k*compression**2,'atInwardStop':u_m==.0015,'atOutwardStop':u_m==0,'stopReactionN':None,'noteNormalForceN':None,'rateFraction':rate_fraction,'freeLengthToleranceUnknown':True}
