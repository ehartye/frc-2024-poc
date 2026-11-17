"""Nominal component law only. Stops, guide friction and NOTE contact solved externally."""
import math,numbers

def spring_state(u_m, seat_adjustment_m=0, rate_fraction=0, free_length_m=.00953):
 if not all(isinstance(v,numbers.Real) and not isinstance(v,bool) and math.isfinite(v) for v in [u_m,seat_adjustment_m,rate_fraction,free_length_m]):raise ValueError('nonfinite or non-real input')
 if free_length_m<=.00366*1.05:raise ValueError('free length must exceed conservative solid height; tolerance not inferred')
 if not 0<=u_m<=.0015 or abs(seat_adjustment_m)>.001 or abs(rate_fraction)>.1:raise ValueError('outside studied parameter domain')
 length=.007+u_m-seat_adjustment_m
 solid_upper=.00366*1.05
 if length<=solid_upper:raise ValueError('possible coil bind')
 compression=max(0,free_length_m-length);k=2890*(1+rate_fraction)
 force=2*k*compression
 try:energy=k*compression**2
 except OverflowError:raise ValueError('nonfinite derived energy')
 if not math.isfinite(force) or not math.isfinite(energy):raise ValueError('nonfinite derived force or energy')
 return {'installedLengthM':length,'pairInwardForceN':force,'storedEnergyJ':energy,'atInwardStop':u_m==.0015,'atOutwardStop':u_m==0,'stopReactionN':None,'noteNormalForceN':None,'rateFraction':rate_fraction,'freeLengthToleranceUnknown':True}
