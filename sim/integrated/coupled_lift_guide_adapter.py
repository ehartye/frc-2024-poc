"""Experiment-only separation fallback; no new guide contact law."""
import math
import finite_retention_guide as original
from guide_separation import query as separation_query

def query_round_xz(center_m,pitch_rad,contract,*,y_m=0.):
 result=original.query_round_xz(center_m,pitch_rad,contract,y_m=y_m)
 if result['status']!='UNKNOWN':return result
 # Preserve archived contradictory native witnesses regardless of broadphase.
 if 'archived' in result.get('reason',''):return result
 axis=[-math.sin(pitch_rad),0.,math.cos(pitch_rad)]
 certificate=separation_query([center_m[0],y_m,center_m[1]],axis,contract)
 if not certificate['separated']:return result
 return dict(result,status='SEPARATED_FROM_ENTIRE_GUIDE',geometryDomainValid=True,penetrationM=0.,gapM=max(certificate['axisGapM']),originalStatus=result['status'],originalReason=result.get('reason'),separationCertificate=certificate,separationFallbackUsed=True)
