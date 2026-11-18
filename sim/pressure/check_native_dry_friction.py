"""Analytical block controls, reciprocal contact work and numerical identities."""
import hashlib,json,pathlib,time
import numpy as np
from native_dry_friction import solve_disks
rows=[]
for label,v,cap in [('sticking',[.001,-.002],.01),('sliding',[.3,-.4],.01),('zero',[.3,-.4],0.)]:
 j=np.eye(2)[None,:,:];m=np.eye(2)*2;v=np.array(v);r=solve_disks(j,m,v,np.array([cap]));speed=np.linalg.norm(v);expected=v*max(0.,1-cap/(2*speed));error=float(np.max(abs(r['velocity']-expected)));assert error<1e-12
 rows.append(dict(label=label,maximumVelocityErrorMps=error,heatJ=r['heatJ'],projectionIdentityJ=r['projectionIdentityJ'],coneExcessNs=r['maximumConeExcessNs'],iterations=r['iterations']))
# Duplication partitions the same load rather than strengthening friction.
v=np.array([.03,-.04]);one=solve_disks(np.eye(2)[None,:,:],np.eye(2)*2,v,np.array([.03]));duplicate=solve_disks(np.repeat(np.eye(2)[None,:,:],100,axis=0),np.eye(2)*2,v,np.full(100,.0003));error=float(np.max(abs(one['velocity']-duplicate['velocity'])));assert error<1e-12
out=dict(schema='dry-friction-analytical-check/1',passed=True,scriptSha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),solverSha256=hashlib.sha256(pathlib.Path('native_dry_friction.py').read_bytes()).hexdigest(),cases=rows,partitionedDuplicateVelocityErrorMps=error,acceptance=False,limits=['Analytical impulse tests only; no native or hardware calibration','Projection dissipation is a discretization diagnostic, not silently booked as physical friction heat'])
pathlib.Path('native-dry-friction-analytical.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(out,indent=2))
