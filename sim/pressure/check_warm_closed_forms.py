import hashlib,json,pathlib
import numpy as np
from native_warm_friction import solve_disks
out=pathlib.Path('native-warm-closed-forms.json');assert not out.exists();cases=[]
for v,cap,expected in [([3.,4.],2.,[1.8,2.4]),([.3,-.4],2.,[0.,0.]),([3.,4.],0.,[3.,4.])]:
 for seed in [[[0.,0.]],[[100.,-300.]]]:
  r=solve_disks(np.eye(2)[None],np.eye(2),v,[cap],initial_impulse=seed);err=float(np.max(abs(r['velocity']-expected)));assert err<1e-12;cases.append(dict(velocity=v,capacity=cap,seed=seed,expected=expected,maximumDifference=err))
out.write_text(json.dumps(dict(producerSha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),solverSha256=hashlib.sha256(pathlib.Path('native_warm_friction.py').read_bytes()).hexdigest(),cases=cases,passed=True),indent=2)+'\n');print('Six independently solved 2D Coulomb cases passed.')
