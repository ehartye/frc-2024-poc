"""Hand the solved moving NOTE to the calibrated field/shot model at the same station."""
from pathlib import Path
import json,sys,numpy as np,hashlib
p=Path(__file__).resolve().parent
n=sys.argv[1];previous=p/f'cam{n}.json';a=json.loads(previous.read_bytes())
assert a['carryResult']['retainedThroughout'] and abs(np.rad2deg(a['endFlapRad'])-90)<1
old=(p.parent/'bounded-risk-01/power.py').read_text()
extra="""
sub('    phi=np.arange', '''    prior=np.array(incoming['endPieceQpos'])
    d.qpos[:]=prior;d.qpos[:3]+=np.array([robotX-incoming['approachM'],robotY,0])
    d.qvel[:]=incoming['endPieceQvel'];mj.mj_forward(model,d)
    initialState=d.qpos.tolist();initialVelocity=d.qvel.tolist()
    phi=np.arange''')
sub('result=dict(policy=', 'result=dict(initialState=initialState,initialVelocity=initialVelocity,sourceCarry=priorHash,policy=')
sub('ancillary=118. if acceleration else 25.', 'ancillary=120. if acceleration else 27.')
env.update(incoming=incoming,priorHash=priorHash)
"""
old=old.replace('env.update(policy=',extra+'\nenv.update(policy=')
sys.argv=[__file__,'fire'+n,'armed',n]
incoming=a;priorHash=hashlib.sha256(previous.read_bytes()).hexdigest()
exec(compile(old,'same-note-field-replay','exec'))

