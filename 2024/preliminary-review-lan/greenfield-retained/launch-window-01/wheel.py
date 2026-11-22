"""Single-factor field correction. Match the saved carry and release; only the ring-counter wheels turn."""
from pathlib import Path
import sys,inspect,json
p=Path(__file__).resolve().parent
speed=float(sys.argv.pop());sys.path[:0]=[str(Path('C:/Users/ehart/Documents/FRC Native/2024/six-speaker-20261118/field-query-libs')),'C:/Users/ehart/repos/frc-2024-poc/sim/pressure']
import run_speaker_field as field
source=inspect.getsource(field.field_wrench)
needle='        normal_speed = (relative*normal).sum(axis=1)'
assert source.count(needle)==1
# The upper wheel's lower working face carries a NOTE toward negative field X.
source=source.replace(needle,"        if part['name']=='Grip_Wheel-2in-MAXSpline' and 'GE-24385' in part.get('path',[]):\n            radial=nearest[touching]-np.array([-.404481,0.,1.50961]);radial[:,1]=0.\n            relative-=np.cross([0.,wheelSpeed/.0254,0.],radial)\n"+needle)
ctx=dict(field.field_wrench.__globals__,wheelSpeed=speed)
exec(compile(source,'velocity on exact counter wheel patches','exec'),ctx);field.field_wrench=ctx['field_wrench']
base=(p.parent/'split-field-02/fire.py').read_text()
# Change only the output label. Keep the incoming file key and shot solver identical.
key,n,policy,setting=sys.argv[1:];sys.argv[3]=policy+'wheel'+str(speed)
exec(compile(base,'unchanged installed transfer','exec'))
