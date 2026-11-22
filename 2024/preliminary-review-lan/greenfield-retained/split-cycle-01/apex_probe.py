"""Choose launch speed from the roof and aperture; record a diagnostic before replanning any route."""
from pathlib import Path
import json,sys,concurrent.futures,subprocess,math
p=Path(__file__).resolve().parent
if len(sys.argv)>1:
    label,n,target,x=sys.argv[1:];sys.argv=[__file__,label,n,'ready',target]
    script=(p/'split_fire.py').read_text().replace("exec(compile(script,'fire",'''script=script.replace("'125','1.53'", "'125','"+x+"'").replace("'split-'+policyName", "'apex-'+x+'-'+policyName")
exec(compile(script,'fire''')
    exec(compile(script,'field lane probe','exec'));sys.exit()
# Target apex ~2.28 m at X -0.20..-0.30. Side belts release the measured NOTE at ~1.00 m.
# Current outgoing R02 horizontal/vertical speeds were 4.250/5.866 at belt command 8.05 m/s.
vz=math.sqrt(2*9.80665*(2.28-.997));vx=4.250*vz/5.866
target=round(8.05*vz/5.866,2)
# At this lower launch speed the free ring reaches its apex closer to the robot.
x=round(.230+vx*vz/9.80665-.220,2)
print('command,station',target,x,flush=True)
def run(row):
    label,n=row
    with (p/f'apex-{label}-{n}.log').open('w') as f:subprocess.run([sys.executable,__file__,label,str(n),str(target),str(x)],stdout=f,stderr=f,check=True)
with concurrent.futures.ThreadPoolExecutor(4) as pool:list(pool.map(run,[('R02-center-high',96),('R01-inner',96),('dev-middle',48),('dev-center-high',96)]))

