"""Deploy a finite-stroke dog when the sector opens; keep hoist power until axial stroke completes."""
from pathlib import Path
import sys
from mass_inputs import bodytext,ledger
p=Path(__file__).resolve().parent
label,dt,cg,present,rate,cutoff=sys.argv[1:];rate=float(rate);cutoff=float(cutoff)
src=(p.parent/'catch-transfer-01/trial.py').read_text()
src=src.replace('exec(compile(src,', '''src=src.replace('cut=4.',f'cut={cutoff}').replace('int(9/dt)',f'int(({cutoff}+5.)/dt)')
src=src.replace('mass="62.56" pos="{xCG} 0 .30" diaginertia="4.5 4.5 6.5"',bodytext('frame',62.56,[float(sys.argv[5]),0,.30],[4.5,4.5,6.5]))
src=src.replace('mass="6" pos=".28 0 .05" diaginertia=".13 .30 .32"',bodytext('arm',6,[.28,0,.05],[.13,.30,.32]))
src=src.replace('activeMassKg=68.56','activeMassKg='+str(ledger['totalRunningHighKg']))
exec(compile(src,''')
a=src.index('    if present and not armed');b=src.index('    sink=',a)
src=src[:a]+'''    # A spring or low-power drive must deliver this stroke under the installed side-load envelope.
    if request is None and clearTime>=.040: request=t
    if present and request is not None and t>=request+.030 and angle>=np.deg2rad(159.5):
        depth=min(10.8,depth+rate*dt)
    if not armed and depth>=10.8-1.e-8:armed=True;armAt=t
    if armed:torque*=1.-.98*np.clip((t-armAt)/.45,0,1)
'''+src[b:]
src=src.replace('armed=False;clearTime','request=None;depth=0.;armed=False;clearTime')
src=src.replace('confirmed=confirmAt is not None','strokeMm=depth,confirmed=confirmAt is not None')
src=src.replace('r.update(sourceHash','r.update(commandTime=request,assumedStrokeRate=rate,cutTime=cutoff,confirmMarginS=cutoff-confirmAt if confirmAt is not None else None,sourceHash')
sys.argv=[__file__,label,dt,cg,present,'350']
exec(compile(src,'finite-stroke catch sequence','exec'))

