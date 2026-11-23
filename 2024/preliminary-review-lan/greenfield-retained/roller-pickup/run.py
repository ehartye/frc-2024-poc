from pathlib import Path
import sys, json, hashlib
here=Path(__file__).resolve().parent
parent=here.parent/'pickup-lip'
sys.path.insert(0,str(parent))
label,lip,enabled,n,offset,height,mu,mode=sys.argv[1:]
sys.argv=['pickup_trial.py',lip,enabled,n,'0.36',offset]
source=(parent/'pickup_trial.py').read_text()
source=source[:source.index("fn='trial-'")]
source=source.replace('exec(src);r=run','''Z+=float(height)/1000
if mode=='drum-only':
    src=src.replace('spacing+speed*t','spacing+0*t').replace('vals[:,k+3]*speed','vals[:,k+3]*0')
elif mode=='stopped-drum':
    src=src.replace('-speed/.012','0.')
elif mode!='all': raise ValueError(mode)
exec(src);r=run''').replace('len(sys.argv)>5 else 0)','len(sys.argv)>5 else 0,float(grip))')
env=dict(__file__=str(parent/'pickup_trial.py'),height=height,grip=mu,mode=mode)
exec(source,env)
r=env['r'];r['minimumSolidClearanceMm']=env['np'].min(env['checks'],axis=0).tolist();r['checkedPoses']=len(env['checks'])
r.update(label=label,relativeIntakeHeightChangeMm=float(height),driveMode=mode)
(here/(label+'.json')).write_text(json.dumps(r,separators=(',',':')))
print(label,r['seated'],r['seatResidenceS'],r['firstContactS'],r['sideBeltTakeoverS'],flush=True)

