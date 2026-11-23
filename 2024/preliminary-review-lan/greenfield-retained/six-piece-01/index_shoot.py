"""Run a finite-current reverse indexing stroke against the existing rear piece stop before firing."""
from pathlib import Path
p=Path(__file__).resolve().parent
script=(p/'shoot.py').read_text()
patch="""
sub('    permitFor=0.', '    seatedFor=0.;maxSeatedFor=0.;indexFirst=None;indexState=None;indexLog=[];permitFor=0.')
sub('        eligible=not acceleration and forecast>=8.2', '''        axis=(center-pivot)@R[:,0]
        axialV=v@R[:,0]
        # Sensor equivalence: rear stop contact plus an optical center window and time dwell.
        seated=any(model.geom(gg).name=='backstop' for ct in d.contact for gg in [ct.geom1,ct.geom2])
        eligibleIndex=seated and abs(axis-.215)<.008 and abs(axialV)<.10
        seatedFor=seatedFor+dt if eligibleIndex else 0.
        maxSeatedFor=max(maxSeatedFor,seatedFor)
        if seatedFor>=.06 and indexFirst is None:indexFirst=t;indexState=[*q,*d.qvel]
        if step%max(1,round(.02/dt))==0 and enableT is None:indexLog.append([t,axis,axialV,seated,seatedFor])
        eligible=not acceleration and forecast>=8.2 and indexFirst is not None''')
sub('t>1.2 and enableT is None','t>2.3 and enableT is None')
sub(') if active and release is None else 0.;Fmotor=', ') if active and release is None else (float(np.clip(25*(-.38-speed),-6,2)) if not active else 0.);Fmotor=')
sub('currents.append(I*min(1.,(.0202*w+.115*I)/bus))','currents.append(max(0.,I*np.clip((.0202*w+.115*I)/bus,-1.,1.)))')
sub('if active else np.clip(F,-1.,1.)','if active else np.clip(F,-9.,9.)')
sub('result=dict(initialState=', 'result=dict(indexFirst=indexFirst,indexState=indexState,indexLog=indexLog,maxSeatDwell=maxSeatedFor,initialState=')
"""
script=script.replace("exec(compile(script,'same-field-NOTE station','exec'))","""script=script.replace("env.update(incoming=incoming,priorHash=priorHash)", indexPatch+"\\nenv.update(incoming=incoming,priorHash=priorHash)").replace("'scored-'+name","'indexed-'+name")
exec(compile(script,'indexed shot','exec'))""")
indexPatch=patch
exec(compile(script,'index sequence','exec'))

