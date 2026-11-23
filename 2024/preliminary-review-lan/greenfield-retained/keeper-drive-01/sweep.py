from pathlib import Path
p=Path(__file__).resolve().parent;root=p.parent
prior=root/'frame-pocket-01/sweep.py'
src=prior.read_text(encoding='utf8').replace("src=p/'frame-and-feeder.step'","src=p/'keeper-drive.step'").replace("root/'split-cycle-01/package.json'","p/'package.json'")
needle="exec(compile(src,'revised package sweep','exec'))"
insert="""src=src.replace("d['step']=receipt['step'];park=","roles.update(receipt['rig']);d['step']=receipt['step'];park=")
src=src.replace("mm=np.array([","M['keeper']=arm@mat(0,0,0,tr)@ip;mm=np.array([")
src=src.replace("for iq,(q,f,dep,mode) in enumerate(poses):","poses=[(*a,0.) for a in poses]+[(q,0.,0.,'gate',tr) for q in [75.386,82.,91.,115.] for tr in [0.,11.,22.,33.,44.]]\\n    for iq,(q,f,dep,mode,tr) in enumerate(poses):")
src=src.replace("q,f,dep,mode,narrow","q,f,dep,mode,tr,narrow")
"""
assert src.count(needle)==1
exec(compile(src.replace(needle,insert+needle),'full gate and drive sweep','exec'))
