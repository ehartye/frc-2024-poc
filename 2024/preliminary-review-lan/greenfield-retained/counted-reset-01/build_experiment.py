from pathlib import Path
p=Path(__file__).resolve().parent;src=(p.parent/'bounded-lift-01/mission.py').read_text(encoding='utf8')
def sub(a,b):
    global src
    assert src.count(a)==1,a
    src=src.replace(a,b)
sub('from pathlib import Path','from pathlib import Path\nimport os\nfrom empty_return import transport')
sub('def call(script,args,log):','def call(script,args,log,seed=None):')
sub('cwd=p,stdout=out','env=({**os.environ,"CYCLE_STATE":str(seed)} if seed else None),cwd=p,stdout=out')
a=src.index('        if not a.exists()');b=src.index('        travel=',a)
get=src[a:b]
src=src[:a]+src[b:]
sub('    for i,leg in enumerate', '    prev=None;previousShotStart=0.;previousShotHash=None\n    for i,leg in enumerate')
sub('        # First setup', '''        offset=[float(row.get("robot_x_mm",0))*.001,float(row.get("robot_y_mm",0))*.001] if i==0 else [0.,0.]
        priorWait=0 if i==0 else depart-previousShotStart-4.8
        initialYaw=leg['yaw']+(math.radians(float(row.get("robot_heading_deg",0))) if i==0 else 0.)
        seed,emptyMotion=transport(leg,travel,initialYaw,offset,prev,priorWait,n)
        seedFile=p/f'initial-{tag}-{n}.json';emptyFile=p/f'empty-{tag}-{n}.json'
        save(seedFile,seed);save(emptyFile,dict(parentShot=previousShotHash,**emptyMotion))
'''+get.replace("a.with_suffix('.log'))","a.with_suffix('.log'),seedFile)").replace("        c=read(a);assert", "        c=read(a);assert c['cycleStart']==seed;assert")+'''        # First setup''')
sub("carry=sha(a),gateTime=", "carry=sha(a),initial=sha(seedFile),emptyMotion=sha(emptyFile),priorShot=previousShotHash,stopOvertravelDeg=math.degrees(c['stopViolationRad']),gateTime=")
sub("f'split-outerlane-{tag}","f'split-tail_.25_0-{tag}")
sub("call('lane.py',[tag,str(n)]","call('counter_tail.py',[tag,str(n),'.25','0']")
sub("            dur=s['both']", "            prev=(c,s);previousShotStart=shotStart;previousShotHash=sha(b)\n            dur=s['both']")
sub("'carry.py','shot.py','lane.py','mission.py'","'carry.py','empty_return.py','counter_tail.py','shot.py','lane.py','mission.py'")
sub('Belt initialization at each pickup, chassis accuracy, loaded voltage, physical field transport calibration and active feedback require separate review.', 'Empty flap and belt state carried between pieces; loading and field counter are contact simulated. Measured friction, chassis tracking and target counter speed remain uncalibrated.')
sub('max_workers=3','max_workers=4')
(p/'mission.py').write_text(src,encoding='utf8')

