"""Installed surface transport based on the registered shafts and gear centers in the supplied 2024 field."""
from pathlib import Path
import hashlib,json,math,inspect
import numpy as np
p=Path(__file__).resolve().parent
fieldfolder=Path('R:/2024/six-speaker-20261118/field-source')
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
def read(f):return json.loads(f.read_bytes())
def transmission():
    f=fieldfolder/'whole-field-sliced-bounds.json'
    parts=read(f)['parts']
    def center(assembly,name):
        xs=[np.mean(r['bounds'],axis=0) for r in parts if r['path'][1:4]==['GE-24300_Briweld Field Blue','GE-24350',assembly] and r['path'][-1]==name]
        assert xs,name
        return [np.array([v[0],0.,v[2]]) for v in xs]
    top='GE-24370';bot='GE-24351'
    wheel=center(top,'GE-24389')[0];stubs=center(top,'GE-24381')
    topMotor=center(top,'am-4215 30T 20DP 500hex 375 FW')[0]
    gears=sorted(center(top,'am-4710 40T 20DP 500hex 375 FW'),key=lambda v:np.linalg.norm(v-topMotor))
    drive=min(stubs,key=lambda v:np.linalg.norm(v-gears[1]));other=max(stubs,key=lambda v:np.linalg.norm(v-gears[1]))
    assert np.linalg.norm(gears[1]-drive)<.001
    err1=abs(np.linalg.norm(gears[0]-topMotor)-(30+40)*.0254/40)
    err2=abs(np.linalg.norm(gears[0]-gears[1])-80*.0254/40)
    assert max(err1,err2)<.0005
    ds=sorted(center(bot,'GE-24357'),key=lambda v:v[0])
    pulley=[np.mean(r['bounds'],axis=0)[[0,2]] for r in parts if r['path'][1:4]==['GE-24300_Briweld Field Blue','GE-24350',top] and 'Pulley Body' in r['path'][-1]]
    assert all(min(np.linalg.norm(x-a[[0,2]]) for x in pulley)<.0005 for a in [gears[0],wheel])
    upperR=(80*.00635-2*np.linalg.norm(drive-other))/(2*math.pi)
    lowerR=[(168*.00635-2*np.linalg.norm(ds[i+1]-ds[i]))/(2*math.pi) for i in [0,2]]
    assert all(.023<r<.026 for r in [upperR,*lowerR])
    return dict(field=sha(f),wheel=wheel.tolist(),gearCenters=[v.tolist() for v in [topMotor,*gears]],meshResidualsMm=[err1*1000,err2*1000],upper=[v.tolist() for v in stubs],lower=[v.tolist() for v in ds],upperPitchRadius=upperR,lowerPitchRadius=lowerR,drawingPages=[189,206,222],motorNoLoadRpm=5310,reducer=12,topRatio=30/40,counts=[2,10,9],scope='Lower surface draws away from the opening. Upper live belt travels toward the exit on its underside; intervening meshed gears reverse the front wheel. Absolute operating polarity and speeds are assumptions until measured. Loaded RPM fraction is varied for each independent motor.')
def bind(module,lowerFrac,upperFrac):
    t=transmission();w=t['motorNoLoadRpm']/60*2*math.pi/t['reducer']
    lowerOmega=-w*lowerFrac;upperOmega=w*t['topRatio']*upperFrac
    lower=np.array(t['lower']);upper=np.array(t['upper']);wheel=np.array(t['wheel'])
    def on_loop(near,a,b,r,speed):
        # Closest pitch-centerline point identifies the two straight runs and rounded shaft regions.
        direction=b-a;v=near.copy();v[:,1]=0
        f=np.clip((v-a)@direction/(direction@direction),0.,1.);rad=v-(a+f[:,None]*direction)
        norm=np.linalg.norm(rad,axis=1)
        return speed*r*np.cross([0,1,0],rad/np.maximum(norm,1e-10)[:,None])
    def velocity(part,near):
        name=part['name']
        if name=='GE-24355':
            i=0 if part['key']=='p50' else 2
            assert part['key'] in ('p49','p50')
            return on_loop(near,*lower[i:i+2],t['lowerPitchRadius'][i//2],lowerOmega)
        if name=='80 tooth RT-25 Belt':
            return on_loop(near,*upper,t['upperPitchRadius'],upperOmega)
        if name=='Grip_Wheel-2in-MAXSpline' and 'GE-24385' in part.get('path',[]):
            return np.cross([0.,-upperOmega,0.],near-wheel)
        return 0.
    src=inspect.getsource(module.field_wrench)
    old='if part["name"] == "GE-24355":\n            relative -= args.belt_speed * np.cross(normal, [0., 1., 0.])'
    assert src.count(old)==1
    fns=dict(module.field_wrench.__globals__,surfaceVelocity=velocity)
    exec(compile(src.replace(old,'relative -= surfaceVelocity(part,nearest[touching])'),'assembled speaker counter','exec'),fns)
    module.field_wrench=fns['field_wrench']
    return t,velocity
if __name__=='__main__':
    t=transmission();(p/'transmission.json').write_text(json.dumps(t,indent=2));print(t)
