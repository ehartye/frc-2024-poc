"""Simultaneously integrate two original shooter/field processes with equal and opposite NOTE contacts."""
from make_generator import *
from concurrent.futures import ThreadPoolExecutor
import importlib,subprocess,math
def solve(which):
    import run_speaker_field as field_model
    from coupled_collection import pair_wrenches
    from speaker_score_observer import shadow_margin,beam_samples
    n=48;dt=.024/n;run=read(old/'run-dev-48.json');aa,bb=run['parts'][:2]
    offset=(bb['shotStart']-aa['shotStart']);wait=round(offset/dt)
    ga,env,sa=make('dev-middle',n,offset+8)
    importlib.reload(field_model)
    gb,_,sb=make('dev-outer',n,8)
    # Bind each generator to its original patched field wrench even after constructing the other.
    a=next(ga);b=None
    params=SimpleNamespace(stiffness=1.2e6,damping=450.,friction=.45)
    target=env['target'];corners={v['key']:np.array(v['caseEndCornersM']) for v in target['sensorCaseEnds']}
    beam=[beam_samples(corners[x],corners[y]) for x,y in target['beamPairs']]
    rays=[]
    for x,y in target['beamPairs']:
        ends=[sorted(corners[k].tolist(),key=lambda v:(v[0],v[1])) for k in (x,y)]
        ends=np.array(ends)
        pts=np.concatenate((ends.mean(axis=1,keepdims=True),.8*ends+.2*ends.mean(axis=1,keepdims=True)),axis=1)
        t=np.linspace(0,1,97);rays.append(pts[0][None,:,:]*(1-t[:,None,None])+pts[1][None,:,:]*t[:,None,None])
    def open_rays(states,line):
        clear=np.ones(line.shape[1],dtype=bool)
        for st in states:
            if abs(st.center[0]-line[0,0,0])>.25:continue
            rel=line-st.center;z=rel@st.rotation[:,2];r2=np.sum(rel*rel,axis=2)-z*z
            distance=np.hypot(np.sqrt(np.maximum(0,r2))-.1524,z)
            clear &= distance.min(axis=0)>.0254+.5*np.linalg.norm(line[1]-line[0],axis=1)+1.e-7
        return bool(clear.any())
    jmax=round((offset+7)/dt);depthMax=0.;firstTouch=None;lastTouch=None;peakN=0.;counterA=None;rearmed=None;counterB=None;secondDark=None;continuity=0.;prevSecond=0.;audit=[];rows=[];capture=None
    for j in range(jmax):
        if j==wait:b=next(gb)
        t=a.t
        if j and j%round(.012/dt)==0 and t<6.999:
            continuity=max(continuity,np.max(abs(a.q-np.array(sa['trace'][j//round(.012/dt)]['q']))))
        aw=bw=None
        if b is not None:
            bj=j-wait
            if bj%round(.012/dt)==0 and firstTouch is None and b.t<6.999:
                prevSecond=max(prevSecond,np.max(abs(b.q-np.array(sb['trace'][bj//round(.012/dt)]['q']))))
            forces,depth=pair_wrenches(a,b,params)
            if depth>0:
                if firstTouch is None:firstTouch=t
                lastTouch=t;depthMax=max(depthMax,depth);peakN=max(peakN,np.linalg.norm(forces[1,:3]))
                if which=='coupled':aw,bw=forces
            if capture is None and b.release is not None:
                capture=b.release;assert capture==sb['release']
        if counterA is None and t>=sa['opening']['fullyThroughByS'] and min(shadow_margin(a.q,h) for h in beam)>=0:counterA=t
        states=[a] if b is None else [a,b]
        if counterA is not None and rearmed is None and all(open_rays(states,h) for h in rays):rearmed=t
        if b is not None and b.t>=sb['opening']['fullyThroughByS'] and min(shadow_margin(b.q,h) for h in beam)>=0:
            if secondDark is None:secondDark=t
            if rearmed is not None and counterB is None:counterB=t
        if j%round(.025/dt)==0:
            rows.append([round(t,6),a.q.tolist(),None if b is None else b.q.tolist()])
        a=ga.send(aw)
        if b is not None:b=gb.send(bw)
    assert continuity<1.e-8 and prevSecond<1.e-8,(continuity,prevSecond)
    assert abs(counterA-sa['both'])<1.e-8
    out=dict(mode=which,source=[aa['shot'],bb['shot']],coupledForceCode=sha(Path(sys.modules['coupled_collection'].__file__)),world='Two free NOTES, same stationary speaker and timed releases. Original floor, traction, field surface velocity, sensor and incoming states retained.',startRoundingErrorS=wait*dt-offset,verifiedPreviousNominal=continuity,verifiedNextBeforeContact=prevSecond,firstCountS=counterA,sharedClearRayBothS=rearmed,individualSecondDarkS=secondDark,distinctSecondEventS=counterB,firstPairContactS=firstTouch,lastPairContactS=lastTouch,peakPairN=peakN,peakIndentationM=depthMax,checkedThroughS=a.t,trace=rows)
    (p/f'pair-{which}.json').write_text(json.dumps(out,separators=(',',':')))
    print({k:v for k,v in out.items() if k!='trace'},flush=True);ga.close();gb.close()
if __name__=='__main__':
    if len(sys.argv)>1:
        sys.path[:0]=[str(p.parent/'counter-drive-01'),'C:/Users/ehart/Documents/FRC Native/2024/six-speaker-20261118/field-query-libs','C:/Users/ehart/repos/frc-2024-poc/sim/pressure']
        solve(sys.argv[1])
    else:
        def run_case(k):
            with (p/f'pair-{k}.log').open('w') as f:subprocess.run([sys.executable,__file__,k],stdout=f,stderr=f,check=True)
        with ThreadPoolExecutor(max_workers=2) as ex:list(ex.map(run_case,['coupled','independent']))

