"""Exact-law continuation of the archived fresh-rest trajectory to an index gate.

This retains physical state, energy and the absolute controller clock. It does
not import a state from another solver or qualify lifted possession.
"""
import argparse,copy,hashlib,json,math,pathlib,sys,time

sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def write(path,data):
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');temp.replace(path)

def run(a):
    sys.path.insert(0,str(a.project/'sim/pressure'))
    import run_fresh_rest_controller as fresh
    source=a.project/'sim/pressure/fresh-rest-controller-3s-evidence.json';source_bytes=source.read_bytes();report=json.loads(source_bytes)
    p,controller,pins,bus,ancestry=fresh.load()
    assert report['schema']=='pressure-fresh-rest-controller/1' and report['inputs']==pins
    assert report['startCheckpoint']==fresh.export_domain(p) and report['controllerStart']==vars(controller)
    assert report['outcome']=='HORIZON' and len(report['trace'])==3000 and report['stepS']==.001
    for i,event in enumerate(report['events']):
        assert abs(event['timeS']-i*.01)<1e-10
        assert controller.update(event['timeS'],event['observation'])==event['command']
    assert vars(controller)==report['controllerEnd']
    state=report['endCheckpoint']['state'];p.drive.state=copy.deepcopy(state['chassis']);p.note=copy.deepcopy(state['note']);p.energy=copy.deepcopy(state['ledger']);p.t=state['timeS'];p.initial_energy=state['initialEnergyJ']
    assert fresh.export_domain(p)==report['endCheckpoint']
    command=copy.deepcopy(report['heldCommand']);next_tick=report['nextControllerTimeS'];bus=report['trace'][-1]['busV']
    assert abs(next_tick-3.)<1e-10 and command==report['events'][-1]['command']
    assert not a.output.exists();a.output.mkdir(parents=True)
    contract=json.loads(a.contract.read_bytes());assert contract['dtS']==a.dt and contract['maximumAbsoluteTimeS']==6.
    initial=fresh.export_domain(p);rows=[];events=[];dwell_start=None;index=None;outcome='HORIZON';reason=None;start=time.perf_counter()
    def snapshot():return copy.deepcopy(dict(chassis=p.drive.state,note=p.note,ledger=p.energy,timeS=p.t,initialEnergyJ=p.initial_energy))
    def eligible(o):return .180<=o['relativeX']<=.190 and abs(o['relativeY'])<=.001 and math.hypot(o['relativeVx'],o['relativeVy'])<=.003 and math.hypot(p.drive.state['vx'],p.drive.state['vy'])<=.003
    def save(partial):
        assert source.read_bytes()==source_bytes and all(sha(a.project/f)==h for f,h in pins.items())
        data=dict(schema='pressure-continuous-index/1',partial=partial,acceptance=False,contractSha256=sha(a.contract),scriptSha256=sha(pathlib.Path(__file__)),sourceSha256=hashlib.sha256(source_bytes).hexdigest(),inputs=pins,startCheckpoint=initial,endCheckpoint=fresh.export_domain(p),controllerEnd=copy.deepcopy(vars(controller)),heldCommand=copy.deepcopy(command),nextControllerTimeS=next_tick,index=index,dwellStartS=dwell_start,outcome=outcome,reason=reason,stepS=a.dt,trace=rows,events=events,runtimeSeconds=time.perf_counter()-start,limits=['Exact prior pressure-law and geometry lineage; not registration to the newer relieved native archive','Ground-supported planar indexing only, not lifted retention or acquisition-to-launch continuity','Thresholds come from the existing loading contract; no controller/material retuning','Archived startup is 1 ms; any continuation timestep refinement is local','Uncalibrated material and drivetrain; no native full-assembly or hardware acceptance'])
        write(a.output/('partial.json' if partial else 'receipt.json'),data)
    for i in range(round((6.-p.t)/a.dt)):
        observation=fresh.prior.observe(p.t,p.drive.state,p.note,bus)
        if eligible(observation):
            if dwell_start is None:dwell_start=p.t
        else:dwell_start=None
        if fresh.controller_due(p.t,next_tick):
            if dwell_start is not None and p.t-dwell_start>=.1-1e-10:
                index=dict(timeS=p.t,observation=observation,dwellStartS=dwell_start,physicalState=snapshot(),nextControllerTimeS=next_tick);outcome='ATTAINED_GROUND_INDEX';break
            command=controller.update(p.t,observation);events.append(dict(timeS=p.t,observation=observation,command=copy.deepcopy(command)));next_tick+=.01
        before=snapshot()
        try:row=p.step(a.dt,command['driveDemands'],command['pinchDemands'],[p.drive.c['massKg']*9.80665/2]*2,modes=command['modes'])
        except (ValueError,fresh.prior.DomainExit) as e:
            assert snapshot()==before;outcome='UNKNOWN_DOMAIN';reason=str(e);break
        row['command']=copy.deepcopy(command);rows.append(row);bus=row['busV']
        if (i+1)%100==0:
            save(True);print(json.dumps({'timeS':p.t,'relativeX':row['relativeRobotXY'][0],'relativeVx':p.note['v'][0]-p.drive.state['vx'],'chassisVx':p.drive.state['vx'],'dwellStartS':dwell_start,'runtimeSeconds':time.perf_counter()-start}),flush=True)
    save(False);print(json.dumps({'outcome':outcome,'reason':reason,'timeS':p.t,'indexTimeS':index['timeS'] if index else None,'runtimeSeconds':time.perf_counter()-start}),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['project','contract','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
    p.add_argument('--dt',type=float,default=.001);a=p.parse_args();assert a.dt in [.001,.0005];run(a)
