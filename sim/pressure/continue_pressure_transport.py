"""Finite-effort transport attempt from an attained, unmodified ground index."""
import argparse,copy,hashlib,json,math,pathlib,sys,time
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def run(a):
    sys.path.insert(0,str(a.project/'sim/pressure'))
    import run_fresh_rest_controller as fresh
    import pressure_tiled_geometry as tiled
    fresh.c.factory.r.PressureAcquisition.prepared.__globals__['fast_geometry'].prepare=tiled.prepare
    source_bytes=a.index.read_bytes();source=json.loads(source_bytes);contract=json.loads(a.contract.read_bytes())
    assert source['partial'] is False and source['outcome']=='ATTAINED_GROUND_INDEX' and source['index'] is not None
    assert source['cullingSourceSha256']==sha(pathlib.Path(tiled.__file__))
    p,controller,pins,bus,ancestry=fresh.load();assert source['inputs']==pins
    s=source['endCheckpoint']['state'];p.drive.state=copy.deepcopy(s['chassis']);p.note=copy.deepcopy(s['note']);p.energy=copy.deepcopy(s['ledger']);p.t=s['timeS'];p.initial_energy=s['initialEnergyJ']
    initial=fresh.export_domain(p);assert initial==source['endCheckpoint'] and s==source['index']['physicalState']
    assert abs(p.t-source['nextControllerTimeS'])<1e-10
    assert not a.output.exists();a.output.mkdir(parents=True)
    bus=source['trace'][-1]['busV'];next_tick=source['nextControllerTimeS'];start_time=p.t;start_x=p.drive.state['x'];start_note=copy.deepcopy(p.note['p']);events=[];rows=[];command=None;outcome='HORIZON';reason=None;start=time.perf_counter()
    def snapshot():return copy.deepcopy(dict(chassis=p.drive.state,note=p.note,ledger=p.energy,timeS=p.t,initialEnergyJ=p.initial_energy))
    def update(o):
        assert abs(o['relativeY'])<=.001 and abs(o['yawRate'])<=.01
        desired=contract['requestedChassisSpeedMps'];voltage=p.drive.c['motorK']*p.drive.c['gearRatio']/p.drive.c['wheelRadiusM']*desired+2*(desired-o['forwardSpeed'])
        dd=[];modes=[]
        for wheel in [o['wheelLeft'],o['wheelRight']]:
            emf=p.drive.c['motorK']*p.drive.c['gearRatio']*wheel
            if voltage<emf+.015:dd.append(0.);modes.append('brake')
            else:dd.append(min(.2,max(0.,voltage/o['busV'])));modes.append('drive')
        pinch=min(.025,max(0.,(o['relativeX']-.185)*.15))
        return dict(driveDemands=dd,pinchDemands=[pinch,pinch],modes=modes,desiredSpeed=desired)
    for i in range(round(contract['maximumDurationS']/a.dt)):
        if fresh.controller_due(p.t,next_tick):
            o=fresh.prior.observe(p.t,p.drive.state,p.note,bus)
            try:command=update(o)
            except AssertionError:outcome='CONTROLLER_DOMAIN_EXIT';reason='straight-controller envelope';break
            events.append(dict(timeS=p.t,observation=o,command=copy.deepcopy(command)));next_tick+=.01
        before=snapshot()
        try:row=p.step(a.dt,command['driveDemands'],command['pinchDemands'],[p.drive.c['massKg']*9.80665/2]*2,modes=command['modes'])
        except (ValueError,fresh.prior.DomainExit) as e:
            assert snapshot()==before;outcome='UNKNOWN_DOMAIN';reason=str(e);break
        row['command']=copy.deepcopy(command);o=fresh.prior.observe(p.t,p.drive.state,p.note,row['busV']);row['relativePlanarSpeedMps']=math.hypot(o['relativeVx'],o['relativeVy']);rows.append(row);bus=row['busV']
        if not (.180<=o['relativeX']<=.190 and abs(o['relativeY'])<=.001 and row['relativePlanarSpeedMps']<=.010):outcome='TRANSPORT_CORRIDOR_EXIT';reason='predeclared position/lateral/relative-speed criterion';break
    assert a.index.read_bytes()==source_bytes and all(sha(a.project/f)==h for f,h in pins.items())
    result=dict(schema='pressure-continuous-transport/1',partial=False,acceptance=False,scriptSha256=sha(pathlib.Path(__file__)),contractSha256=sha(a.contract),sourceIndexSha256=sha(a.index),cullingSourceSha256=sha(pathlib.Path(tiled.__file__)),inputs=pins,startCheckpoint=initial,endCheckpoint=fresh.export_domain(p),stepS=a.dt,startTimeS=start_time,endTimeS=p.t,heldCommand=command,nextControllerTimeS=next_tick,events=events,trace=rows,outcome=outcome,reason=reason,chassisTravelM=p.drive.state['x']-start_x,noteWorldTravelM=p.note['p'][0]-start_note[0],maximumRelativeSpeedMps=max([r['relativePlanarSpeedMps'] for r in rows],default=0.),runtimeSeconds=time.perf_counter()-start,limits=['Actual attained ground index and unchanged physical state/ledger/clock at transition','New predeclared transport controller uses only existing positive pinch drive or coast','First corridor exit is a model observation; full-startup refinement and native registration remain incomplete','No lift, launcher, hardware validation, or completed loading-contract claim'])
    (a.output/'receipt.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({k:result[k] for k in ['outcome','startTimeS','endTimeS','stepS','chassisTravelM','noteWorldTravelM','maximumRelativeSpeedMps','runtimeSeconds']}))
if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['project','index','contract','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
    p.add_argument('--dt',type=float,default=.001);a=p.parse_args();assert a.dt in [.001,.0005,.00025];run(a)
