"""Fresh isolated hypothesis cases, not a robot trajectory."""
import pathlib,json,hashlib,math
from dataclasses import asdict,replace
from cassette_cam_dynamics import Parameters,State,ports,step,equilibrium,DomainStop
ROOT=pathlib.Path(__file__).resolve().parents[2]
PARAMETERS=Parameters(1.5,.0001,1000000,10,.03)

def trajectory(initial,dt,duration,torque,load=0.0):
    s=initial;start=ports(s,PARAMETERS,torque,load)['energy'];maximum=0.0
    for i in range(round(duration/dt)):
        s=step(s,PARAMETERS,dt,torque,load)
        r=ports(s,PARAMETERS,torque,load)
        residual=r['energy']-start-(s.actuator_work-initial.actuator_work)-(s.load_work-initial.load_work)+(s.contact_heat-initial.contact_heat)
        maximum=max(maximum,abs(residual))
    return {'initial':asdict(initial),'final':asdict(s),'dt':dt,'duration':duration,'requestedTorqueNm':torque,'noteGeneralizedForceN':load,'maxEnergyResidualJ':maximum,'endpoint':r}

def run():
    held,holding_torque=equilibrium(.2,PARAMETERS)
    disturbed=replace(held,udot=.002,thetadot=.1)
    dead,zero=equilibrium(0,PARAMETERS)
    perturb=replace(dead,theta=.001)
    rows={
      'held_open':trajectory(held,1e-5,.01,holding_torque),
      'moving_powered':trajectory(disturbed,2e-5,.01,holding_torque),
      'moving_powered_half_dt':trajectory(disturbed,1e-5,.01,holding_torque),
      'torque_removed_hypothesis':trajectory(disturbed,1e-5,.01,0),
      'exact_dead_center':trajectory(dead,1e-5,.01,0),
      'perturbed_dead_center':trajectory(perturb,1e-5,.01,0),
      'external_note_load_port':trajectory(disturbed,1e-5,.01,holding_torque,-.1),
      'loss_of_hold_250ms':trajectory(held,2e-5,.25,0),
      'loss_of_hold_250ms_half_dt':trajectory(held,1e-5,.25,0)}
    stop=ports(State(.0015,0,math.pi,0),PARAMETERS,0)
    impact=State(.001499,1,math.pi,0)
    try:step(impact,PARAMETERS,1e-4,0)
    except DomainStop as ex:impact_result={'rejected':True,'unchangedInput':asdict(impact),'reason':str(ex)}
    else:raise AssertionError('impact was not rejected')
    contract=json.loads((ROOT/'cad/assembly/powered-cam-interface.json').read_text())
    pins=dict(contract['geometrySourcePins'])
    for n in ['powered_cam_contact.py','powered-cam-interface.json','cassette_cam_dynamics.py','check_cassette_cam_dynamics.py']:
        p=ROOT/'cad/assembly'/n;pins[str(p.relative_to(ROOT)).replace('\\','/')]=hashlib.sha256(p.read_bytes()).hexdigest()
    for p,h in pins.items():
        if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h:raise ValueError('source mismatch '+p)
    return {'schema':'isolated-cassette-cam-dynamics/1','accepted':False,'inputs':pins,'hypothesisParameters':asdict(PARAMETERS),'rows':rows,'staticInwardStop':stop,'impactControl':impact_result,'electricalCurrentModel':None,'limits':['Fixed cradle, no integrated NOTE; external NOTE load fixture only','Mass/inertia/stiffness/damping/torque bounds are explicit unmeasured hypotheses','Zero applied torque is one mechanical fault hypothesis, not proven SRS lost-power behavior','No latch, hard-stop impact law or verified thermal holding','RK4 residual retained; no silent energy correction','Angle range is geometry domain, not a modeled mechanical angle stop']}
if __name__=='__main__':
    r=run();(ROOT/'cad/assembly/evidence/cassette-cam-dynamics.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n')
    print(json.dumps({k:{'u':v['final']['u'],'theta':v['final']['theta'],'residual':v['maxEnergyResidualJ']} for k,v in r['rows'].items()},indent=2))
