"""Same-clock local temporal refinement, without claiming full fresh-start convergence."""
import pathlib,json
import run_fresh_local_refinement as model
R=model.R
FILES=[model.SOURCE,'sim/pressure/fresh-local-half-evidence.json','sim/pressure/compare_fresh_local_refinement.py']
def compare():
 n=json.loads((R/FILES[0]).read_text());h=json.loads((R/FILES[1]).read_text());p,controller,pins,bus,a=model.load()
 if h['inputs']!=pins or h['startCheckpoint']!=model.export_domain(p) or h['controllerStart']!=vars(controller) or h['stepS']!=.0005 or len(h['trace'])!=206 or h['outcome']!='HORIZON':raise ValueError('local refinement identity')
 ne=n['events'][140:];he=h['events']
 if len(ne)!=len(he) or any(abs(x['timeS']-y['timeS'])>1e-12 for x,y in zip(ne,he)):raise ValueError('controller clocks differ')
 first=n['trace'][1399];last=n['trace'][-1];half=h['trace'][-1]
 if abs(last['timeS']-half['timeS'])>1e-12:raise ValueError('endpoint clocks differ')
 result=dict(schema='fresh-local-refinement-comparison/1',inputs=dict(pins,**{f:model.c.factory.r.sha(R/f) for f in FILES}),sameStartTimeS=first['timeS'],sameEndTimeS=last['timeS'],nominalStepS=.001,halfStepS=.0005,nominalSteps=103,halfSteps=206,firstContactAcceptedTimeS=[n['firstContact']['acceptedTimeS'],next(x['timeS'] for x in h['trace'] if x['contacts'])],endpointAbsoluteDeltas=dict(relativeXM=abs(last['relativeRobotXY'][0]-half['relativeRobotXY'][0]),chassisVxMps=abs(last['chassis']['vx']-half['chassis']['vx']),noteVxMps=abs(last['note']['v'][0]-half['note']['v'][0]),potentialJ=abs(last['energyJ']['spring']-half['energyJ']['spring'])),localResidualJ=[last['energyJ']['residual']-first['energyJ']['residual'],half['newPhaseResidualJ']],controllerModeDifferences=[i for i,(x,y) in enumerate(zip(ne,he)) if x['command']['modes']!=y['command']['modes']],acceptance=False,limits=['Both use exact recorded1.400s state and original clock; original startup remains1ms.','Same fixed1024-square quadrature/material assumptions, not spatial convergence or physical calibration.','No full3s result, capture, stopped or lift acceptance.'])
 if any(model.c.factory.r.sha(R/f)!=v for f,v in result['inputs'].items()):raise ValueError('source drift')
 return result
if __name__=='__main__':
 import sys
 pathlib.Path(sys.argv[1]).write_text(json.dumps(compare(),indent=2)+'\n',encoding='utf-8',newline='\n')
