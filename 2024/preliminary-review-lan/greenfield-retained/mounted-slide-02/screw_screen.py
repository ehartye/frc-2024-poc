"""Compare catalog leads on the existing stroke and preserve the required nut and end-support ratings."""
from pathlib import Path
import json, math,sys,hashlib,numpy as np
here=Path(__file__).resolve().parent;root=here.parent
sys.path[:0]=[str(root/'loaded-actuation-01'),str(root/'follower-01')]
from power import *
here=Path(__file__).resolve().parent
raw=(root/'loaded-actuation-01/purchased_power.py').read_text().split('cases=[motor_run(',1)[0]
raw=raw.replace("(here/'power.py')","(root/'loaded-actuation-01/power.py')").replace("exec(compile(body,'matched NEO 550 terminal curve','exec'))","")
exec(compile(raw,'matched curve','exec'))
r={};L=279.;forceBound=0.
for code,lead,eff in [('original',.008,.65),('032315',.008,.79),('039393',.010,.79),('039393-loss',.010,.65)]:
 runSrc=body.replace('lead=.008;eta=.65;',f'lead={lead};eta={eff};')
 runSrc=runSrc.replace("return dict(piece=leg", "return dict(maxRpm=float(max(abs(vx))*60/lead),maxMotorTorqueNm=float(max((slideI-1.4)*.0097)),maxOutputPowerW=float(max(slideForce*abs(vx))),piece=leg")
 exec(compile(runSrc,'specific screw lead and motor','exec'))
 q=[motor_run(leg,.16,25,2.,4.8) for leg in plan['plan']]
 rpm=max(v['maxRpm'] for v in q);F=max(v['maxSlideForceN'] for v in q);torque=max(v['maxMotorTorqueNm'] for v in q)
 minor=[dict(endFixity=f,requiredCoreMm=(rpm/.8)*(L/25.4)**2/(c*4.76e6)*25.4) for f,c in [('two simple supports',1.),('single fixed support, other free',.36)]]
 r[code]=dict(leadMm=lead*1000,assumedEfficiency=eff,weakBusMinV=min(v['weakBusMinV'] for v in q),terminalReserveV=min(v['minimumMotorVoltageReserveV'] for v in q),peakSlidePhaseA=max(v['maxSlidePhaseA'] for v in q),peakScrewRPM=rpm,peakAxialN=F,peakMotorTorqueNm=torque,peakSlideOutputW=max(v['maxOutputPowerW'] for v in q),minimumRootDiameterAtSpeed=minor)
report=dict(hypotheses=r,spanMm=L,root=sha(root/'guided-hoist-01/service-locked-robot.step'),sourceCatalog=sha(here/'helix-screw-catalog.pdf'),catalogDate='2024-10-18 PDF, later than kickoff; selection sensitivity only',sourceFormula='https://www.thomsonlinear.com/en/training/ball_screws/screw_critical_speed',scope='279 mm conservative end-to-end span, compared with 269 mm between block centers. Need actual core, end bearings, speed/load duty rating of the nut and ordered article evidence. The nominal 10 mm screw envelope cannot establish root diameter.')
(here/'screw-screen.json').write_text(json.dumps(report,indent=2))
print(r)

