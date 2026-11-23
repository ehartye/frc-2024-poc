"""Replace two early allocations with actual occupied stock plus explicitly unmodeled purchasing ranges."""
from pathlib import Path
import json,numpy as np,math,hashlib
p=Path(__file__).resolve().parent;root=p.parent
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
doc=read(root.parent/'www/data.json');c=doc['concepts']['field'];bs={**doc['bodies'],**c['add']}
parent=read(root/'tread-underlay-01/review.json');pkg=read(p/'stop-slot.json')
rows={r['id']:r for r in c['mass']}
old=['Shared cassette lift','Common keeper and return drive']
prior=np.array([rows[k]['kg'] for k in old]).sum(0)
material={}
for k,rho in [('Climb yoke post · left',2700),('Climb yoke post · right',2700),('Shared lift crosspiece',2700),('Chain hook · left',7850),('Chain hook · right',7850),('Common keeper yoke and sliding rack',7850),('Keeper drive mast',2700),('Keeper pinion output',7850),('Keeper return pinion',7850)]:
    material[k]=bs[k]['volume']*rho*1e-9
# Net includes only the actual differences on the separately budgeted pivot shaft, sector and receiver cheek.
stock=sum(material.values())+pkg['netStockKg']
allowances={
    'Keeper gearmotor, spring, sensor and terminations, original incremental high allocation':[.128,.295],
    'Two slide-stop actuators and springs including sleeves, beyond shown shells':[.060,.220],
    'Clutch shifter and fork including spring and reaction bracket':[.085,.300],
    'Pitch, clutch and slide proof switches and local harness':[.060,.180],
    'Guide and anchor screws, ties, end retainers, rope windings and dead ends':[.090,.250],
    'Brackets, two journals, guards and local structural reinforcement':[.125,.550]}
package=stock+np.array(list(allowances.values())).sum(0)
new=np.array(parent['inspection'])-prior+package
ro=np.array(parent['running'])-prior+package
mass=dict(originalInspection=parent['inspection'],originalRunning=parent['running'],replacedBudget={k:rows[k]['kg'] for k in old},unchangedGeneralReserve=rows['Unallocated design reserve']['kg'],retainedBodyKg=material,netChangedStockKg=pkg['netStockKg'],dimensionedStockKg=stock,purchasingAllowances=allowances,packageRangeKg=package.tolist(),inspectionRangeKg=new.tolist(),runningRangeKg=ro.tolist(),inspectionLimitKg=125*.45359237,highReserveKg=125*.45359237-new[1],scope='Conservative purchasing ranges. All general, pivot, catch and other phase reserves retained. The two older lump sums are replaced. Prospective changes require another takeoff.')
trials={}
for z in ['front','rear','open','no-catch']:
    f=p/('winch-'+z+'.json');r=read(f);q=[v for v in r['trace'] if v['t']>=r['cutS']-1.e-7]
    trials[z]=dict(sha=sha(f),ground=max(v['ground'] for v in q),floorMm=1000*min(v['z'] for v in q),minHookN=np.min([v['sides'] for v in q],axis=0).tolist(),peakRopeN=r['maxCombinedCableN'],minBusV=r['weakBusMin'],phaseA=r['peakPhaseA'],peakBatteryA=r['peakBatteryA'],peakMotorW=r['peakMotorW'],peakPitchNm=r['peakPitchLockNm'],signedCg=r['cg'],captured=r['firstCapture'],latched=r['latched'],durationS=r['cutS'],timeAfterDecisionAtCutoff=13.98678471445554+r['cutS'])
peak=max(trials[k]['peakRopeN'] for k in ['front','rear']);lock=max(trials[k]['peakPitchNm'] for k in ['front','rear'])
# Screening loads only. 1.5 dynamic multiplier is a sensitivity. A single branch can transiently take all the line load.
side=1.5*peak
strength=dict(singleCableShockN=side,requiredMinBreakingLoadAtFiveToOneN=5*side,perStopSingleShearMPa=side/(math.pi*3**2),socketBearingAt8mmEntryMPa=side/(6*7.7),lockPinSideN=lock*1.5/.055,lockPinCantileverMm=7.2,lockPinBendingMPa=32*lock*1.5/.055*7.2/(math.pi*12**3),lockBushingSeatMPa=lock*1.5/.055/(15*10),lineDrumTorqueNm=peak*.0295,fullWorkingWraps=.230/(2*math.pi*.0295),usableDrumWidthMm=14,requiredWrapWidthMm=(2+.230/(2*math.pi*.0295))*3.2,scope='Maximum modeled loads with stated factors. Either 3 mm rope needs manufacturer rating after termination and bend loss. Pivot pin, backing, web fasteners, wear and shock require qualification.')
out=dict(packageStep=pkg['step'],mass=mass,trials=trials,mechanical=strength)
(p/'budget.json').write_text(json.dumps(out,indent=2));print(package,new,mass['highReserveKg'],trials['front']['floorMm'],trials['rear']['floorMm'],strength)

