"""Export selectable preliminary geometry of the service trial, including source motor wire terminations."""
from pathlib import Path
import json,sys,hashlib,math,numpy as np
p=Path(__file__).resolve().parent;root=p.parent
sys.path.insert(0,str(root/'assembly-recheck-01'))
from sweep import load_solids,name
from layout import viewmesh,fuse,cyl,X
def get(f):return {name(k):s for k,s in load_solids(f)[0].items()}
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
cap=p/'slide-cartridge.step';hub=p/'drive-service.step'
stock=get(cap);stock['Slide drive motor']=stock.pop('SOLID');stock['Slide bearing mount 220']=stock.pop('Machined bearing block');del stock['Trial coupling blank']
service=json.loads((p/'drive-service.json').read_bytes())
D=np.array([0.,1.,1.])/math.sqrt(2)
motor=fuse(cyl((256,275,428),X,17.5,44.5),cyl((251.75,275,428),X,6.5,4.26))
motor=fuse(motor,cyl((244.60,275,428),X,1.585,7.16))
for port in service['path']:
    last=np.array(port['start'])
    motor=fuse(motor,cyl(last-16*D,D,port['radius'],16))
stock['Slide drive motor']=motor
parts=stock|get(hub)
out={};checks={}
for k,shape in parts.items():
    print(k,flush=True);m=viewmesh(shape);kind='mechanism';group='Slide drive and service';material='Preliminary aluminum stock'
    if k.startswith(('M2','M3','M4','Spindle')):material='Steel hardware'
    if k in ['Slide drive motor','Slide bearing mount 220','Double pinch hub','Spindle']:
        reason={'Slide drive motor':('Drive the slide screw in both directions.','The purchase-bound motor fits the cap at the calculated loaded voltage.','Purchased NEO 550, mixed materials'), 'Slide bearing mount 220':('Support the screw and attach the motor receiver to the chassis.','Two offset pilots leave room for the spinning clamp and lie in solid stock.','Machined aluminum'), 'Double pinch hub':('Couple the screw spindle to the round motor shaft.','Separate pinches can be loosened with the cartridge still in the robot.','Aluminum clamp blank'), 'Spindle':('Transmit drive torque into the screw.','The face keeps the motor and screw shaft ends separated.','Steel shaft')}[k]
    elif k.startswith(('lead','plug','socket')):
        kind='electrical';item=k.split()[0];reason={'lead':('Carry phase power or encoder signals from the motor.','Each swept tube starts at a measured wire end and separates gently before the connector.','Insulated copper wire allowance'), 'plug':('Disconnect the motor-side lead.','A separated socket lets this end travel with the cartridge.','Connector envelope'), 'socket':('Connect to the chassis harness.','The harness end moves clear before the motor is drawn out.','Connector envelope')}[item]
    elif k=='Removable cap and ears':reason=('Locate the motor pilot and fasten the motor at the bench.','The assembled bearing block obstructs both rear motor-screw driver paths.',material)
    elif k=='Stationary rail receiver':reason=('Locate and support the removable cap.','Two top-access fasteners capture the cartridge against the installed axial seats.',material)
    elif k.startswith('M3 on cap'):reason=('Fasten the motor to the removable cap.','These fasteners stay installed during cartridge removal.',material)
    elif k.startswith('M4 cap retention'):reason=('Retain the cartridge on its locating rails.','Both screws must be removed before axial withdrawal.',material)
    elif k.startswith('M4 receiver'):reason=('Attach the stationary receiver to the prepared bearing block.','Installing these first leaves clear straight driver paths.',material)
    else:reason=('Clamp the hub to its shaft.','At the indexed maintenance position a straight driver reaches this screw.',material)
    m.update(category=kind,group=group,material=reason[2],purpose=reason[0],rationale=reason[1],basis='Preliminary package derived from the service trial, 1:1 in millimeters. Connector mating hardware and clamp preload require measured confirmation.')
    if k=='Slide drive motor':m['category']='electrical';m['basis']='Lightweight motor envelope from the purchased STEP, including its pilot, shaft, 35 mm case and all four measured wire-end faces. Full purchased BRep was used in the service check.'
    out[k]=m;checks[k]={'faces':len(m['f']),'closed':m['closed'],'volume':m['volume'],'error':m['error']}
(p/'display-mesh.json').write_text(json.dumps(out,separators=(',',':')));(p/'display-register.json').write_text(json.dumps(dict(cartridge=sha(cap),drive=sha(hub),parts=checks),indent=2));print(checks)

