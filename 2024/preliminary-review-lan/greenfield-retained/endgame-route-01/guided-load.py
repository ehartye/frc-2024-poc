"""Re-use physical hook, gate, chain, unilateral floor contact and registered inertia in a guided-pull layout trial."""
from pathlib import Path
import math,sys,json
import numpy as np
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape
import mujoco as mj
p=Path(__file__).resolve().parent;srcdir=p.parent/'chain-contact-02'
tag,cg,mode=sys.argv[1:4];file=p/'guided-raw.xml'
# Emit the earlier solver's actual assembled XML before time integration. Only the generated model is used.
template=(srcdir/'common_gate.py').read_text()
mark="env['configureCommon']=globals()['configureCommon'];"
template=template.replace(mark,mark+"env['write_xml']=writer;src=src.replace('m=mj.MjModel.from_xml_string(configureCommon(xml))','write_xml(configureCommon(xml))');")
class Saved(Exception):pass
def writer(s):
    global assembled
    assembled=s
    raise Saved
sys.argv=[str(srcdir/'common_gate.py'),'unused','48','.00025',cg,'normal']
try:exec(compile(template,'source hook and chain model','exec'),dict(__file__=str(srcdir/'common_gate.py'),writer=writer))
except Saved:pass
tree=ET.fromstring(assembled);arm=tree.find(".//body[@name='cassette']");frame=tree.find(".//body[@name='frame']")
car=ET.Element('body',name='hookCarriage')
for child in list(arm):
    if child.get('name','').startswith(('hook','linked-keepers','h1','h2')):
        arm.remove(child);car.append(child)
arm.append(car)
ET.SubElement(car,'joint',name='draw',type='slide',axis='-1 0 0',limited='true',range='0 .103',damping='25',solreflimit='.001 1')
ET.SubElement(car,'inertial',mass='.75',pos='.18 0 .232',diaginertia='.004 .002 .004')
# Fit measured keeper mast vertices as a conservative convex envelope (the real stock has reliefs).
doc=json.loads((p.parent.parent/'www/data.json').read_bytes());v=np.array(doc['concepts']['field']['add']['Keeper drive mast']['v'])-[185,0,501];a=math.radians(125);v=v@np.array([[math.cos(a),0,-math.sin(a)],[0,1,0],[math.sin(a),0,math.cos(a)]])/1000
ET.SubElement(tree.find('asset'),'mesh',name='mastEnvelope',vertex=' '.join(f'{x:.9f}' for x in np.unique(v.round(8),axis=0).flatten()))
ET.SubElement(car,'geom',type='mesh',mesh='mastEnvelope',name='mast',contype='8',conaffinity='4',mass='0',solref='.003 1')
for name,pos,size in [('battery','-.0625 -.180 .2375','.0925 .040 .0865')]:
    ET.SubElement(frame,'geom',type='box',name=name,pos=pos,size=size,contype='8',conaffinity='4',mass='0')
ET.SubElement(frame,'geom',type='cylinder',name='slideMotor',fromto='-.102 .275 .428 -.049 .275 .428',size='.014',contype='8',conaffinity='4',mass='0')
angle=math.radians(75.38642106234);eq=tree.find(".//joint[@name='brake']")
eq.set('active','true');eq.set('polycoef',f'{angle} 0 0 0 0')
m=mj.MjModel.from_xml_string(ET.tostring(tree,encoding='unicode'));d=mj.MjData(m)
pivot=m.joint('pivot');draw=m.joint('draw');gate=m.joint('closedYoke')
def x(j):return float(d.qpos[j.qposadr[0]])
def speed(j):return float(d.qvel[j.dofadr[0]])
d.qpos[pivot.qposadr[0]]=angle;d.qpos[gate.qposadr[0]]=.044
dt=m.opt.timestep;cut=5.;records=[];bilateral=False;latched=False;maxContact={};load=0
sensors=[m.site(f's{i}').id for i in range(48)]+[m.site('last').id]
for i in range(round((cut+5)/dt)+1):
    mj.mj_forward(m,d);t=d.time
    openGate=t<.1 or mode=='open'
    d.qfrc_applied[gate.dofadr[0]]=np.clip(8000*(.044-x(gate))-9*speed(gate)+19.44,0,45) if openGate and t<cut else 0
    gap=[np.linalg.norm(d.site_xpos[m.site(h).id]-d.site_xpos[sensors[j]]) for h,j in [('h1',22),('h2',26)]]
    if t>.12 and max(gap)<.018 and x(gate)<.001:bilateral=True
    u=np.clip((t-.35)/2.5,0,1);target=.102*u*u*(3-2*u) if bilateral else 0.
    if x(draw)>.098 and mode!='no-catch' and not latched:
        latched=True;m.jnt_range[draw.id,0]=.097
    pull=float(np.clip(230000*(target-x(draw))-1300*speed(draw),0,1350)) if t<cut and bilateral else 0.
    d.qfrc_applied[draw.dofadr[0]]=pull
    cf=np.zeros(6);ground=0.;sides=[0.,0.]
    for j in range(d.ncon):
        c=d.contact[j];names=[m.geom(g).name or '' for g in [c.geom1,c.geom2]];mj.mj_contactForce(m,d,j,cf)
        if 'floor' in names:ground+=cf[0]
        for n in ('mast','battery','slideMotor'):
            if n in names:maxContact[n]=max(maxContact.get(n,0),cf[0])
        for k in range(2):
            if any(n.startswith(('hook'+str(k)+'_','keeper'+str(k))) for n in names):sides[k]+=cf[0]
    if i%round(.025/dt)==0:
        fr=m.body('frame').id;R=d.xmat[fr].reshape(3,3);z=min(min((d.xpos[fr]+R@np.array([dx,dy,.058]))[2] for dx in [-.437,.437] for dy in [-.436,.436]),min(d.geom_xpos[m.geom('tire'+str(k)).id,2]-.0508 for k in range(4)))
        records.append(dict(t=t,p=d.xpos[fr].tolist(),q=d.xquat[fr].tolist(),angle=np.rad2deg(x(pivot)),draw=x(draw),gate=x(gate),pull=pull,ground=ground,z=z,sides=sides,chain=d.site_xpos[sensors].tolist()))
    mj.mj_step(m,d)
held=[a for a in records if a['t']>cut+.15];out=dict(cg=float(cg),mode=mode,captured=bilateral,latched=latched,addedCarriageMassKg=.75,neighborN=maxContact,maxPullN=max(a['pull'] for a in records),disabledMinTireOrBumperM=min(a['z'] for a in held),maxDisabledFloorN=max(a['ground'] for a in held),disabledMinSideN=np.min([a['sides'] for a in held],axis=0).tolist(),peakDraw=max(a['draw'] for a in records),trace=records,scope='Free chassis; passive pivot lock and unilateral slide stop modeled ideally. No chain-to-hook attachment constraint. Assumed 1350 N drive cap and 750 g carriage increment. Mast uses a conservative convex envelope. Certain joints and equipment still require geometric/load and budget closure.')
(p/(tag+'.json')).write_text(json.dumps(out,separators=(',',':')));print({k:v for k,v in out.items() if k!='trace'})

