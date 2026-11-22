"""Re-use physical hook, gate, chain, unilateral floor contact and registered inertia in a guided-pull layout trial."""
from pathlib import Path
import math,sys,json
import numpy as np
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape
import mujoco as mj
p=Path(__file__).resolve().parent;srcdir=p.parent/'chain-contact-02'
tag,cg,mode=sys.argv[1:4];stroke=float(sys.argv[4]);assert .10<=stroke<=.23;file=p/'guided-raw.xml'
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
ET.SubElement(car,'joint',name='draw',type='slide',axis='-1 0 0',limited='true',range=f'0 {stroke+.003}',damping='25',solreflimit='.001 1')
ET.SubElement(car,'inertial',mass='.75',pos='.18 0 .232',diaginertia='.004 .002 .004')
# Three source-defined occupied lands. The small axle bore lies away from the chain.
for i,(origin,size) in enumerate([((126,9,192),(62,12,7)),((126,9,199),(10,12,122)),((126,9,315),(74,12,16))]):
    half=np.array(size)*.0005;center=np.array(origin)*.001+half
    ET.SubElement(car,'geom',type='box',name='mast'+str(i),pos=' '.join(map(str,center)),size=' '.join(map(str,half)),contype='8',conaffinity='4',mass='0',solref='.003 1')
# Drive envelopes from keeper-drive-01 and the 6 mm bridge correction in keeper-drive-02.
for name,y1,y2,radius in [('shaft',-.004,.026,.003),('pinion',-.004,.004,.008),('motor',.021,.059,.014)]:
    ET.SubElement(car,'geom',type='cylinder',name=name,fromto=f'.189 {y1} .323 .189 {y2} .323',size=str(radius),contype='8',conaffinity='4',mass='0')
yoke=car.find(".//body[@name='linked-keepers']")
for name,origin,sz in [('bridge',(207,-104.88,273),(5,209.76,5)),('rack',(197,-4,270),(4,8,60)),('link',(198,-4,273),(14,8,5)),('leftbridge',(201,96.88,272),(11,8,6)),('rightbridge',(201,-104.88,272),(11,8,6))]:
    half=np.array(sz)*.0005;xyz=np.array(origin)*.001+half-[.2035,0,.2405]
    ET.SubElement(yoke,'geom',type='box',name=name,pos=' '.join(map(str,xyz)),size=' '.join(map(str,half)),contype='8',conaffinity='4',mass='0')
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
    u=np.clip((t-.35)/2.5,0,1);target=(stroke+.002)*u*u*(3-2*u) if bilateral else 0.
    if bilateral and x(draw)>stroke-.002 and mode!='no-catch' and not latched:
        latched=True;m.jnt_range[draw.id,0]=stroke-.003
    pull=float(np.clip(230000*(target-x(draw))-1300*speed(draw),0,1350)) if t<cut and bilateral else (float(np.clip(-180000*x(draw)-900*speed(draw),-150,150)) if t<cut else 0.)
    d.qfrc_applied[draw.dofadr[0]]=pull
    cf=np.zeros(6);ground=0.;sides=[0.,0.]
    for j in range(d.ncon):
        c=d.contact[j];names=[m.geom(g).name or '' for g in [c.geom1,c.geom2]];mj.mj_contactForce(m,d,j,cf)
        if 'floor' in names:ground+=cf[0]
        for n in ('shaft','pinion','motor','bridge','rack','link','leftbridge','rightbridge','mast0','mast1','mast2','battery','slideMotor'):
            if n in names:maxContact[n]=max(maxContact.get(n,0),cf[0])
        for k in range(2):
            if any(n.startswith(('hook'+str(k)+'_','keeper'+str(k))) for n in names):sides[k]+=cf[0]
    if i%round(.025/dt)==0:
        fr=m.body('frame').id;R=d.xmat[fr].reshape(3,3);z=min(min((d.xpos[fr]+R@np.array([dx,dy,.058]))[2] for dx in [-.437,.437] for dy in [-.436,.436]),min(d.geom_xpos[m.geom('tire'+str(k)).id,2]-.0508 for k in range(4)))
        records.append(dict(t=t,p=d.xpos[fr].tolist(),q=d.xquat[fr].tolist(),angle=np.rad2deg(x(pivot)),draw=x(draw),gate=x(gate),pull=pull,ground=ground,z=z,sides=sides,chain=d.site_xpos[sensors].tolist()))
    mj.mj_step(m,d)
held=[a for a in records if a['t']>cut+.15];out=dict(strokeM=stroke,cg=float(cg),mode=mode,captured=bilateral,latched=latched,addedCarriageMassKg=.75,neighborN=maxContact,maxPullN=max(a['pull'] for a in records),disabledMinTireOrBumperM=min(a['z'] for a in held),maxDisabledFloorN=max(a['ground'] for a in held),disabledMinSideN=np.min([a['sides'] for a in held],axis=0).tolist(),peakDraw=max(a['draw'] for a in records),trace=records,scope='Free chassis; passive pivot lock and unilateral slide stop modeled ideally. No chain-to-hook attachment constraint. Assumed 1350 N drive cap and 750 g carriage increment. Mast uses three source-dimensioned lands. Prior filled-hull trial is preserved separately. Certain joints and equipment still require geometric/load and budget closure.')
(p/(tag+'.json')).write_text(json.dumps(out,separators=(',',':')));print({k:v for k,v in out.items() if k!='trace'})
