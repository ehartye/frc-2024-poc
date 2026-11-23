from pathlib import Path
import hashlib,json,numpy as np
q=Path(__file__).parent
r=json.loads((q/'roster.json').read_bytes())
def h(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def mass(row):
 if '1683-1' in row['name']:return row['volumeMm3']*.00270 # aluminium sensitivity only
 return row['massG']
for s,y,x in [('left',78,10),('right',-78,10),('rear',0,-75)]:
 r.append(dict(name=s+' original finger inertia',j=s,massG=80.,cgMm=[x,y,45]))
m=np.array([mass(x)for x in r])/1000;c=np.array([x['cgMm'] for x in r]);report={'rosterSha256':h(q/'roster.json'),'massKg':m.sum(),'loadedWithNoteKg':m.sum()+.2413,'minIncludedUsingAluminumForFourBlockHousings':True,'legacyMassSubstitution':'0.800 kg replaced by metal backbone, finger mass 0.08 kg x3 retained. No blanket additive allowance. Four bearing blocks assumed aluminum except insert and races; vendor net mass absent. Catalog masses for purchased motor/reducer subdivided by geometry volume as proxy only. Harness, guards, hardware, guide corrections and shoulders not included.','poses':{},'cases':{}}
for lab,sl,sr in [('neutral',0,0),('capture',34,34),('home',-23,-53)]:
 d=np.array([{'left':[0,sl,0],'right':[0,-sl,0],'rear':[-sr,0,0],None:[0,0,0]}[p['j']] for p in r]);cg=np.average(c+d,axis=0,weights=m);report['poses'][lab]={'cgMm':cg.tolist(),'maxGravityNmPitchAboutXneg70Z195':float(np.linalg.norm((cg-np.array([-70,0,195]))[[0,2]])/1000*9.80665*m.sum()),'gravityNmAtZeroPitch':float((cg[0]+70)*9.80665*m.sum()/1000)}
for n in [96,192]:
 a=np.load(q.parent/f'replay-{n}'/'toe.npy');a=a[(a[:,1]>=.32)&(a[:,1]<=4.75)]
 F=abs(a[:,4::3]);v=abs(a[:,3::3]);height=np.array([153.45,110.55,221.45])-4;span=np.array([22,22,31]);ratio=76/21;r0=.01125
 for mu in [0,.1,.25]:
  drag=2*mu*F*height/span
  drive=F+drag;pair=np.stack([drive[:,0]+drive[:,1],drive[:,2]],axis=1)
  current=1.4+r0*pair/(.0097*ratio*.65)
  report['cases'][f'{n}/{mu}']={'condition':'Entire recorded generalized axial demand applied at 4 mm toe center; unknown contact/guide friction and force split. Excludes grip preload, seals, wires and acceleration of revised carriage.','peakToeN':F.max(axis=0).tolist(),'peakSinglePadCoupleN':(F*height/span).max(axis=0).tolist(),'peakDriveN':pair.max(axis=0).tolist(),'peakMotorA':current.max(axis=0).tolist(),'toeSpeedAtMaxDemandMps':[float(v[np.argmax(pair[:,0]),0]),float(v[np.argmax(pair[:,1]),2])]}
(q/'loads.json').write_text(json.dumps(report,indent=2))
print(report['massKg'],report['poses'],report['cases']['96/0.1'])
