"""Measured chassis motion and NOTE clearance from complete simulation traces."""
import argparse,hashlib,json,pathlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--runs',nargs='+',required=True);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();assert not a.output.with_suffix('.svg').exists()
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'svg.hashsalt':'native-static-cycle-20261118'})
fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True,layout='constrained');sources=[]
for num,name in enumerate(a.runs):
 path=pathlib.Path(name)/'receipt.json';r=json.loads(path.read_bytes());sources.append({'receiptSha256':hashlib.sha256(path.read_bytes()).hexdigest(),'stepS':r['stepS'],'outcome':r['outcome']});rows=r['trace'];t=np.array([x['timeS'] for x in rows]);s=[x['state'] for x in rows];label=f"{1000*r['stepS']:g} ms step";style='-' if num==0 else '--'
 axes[0].plot(t,[v['chassis']['x'] for v in s],style,label=label,lw=1.7)
 clearance=[1000*(v['noteZ']-.1524*abs(np.sin(v['notePitch']))-.0254) for v in s];axes[1].plot(t,clearance,style,lw=1.7)
 pickup=next((j for j in r['joins'] if j['toPhase']=='lift'),None);idx=next((j for j in r['joins'] if j['toPhase']=='index'),None)
 if pickup:
  values=[1000*(v['note']['p'][0]-v['chassis']['x']) if ti>=pickup['state']['timeS'] else np.nan for ti,v in zip(t,s)];axes[2].plot(t,values,style,lw=1.7)
 if idx and num==0:
  state=idx['state'];target=1000*(state['note']['p'][0]-state['chassis']['x']-.004);axes[2].hlines(target,state['timeS'],t[-1],color='#555555',ls=':',label='Final index target')
 if num==0:
  for j in r['joins']:
   for ax in axes:ax.axvline(j['state']['timeS'],color='#999999',lw=.8,ls=':')
  for j in r['joins']:axes[0].text(j['state']['timeS']+.03,.04,j['toPhase'].capitalize(),rotation=90,va='bottom',color='#555555')
axes[0].set_ylabel('Chassis travel (m)');axes[0].legend(loc='upper left');axes[1].set_ylabel('NOTE floor clearance (mm)');axes[1].axhline(0,color='#555555',lw=.8);axes[2].set_ylabel('NOTE relative X (mm)');axes[2].set_xlabel('Physical simulation time (s)');axes[2].legend(loc='upper right')
for ax in axes:ax.grid(alpha=.2);ax.spines[['top','right']].set_visible(False)
fig.suptitle('Fresh floor pickup → bounded lift → raised indexing\nOne contact law and physical state throughout',fontsize=14)
fig.get_layout_engine().set(rect=(0,.045,1,.955))
fig.text(.5,.01,'Uncalibrated finite-motor model • 8 mm lift envelope • Full launcher transfer and hardware validation remain open',ha='center',fontsize=9,color='#555555')
fig.savefig(a.output.with_suffix('.svg'),metadata={'Date':None});fig.savefig(a.output.with_suffix('.png'),dpi=150)
svg=a.output.with_suffix('.svg');svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n',encoding='utf-8',newline='\n')
a.output.with_suffix('.json').write_text(json.dumps({'producerSha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),'sources':sources,'scope':'Trace plot only; no animation, GUI or real-time demonstration claim'},indent=2)+'\n')
