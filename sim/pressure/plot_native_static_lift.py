"""Plot measured lift/index trajectories and constitutive counterexamples."""
import argparse,json,pathlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--packet',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();assert not a.output.exists()
load=lambda n:json.loads((a.packet/n/'receipt.json').read_bytes());fig,axes=plt.subplots(2,1,figsize=(8.5,6.5),layout='constrained')
for name,label,style in [('native-static-lift-hold-half','Dry friction, finite lift / free rotors','-'),('native-static-lift-zero','Zero friction','--'),('native-static-lift-low','Friction coefficient0.1',':'),('native-pressure-lift-clocked-half','Prior regularized law (different integration/control)','-.')]:
 r=load(name);rows=r['trace'];t=[x['timeS']-r['initial']['timeS'] for x in rows];axes[0].plot(t,[(x['state']['noteZ']-.0254)*1000 for x in rows],style,label=label)
axes[0].set(xlim=(0,2),ylabel='NOTE rise (mm)',xlabel='Time since attained ground stop (s)');axes[0].legend(fontsize=7,loc='lower right')
for name,label,style in [('native-raised-index-half','0.5 ms','-'),('native-raised-index-quarter','0.25 ms','--')]:
 r=load(name);initial=r['initial']['note']['p'][0]-r['initial']['chassis']['x'];rows=r['trace'];t=[x['timeS']-r['initial']['timeS'] for x in rows];axes[1].plot(t,[(x['state']['note']['p'][0]-x['state']['chassis']['x']-initial)*1000 for x in rows],style,label=label)
axes[1].axhline(-4,color='gray',linewidth=.8,label='Requested stroke');axes[1].set(xlim=(0,.8),ylabel='Relative rearward travel (mm)',xlabel='Time since attained raised hold (s)');axes[1].legend(fontsize=8)
for ax in axes:ax.grid(alpha=.25)
fig.suptitle('Finite-effort retained lift and raised indexing\nNative-bound, uncalibrated model evidence; no full launcher handoff',fontsize=11);fig.savefig(a.output,metadata={'Date':None,'Creator':'Matplotlib; first-party simulation scalar data'})
if a.output.suffix=='.svg':a.output.write_text('\n'.join(x.rstrip() for x in a.output.read_text().splitlines())+'\n',encoding='utf-8',newline='\n')
print(str(a.output))
