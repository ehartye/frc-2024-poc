"""Render a data plot of the measured model continuation; no CAD animation."""
import argparse,json,pathlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('--packet',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();assert not a.output.exists()
fig,axes=plt.subplots(2,1,figsize=(8,6),sharex=True,layout='constrained')
for suffix,style in [('half','-'),('quarter','--'),('eighth',':')]:
 r=json.loads((a.packet/f'native-pressure-lift-clocked-{suffix}/receipt.json').read_bytes());rows=r['trace'];t=[x['timeS']-r['initial']['timeS'] for x in rows]
 axes[0].plot(t,[(x['state']['noteZ']-.0254)*1000 for x in rows],style,label=f"NOTE, {r['stepS']*1000:g} ms")
 if suffix=='eighth':axes[0].plot(t,[(x['state']['zPivot']-.0254)*1000 for x in rows],color='black',alpha=.55,label='Cradle pivot, 0.125 ms')
 axes[1].plot(t,[x['floorForceN'][2] for x in rows],style,label=f"{r['stepS']*1000:g} ms")
axes[0].set_ylabel('Rise above initial height (mm)');axes[1].set_ylabel('Floor normal force (N)');axes[1].set_xlabel('Time since attained ground stop (s)');axes[0].legend(fontsize=8)
for ax in axes:ax.grid(alpha=.25);ax.axvline(.5,color='gray',linewidth=.8);ax.set_xlim(0,.8)
fig.suptitle('Continuous lift attempt: transient rise, then floor return\nUncalibrated pressure/mass/transmission hypotheses',fontsize=11)
fig.savefig(a.output,metadata={'Date':None,'Creator':'Matplotlib; first-party simulation scalar data'});print(str(a.output))
