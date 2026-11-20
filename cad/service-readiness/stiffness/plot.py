"""Student plan view of the structural model and three matched wheel-twist comparisons."""
import json
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from study import build

here = Path(__file__).resolve().parent
results = json.loads((here/"result.json").read_bytes())
fig, (plan,chart) = plt.subplots(1,2,figsize=(11,5.3),gridspec_kw={"width_ratios":[1.15,1]})
fig.subplots_adjust(left=.07,right=.97,top=.83,bottom=.16,wspace=.28)
fig.suptitle("Where the removable tube carries wheel twist",fontsize=17,weight="bold")
g,wheels,deck = build(True,"relaxed")
for a,b,sec in g.beams:
    if a[0]=="removable cross tube":
        color,width="#b5531c",5
    else:
        color,width="#465c63",2.5 if a[0] in ("rear tube","side tube","aft cross tube") else 1.1
    pts=np.stack([g.xy[a],g.xy[b]])
    plan.plot(pts[:,0],pts[:,1],color=color,lw=width,solid_capstyle="butt")
for k in wheels.values():
    x,y=g.xy[k]
    plan.scatter(x,y,s=120,facecolor="white",edgecolor="#253d45",linewidth=1.4,zorder=5)
    plan.text(x,y,"+" if x*y>0 else "−",ha="center",va="center",size=12,zorder=6)
for k in deck:
    plan.plot(*g.xy[k],"s",color="#477681",ms=4)
plan.set(aspect="equal", xlabel="Robot X (mm)   ·   forward →", ylabel="Robot Y (mm)",
         xlim=(-390,465),ylim=(-398,398),title="Four wheels, alternating vertical forces")
plan.legend(handles=[Line2D([],[],color="#b5531c",lw=4,label="Removable tie"),
                     Line2D([],[],marker="s",ls="",color="#477681",label="Deck load stations")],
            loc="upper center",bbox_to_anchor=(.5,-.17),ncol=2,frameon=False,fontsize=8)
modes=["rigid","relaxed","softWelds"]
ys=np.arange(len(modes))*1.4
for delta,label,model,color in [(-.28,"R9 / bolted R14","R14","#b5531c"),
                                (0,"Bare R13","R13","#526b74"),(.28,"Lower welded trial","lowerTie","#9aaab0")]:
    values=[results[model][mode]["twistNmPerDegree"] for mode in modes]
    bars=chart.barh(ys+delta,values,height=.25,color=color,label=label)
    chart.bar_label(bars,fmt="%.0f",padding=3,fontsize=8)
chart.set(yticks=ys,yticklabels=["Rigid joints","Compliant bolts","Compliant bolts\nand welds"],
          xlim=(0,950),xlabel="Wheel-twist stiffness (N·m/degree)",title="Same 52 N·m axle couple in every case")
chart.invert_yaxis()
chart.legend(loc="upper center",bbox_to_anchor=(.5,-.16),ncol=2,frameon=False,fontsize=8)
for ax in (plan,chart):
    ax.spines[["top","right"]].set_visible(False)
    ax.tick_params(labelsize=8)
fig.savefig(here/"comparison.png",dpi=150,bbox_inches="tight",pad_inches=.2)
