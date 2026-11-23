from pathlib import Path
p=Path(__file__).resolve().parent
s=(p/'approach-map.py').read_text(encoding='utf8').replace('np.arange(-18,166,1.)','np.arange(-49,166,1.)')
idx=s.index("out=dict(")
s=s[:idx]+"""from scipy.ndimage import label,distance_transform_edt
lab,_=label(~closed);component=lab==lab[start];near,index=distance_transform_edt(~component,return_indices=True);goalComp=lab==lab[goal]
cst=np.where(goalComp,near,np.inf);ea,ei=np.unravel_index(cst.argmin(),cst.shape);sa,si=index[:,ea,ei]
closest=dict(start=[float(angles[sa]),float(xs[si])],end=[float(angles[ea]),float(xs[ei])],gridRange=float(cst.min()),endComponentCells=int(goalComp.sum()))
"""+s[idx:]
s=s.replace('out=dict(source=','out=dict(nearest=closest,source=').replace("p/'reverse-approach.json'","p/'low-entry.json'")
exec(compile(s,'lowered reverse approach','exec'))

