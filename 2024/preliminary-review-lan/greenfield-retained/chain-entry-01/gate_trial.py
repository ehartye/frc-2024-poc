"""Trial a straight spring-closed gate with an integral guide and lower pocket in each hook."""
from pathlib import Path
p=Path(__file__).resolve().parent
s=(p/'entry.py').read_text();exec(compile(s[:s.index('nominal=check(')],'registered hook sections','exec'))
from layout import ann,prism,write_step,G,viewmesh
from OCP.BRepCheck import BRepCheck_Analyzer
def cat(parts):
    sh=parts[0]
    for b in parts[1:]:sh=fuse(sh,b)
    return sh
new={};attr={};stock=set()
for sign,side in [(1,'left'),(-1,'right')]:
    y=sign*100.88
    # Radius and socket chosen from the measured link section; guide stands on the upper jaw.
    hoop=cat([cyl((180,y-4,232),Y,31,8),box(172,y-4,192,16,8,40)])
    hoop=cut(hoop,cyl((180,y-5,232),Y,18,10));hoop=cut(hoop,box(179,y-5,214,36,10,36))
    shape=cat([hoop,box(187,y-6,250,21,12,14),box(187,y-6,204,21,12,10)])
    slot=box(200.7,y-4.2,207,5.6,8.4,67)
    shape=cut(shape,slot)
    gate=box(201,y-4,208,5,8,65)
    new[side+' hook']=shape;new[side+' gate']=gate
    attr[side+' hook']='fixed';attr[side+' gate']='slide'
    assert BRepCheck_Analyzer(shape).IsValid()
def buildloc(stage):
    return {n:(place(b,mat(0,0,0,stage)) if attr[n]=='slide' else b) for n,b in new.items()}
# Continue to use the exact manufactured chain at each lateral setup.
logs=[];zErrs=[-2.,0.,2.];dxs=[-2.,0.,2.]
for dy in [-10.,0.,10.]:
    localParts={s:BRepAlgoAPI_Common(chain,box(cx-40,nom_y+dy+y-4.05,690,80,8.1,85)).Shape() for s,y in [('left',100.88),('right',-100.88)]}
    for ex in dxs:
        for ez in zErrs:
            th,x=ik(180,cz+ez);pose=mat(th,x+185+ex,nom_y+dy,501)
            gap=min(distance(place(shape,pose),localParts[n.split()[0]]) for n,shape in new.items())
            logs.append([dy,ex,ez,gap])
strokes=[];ms=[]
for chainX,stroke in [(float(a),44) for a in np.linspace(238,180,30)]+[(180,int(w)) for w in range(44,-1,-2)]:
    th,x=ik(chainX,cz);world=mat(th,x+185,nom_y,501);posed=buildloc(stroke)
    sets=[]
    for side in ['left','right']:
        q=posed[side+' gate'];h=posed[side+' hook']
        sets.append([common(q,h),distance(q,h),distance(place(h,world),parts[side]),distance(place(q,world),parts[side])])
    strokes.append(dict(chainX=chainX,travel=stroke,values=sets))
    ms.append(min(min(a[2:]) for a in sets))
negative=[]
for dist in [210,204,198]:
    th,x=ik(dist,cz);ww=mat(th,x+185,nom_y,501)
    negative.append([dist,*[common(place(new[k+' gate'],ww),parts[k]) for k in ['left','right']]])
out=dict(sectionDatum=[cx,cz],registered=logs,approach=strokes,closedControl=negative,minAcrossSetup=min(v[-1] for v in logs),minAcrossNominal=min(ms),maxGuideCommon=max(v[0] for row in strokes for v in row['values']),scope='Two revised hooks and two guided gates. Real chain captured in its as-saved shape. Gate drive/sensing, motion between sampled poses, loaded chain reaction, mounts and compliance remain open.')
(p/'gate-trial.json').write_text(json.dumps(out,indent=2));G.clear();G.update(new);write_step(p/'hook-and-guided-keepers.step')
print(out['minAcrossSetup'],out['minAcrossNominal'],out['maxGuideCommon'],negative)

