from pathlib import Path
p=Path(__file__).resolve().parent
g=p/'geometry.py';s=g.read_text(encoding='utf8')
anchor='prev=set(G);new=[];desc={}'
insert='''# Shift the full discharge/retention pivot so the entire folding leaf clears the seated front rim.
oldAxis=399.5;axis=457.5;flapName='Mouth retention flap';delta=mat(0,axis-oldAxis,0,0)
oldMove=mat(park,*pivot);inverseBase=np.linalg.inv(oldMove)
for name in (flapName,'Flap axle'):G[name]=place(G[name],oldMove@delta@inverseBase)
for side,y in [('left',218),('right',-226)]:
    name='Lid edge rail · '+side
    ledge=box(36,208 if side=='left' else -218,76,372,10,4)
    sidewall=cut(box(36,y,76,430,8,12),box(172,y,75,16,8,15))
    G[name]=place(cut(fuse(sidewall,ledge),cyl((axis,y,84),Y,2,8)),oldMove)
hingeChanged=[flapName,'Flap axle','Lid edge rail · left','Lid edge rail · right']
'''
s=s.replace(anchor,insert+anchor).replace("prev=set(G);","prev=set(G);")
s=s.replace("for i,n in enumerate(new):","for i,n in enumerate(new+hingeChanged):").replace("sorted(prev|set(new[:i]))","sorted((prev-set(hingeChanged))|set((new+hingeChanged)[:i]))")
s=s.replace("out['nominalBeamLapPerSideMm2']=450","out['nominalBeamLapPerSideMm2']=450\nout['hingeChanges']={n:viewmesh(G[n]) for n in hingeChanged}\nout['flapAxleLocalMm']=[axis,0,84]")
g.write_text(s,encoding='utf8')
q=p/'pickup_trial.py';s=q.read_text(encoding='utf8');s=s.replace("    if LIP:", '''    fixed+=geom('cylinder','name="sleeve" pos=".4575 0 .084" size=".003 .037" quat=".70710678 .70710678 0 0"','margin=".0018" contype="2" conaffinity="1"')
    fixed+=geom('cylinder','name="axle" pos=".4575 0 .084" size=".002 .226" quat=".70710678 .70710678 0 0"','margin=".0018" contype="2" conaffinity="1"')
    if LIP:''')
q.write_text(s,encoding='utf8')

