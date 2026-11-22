from travel import *
from OCP.TopAbs import TopAbs_FACE
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
def area(a,b):
    e=TopExp_Explorer(a,TopAbs_FACE); total=0
    while e.More():
        f=TopExp_Explorer(b,TopAbs_FACE)
        while f.More():
            shape=BRepAlgoAPI_Common(e.Current(),f.Current()).Shape()
            g=GProp_GProps(); BRepGProp.SurfaceProperties_s(shape,g);total+=g.Mass()
            f.Next()
        e.Next()
    return total
site=lambda n:place(G[n],inverse)
rows={}
for side in ('left','right'):
    b='Rear stop bracket · '+side; c='Runner crossbar 125';beam='Cassette side beam · '+side
    for a,n in [('Rear piece stop',b),(b,c),(c,beam),('Rear stop screw · '+side,b),('Stop bracket through bolt · '+side,c),('Stop bracket locknut · '+side,b)]:
        v=area(site(a),site(n));assert v>.1,(a,n,v);rows[a+' / '+n]=round(v,3)
v=area(site('Rear stop pad'),site('Rear piece stop'));assert v>4000;rows['Pad / backing']=round(v,3)
# Removing both brackets and screws leaves the original support gap.
omitted={held,'Rear stop pad','Rear piece stop',*[n for n in G if n.startswith(('Rear stop screw','Rear stop bracket','Stop bracket through','Stop bracket locknut'))]}
neg=min((distance(G['Rear piece stop'],sh),n) for n,sh in G.items() if n not in omitted)
out=dict(step=sha(HERE/'retained-cassette.step'),contactAreaMm2=rows,removedBracketNearest=dict(gapMm=neg[0],name=neg[1]),scope='BRep face contact, nominal envelopes. Crossbar ends seat on the beams; welded attachments and bracket fastener capacity require sizing.')
dump(HERE/'stop-support.json',out);print(out)

