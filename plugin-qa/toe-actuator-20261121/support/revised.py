from pathlib import Path
import sys,runpy
q=Path(__file__).parent
source=(q/'support.py').read_text()
def change(a,b):
 global source
 assert source.count(a)==1,a
 source=source.replace(a,b)
# Mount rear link behind the motor's rear extremity across the complete return. Side links straddle independent rack carriers in X.
change('piece(75,81,-(zh-89),3,3,8)','piece(73,81,-3,3,3,13),piece(75,81,-(zh-89),3,10,13)')
change('box([-144,-16,zh-4],[-136,2,zh+4]),box([-144,-2,82],[-136,2,zh+3]),box([-144,-3,82],[-72,3,87])','box([-177,-16,zh-3],[-169,-13,zh+16]),box([-177,-16,zh+10],[-169,2,zh+16]),box([-177,-2,82],[-169,2,zh+16]),box([-177,-3,82],[-72,3,87])')
# Boss face is the actual 0.25-mm recessed face on each supplier rack. Machine a stepped boss only where parent extrusion physically seats.
change('cyl(p,n,3.25,3.02)','cyl(p+n*.25,n,3.25,2.77)')
change('gib=cut(gib,piece(u-6,u+6,-4.7,4.7,1.55,6.2))','gib=cut(gib,piece(u-6,u+6,-4.7,4.7,2.8,6.2));gib=cut(gib,piece(u-6,u+6,-3.55,3.55,1.5,3.0))')
# End-of-rectangular-return beam needs a deck outside the central rear slot. Both rear brackets engage this base in broad positive metal contact. Keep aftmost X inside -128 at Z<104.
change("add('slotted head backbone',back)","back=union([back,box([-128,-58,94],[-104,-7.1,101])]);add('slotted head backbone',back)")
ns={'__name__':'fixture_v2','__file__':str(q/'support.py')};exec(compile(source,'<support-v2>','exec'),ns)
def fixture():
 w=ns;r,old=w['headRows']();lookup={p['name']:p for p in r}
 box=w['box'];cyl=w['cyl'];cut=w['cut'];un=w['union'];xf=w['xf'];np=w['np']
 # Integrate coincident aluminum brackets; relieve the lower POM guide and side motor envelope. Positive pads remain outside the measures below.
 brackets=[p for p in r if 'cheek' in p['name'] or 'stanchion' in p['name']];r=[p for p in r if p not in brackets];outer=[[-95,-19,104],[-35,27,152]];motor=box(*outer)
 # Physically route around the motor. The rear guide tower gets a bridge under the motor and rises outboard of X -95.
 bypass=un([box([-109,-9,100],[-86,-6,104]),box([-109,-9,101],[-97,-6,227]),box([-109,-9,224],[-82,-6,228])])
 # Union low structural stock, plate and guide risers; split for machining is an open process decision.
 plate=lookup['slotted head backbone'];parts=[p['shape'] for p in brackets]+[plate['shape'],bypass];built=un(parts);built=cut(built,motor)
 for p in r:
  if 'stationary POM' in p['name']:built=cut(built,p['shape'])
 # Keep presized M2 bracket holes and both journal feet separate.
 plate['shape']=built;plate['name']='integrated slotted backbone and six guide pedestals'
 # Machine the actual alloy carrier end with a small clearance pocket if moving link passes fixed rear gib.
 # Assembly bolt holes for the gibs pass their roof at V=±5.5, U centered. Two pilot datums exist per gib.
 for label,s,zh,us in [('left',1,153.45,[-1,21]),('right',-1,110.55,[-1,21]),('rear',0,221.45,[87,118])]:
  for k,u in enumerate(us):
   gib=lookup[f'{label} stationary POM guide {k}']
   for dz in [-5.5,5.5]:
    if s:pt=[3.4,s*u,zh+dz];ax=[1,0,0]
    else:pt=[-u,-17.6,zh+dz];ax=[0,1,0]
    hole=cyl(pt,ax,.9,12);gib['shape']=cut(gib['shape'],hole);plate['shape']=cut(plate['shape'],hole)
 # Clock the close shaft/insert against actual keyway shapes, measure per interface, enforce coexistence.
 return r,old
if __name__=='__main__':
 source2=(q/'screen.py').read_text().replace("rows,old=w['headRows']()","rows,old=fixture()").replace("'screen.json'","'screen-v2.json'").replace("('capture',34,34)","('mid-withdraw',5.5,-9.5),('capture',34,34)")
 exec(compile(source2,'<screen-v2>','exec'),dict(ns,fixture=fixture,__file__=str(q/'screen.py'),__name__='__main__'))
