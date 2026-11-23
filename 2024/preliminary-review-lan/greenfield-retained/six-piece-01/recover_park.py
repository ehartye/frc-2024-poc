"""Choose a lower empty-cassette posture and route around the actual Stage and peer envelopes."""
from pathlib import Path
import sys
p=Path(__file__).resolve().parent
original=(p/'park.py').read_text()
setup=original[:original.index('ab,roster=build_bounds()')].replace('zero=mat(0,scheduled(0),501)','zero=mat(park_angle,scheduled(park_angle),501)').replace('mat(0,scheduled(0)-185)','mat(0,scheduled(park_angle)-185)')
exec(compile(setup,'registered stock bounds','exec'))
parkroot=p
for park_angle in [-18.,-26.,-36.]:
    ab,roster=build_bounds();d=json.loads((root/'park-inputs.json').read_bytes())
    blocks=[];codes=[]
    for ob in fielddata['parts']:
        if ob['path'][1] in ignore:continue
        for aa,bb in ob.get('tightBounds',[ob['bounds']]):
            aa=np.array(aa)*1000;bb=np.array(bb)*1000
            if bb[2]<.01 or bb[0]/25.4<105 or aa[0]/25.4>560 or aa[1]/25.4>250 or bb[1]/25.4<15:continue
            near=ab[(ab[:,2]-.25<=bb[2])&(ab[:,5]+.25>=aa[2])]
            if len(near)==0:continue
            blocks.extend(np.c_[aa[:2]-near[:,3:5]-.25,bb[:2]-near[:,:2]+.25]/25.4)
    arr=np.unique(np.round(blocks,7),axis=0)
    src=(root/'park_rehearsal.py').read_text();src=src[src.index('frames=d['):];src=src[:src.index("dump(HERE/")];i=src.index('mx=mat(');j=src.index('mins=ab',i);src=src[:i]+src[j:]
    patch="""peerlow=low.copy();peerhigh=high.copy()
boundsarray=arr
def hits(a,b,padding=.15):
    a=np.asarray(a);b=np.asarray(b)
    if np.any(np.minimum(a,b)+lo<0) or np.any(np.maximum(a,b)+hi>[651.25,323.25]):return ['wall']
    l=np.minimum(a,b)-padding;h=np.maximum(a,b)+padding
    ix=np.flatnonzero((boundsarray[:,0]<=h[0])&(boundsarray[:,2]>=l[0])&(boundsarray[:,1]<=h[1])&(boundsarray[:,3]>=l[1]))
    L=np.vstack([peerlow,boundsarray[ix,:2]])-padding;U=np.vstack([peerhigh,boundsarray[ix,2:]])+padding
    vel=b-a;mn=np.zeros(len(L));mx=np.ones(len(L))
    for i in range(2):
        if abs(vel[i])<1e-9:mn=np.where((a[i]<L[:,i])|(a[i]>U[:,i]),2,mn)
        else:
            f=(L[:,i]-a[i])/vel[i];g=(U[:,i]-a[i])/vel[i]
            mn=np.maximum(mn,np.minimum(f,g));mx=np.minimum(mx,np.maximum(f,g))
    return [names[j] if j<len(names) else 'registered field' for j in np.flatnonzero(mn<=mx)]
"""
    i=src.index('def hits(');j=src.index('delta=8;',i);src=src[:i]+patch+src[j:]
    # A bumper corner in the PARK footprint is the terminal witness. Seek a clear one near original goal.
    i=src.index('delta=8;');src=src[:i]+"""tri=np.array([[220.97,110.845],[220.97,212.405],[133.01,161.63]])
def inzone(test):
    s=[np.linalg.det(np.array([b-a,test-a])) for a,b in zip(tri,np.roll(tri,-1,axis=0))]
    return all(x>0 for x in s) or all(x<0 for x in s)
oldgoal=goal.copy()
choices=[oldgoal+np.array([dx,dy]) for dx in range(-36,37,2) for dy in range(-36,37,2)]
goal=next(g for g in sorted(choices,key=lambda v:np.linalg.norm(v-oldgoal)) if inzone(g+[0,370/25.4]) and not hits(g,g))
"""+src[i:]
    try:exec(compile(src,'all-field PARK','exec'))
    except (StopIteration,AssertionError):
        print('posture rejected',park_angle,flush=True);continue
    out.update(parent=design['step'],postureDeg=park_angle,placedBodies=len(ab),configRectangles=len(arr),goalIn=goal.tolist(),scope='Continuous travel of 153 fixed-heading stock bounds at selected posture, registered field leaf boxes, and the conservative five-peer sweeps on the terminal 24 s. Verify transition into posture at the decision point.')
    dump(parkroot/'park-recovery.json',out);print(park_angle,total,goal,pts,flush=True);break

