from travel import *
# Find a load point admissible for the measured existing cassette, then return
# the worst grounded-envelope bound across the eight CG cases for each placement.
keep=[(n,g) for n,g in G.items() if meta[n]['group'] not in ('Solo climber','Capture latch') and n!=held]
cgs=np.array(list(itertools.product((-40.,40.),(-25.,25.),(220.,350.))))
res=[]
for theta in range(135,181,5):
    a=radians(theta); arm=mat(theta,pivot_x(theta),0,501)@inverse
    st=np.array([bounds(posegeom(sh,arm if n in moving else identity)) for n,sh in keep])
    for s in range(70,261,10):
        for h in range(40,261,10):
            radius=hypot(s,h) if False else np.hypot(s,h)
            if radius<247:continue
            x=185+s*cos(a)-h*sin(a);z=501+s*sin(a)+h*cos(a)
            if z<370 or z>610:continue
            pos=np.array([x,0,z]);d=cgs-pos;d/=np.linalg.norm(d,axis=1)[:,None]
            boxes=np.where(d[:,None,:]>=0,st[None,:,3:],st[None,:,:3]);clear=663.97-np.max(np.sum((boxes-pos)*d[:,None,:],axis=2))
            res.append((round(float(clear),2),theta,s,h,round(x),round(z),round(radius)))
print(sorted(res,reverse=True)[:10]);good=[v for v in res if v[0]>12 and v[3]<210];print('lower',sorted(good,key=lambda q:(q[3],-q[0]))[:6]);dump(HERE/'hook-trade.json',sorted(res,reverse=True)[:30])

