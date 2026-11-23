from pathlib import Path
src=Path(__file__).with_name('travel.py')
exec(compile(src.read_text().split('results=[];near=[]')[0],str(src),'exec'))
broad=json.loads((p/'travel-broad.json').read_bytes());pairs=[]
for a,c in broad['candidates']:
    assert role(a)=='slide' and role(c)=='arm'
    radius=max(math.hypot(x-185,z-501) for x,z in itertools.product(b[c][[0,3]],b[c][[2,5]]))
    leaves=[];unresolved=[];witness=[]
    def check(lo,hi):
        mid=(lo+hi)/2;m=mat(mid-125,185,0,501)@mat(0,-185,0,-501)
        shape=place(G[c],m);dist=distance(G[a],shape)
        lower=dist-radius*math.radians((hi-lo)/2)
        if lower>.01:leaves.append(lower);return
        if hi-lo<.05:
            v=common(G[a],shape);unresolved.append([lo,hi,dist,v]);return
        check(lo,mid);check(mid,hi)
    check(-50.5,165);pairs.append(dict(pair=[a,c],certifiedIntervals=len(leaves),minLowerMm=min(leaves) if leaves else None,unresolved=unresolved))
print([(x['pair'],len(x['unresolved'])) for x in pairs],flush=True)
# In both service cycles the hardware is carried with the truck or cassette. Compare
# transformed fixed-pose gauges with the actual extracted articles on every interval.
serv=[]
for case in (c for c in d['cases'] if c['key'] in ('service','supported-service')):
    def frame(n,i):
        fs=case['frames'].get(n)
        return np.vstack([np.array(fs[i]).reshape(3,4),[0,0,0,1]]) if fs else np.eye(4)
    for a in stock:
        carrier='Cassette cheek · left' if role(a)=='arm' else 'Translating bearing block · left'
        for c in ['Battery and terminal case','Battery service pin','Service index disk']:
            if c in case['removed']:continue
            lows=[];unchecked=[]
            for i in range(len(case['positions'])-1):
                T0=np.linalg.inv(frame(carrier,i))@frame(c,i);T1=np.linalg.inv(frame(carrier,i+1))@frame(c,i+1)
                radius=max(np.linalg.norm(v) for v in itertools.product(*zip(b[c][:3],b[c][3:])))
                angle=math.acos(np.clip((np.trace(T1[:3,:3]@T0[:3,:3].T)-1)/2,-1,1))
                travel=np.linalg.norm(T1[:3,3]-T0[:3,3])+radius*(angle+1e-6)
                shape=place(G[c],T0);lo=distance(G[a],shape)-travel
                if lo<.001:unchecked.append([i,lo,common(G[a],shape)])
                lows.append(lo)
            serv.append(dict(case=case['key'],pair=[a,c],boundMm=min(lows),unchecked=unchecked))
out=dict(rotation=pairs,service=serv);(p/'travel-detail.json').write_text(json.dumps(out,separators=(',',':')))
print('service requiring finer checks',[(s['pair'],s['case'],len(s['unchecked'])) for s in serv if s['unchecked']])

