from layout import *
AAB={n:bounds(s) for n,s in G.items()}
def close(a,b,tol=1e-6):return all(a[i]<=b[i+3]+tol and b[i]<=a[i+3]+tol for i in range(3))
def parked_check():
    collisions=[];unresolved=[];tests=0
    for a,b in itertools.combinations(G,2):
        if not close(AAB[a],AAB[b]):continue
        tests+=1
        try:
            v=common(G[a],G[b])
            if v>.001:collisions.append(dict(pair=[a,b],volume=round(v,4)))
        except Exception as e:unresolved.append([a,b,str(e)])
    return dict(narrow=tests,static=collisions,unresolved=unresolved)
if __name__=='__main__':
    r=parked_check();dump(HERE/'parked.json',r);print(len(G),r,flush=True)

