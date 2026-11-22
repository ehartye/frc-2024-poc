from travel import *
def hang(x,z):
    # Fully retracted, empty horizontal cassette. Elevate to the assumed loaded chain
    # and rotate until the given CG hangs directly below its single support.
    roof=650.635+13.335; geom=mat(0,pivot_x(0),0,501)@inverse;sm=mat(0,pivot_x(0)-185,0,0)
    items={n:posegeom(sh,geom if n in moving else sm if n in slide else identity) for n,sh in G.items() if n!=held}
    rows=[]
    for cx,cy,cz in itertools.product((-40,40),(-25,25),(220,350)):
        r=np.array([cx-x,cy,cz-z]);down=r/np.linalg.norm(r)
        # The scalar projection on the suspension line gives each vertex's drop.
        worst=-1e6;name=''
        for n,sh in items.items():
            ab=bounds(sh);v=np.where(down>=0,ab[3:],ab[:3])
            h=np.dot(v-[x,0,z],down)
            if h>worst:worst=h;name=n
        rows.append(dict(cgMm=[cx,cy,cz],lowerClearanceBoundMm=roof-worst,body=name))
    return rows
if __name__=='__main__':
    for x,z in [(-296,625),(-55,545),(-15,505)]:
        rows=hang(x,z);print(x,z,min(r['lowerClearanceBoundMm'] for r in rows))

