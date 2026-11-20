"""Make the student's plan section directly from the checked mesh and match receipts."""
import json
from contact import meshes, HERE, ROOT


def section(triangles):
    segments=[]
    z=25.401  # avoid vertices exactly coplanar with the section
    for tri in triangles:
        points=[]
        for a,b in zip(tri,tri[[1,2,0]]):
            if (a[2]<z) != (b[2]<z):
                f=(z-a[2])/(b[2]-a[2])
                points.extend(round(float(x),2) for x in a[:2]+f*(b[:2]-a[:2]))
        if len(points)==4:
            segments.append(points)
    return segments


def build():
    groups={name:section(ts) for name,ts in meshes() if name in ("534","535") or
            name.startswith(("R7 504 ","R7 519 ","R7 510 ","R7 525 ","R7 536 ","R7 538 "))}
    payload={"section":groups,"results":json.loads((HERE/"results.json").read_bytes()),
             "loads":json.loads((HERE/"loads.json").read_bytes())[-1]}
    (ROOT/"docs/ground-contact.json").write_bytes((json.dumps(payload,separators=(",",":"))+"\n").encode())


if __name__=="__main__":
    build()
