"""Complete private Speaker leaf inventory with shared, local tessellations."""
import hashlib
import json
import sys
from pathlib import Path
import numpy as np
from OCP.TopAbs import TopAbs_REVERSED
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from inspect_solids import read_shape,faces

root=Path(sys.argv[1])
parts=json.loads((root/'blue-speaker/panels.json').read_bytes())['parts']
covered={p['name'] for p in parts}
rows=json.loads((root/'cad-extraction/occurrences.json').read_bytes())
basis=np.array([[-1.,0.,0.],[0.,0.,1.],[0.,1.,0.]])
origin=np.array([8270.875,4105.275,-4.7752])
assets={}
dedup={}
extras=[]
for i,row in enumerate(rows):
    if 'GE-24300_Briweld Field Blue' not in row['path'] or row['children'] or row['path'][-1] in covered:continue
    box=np.asarray(row['boundsCadMm']).reshape(2,3)
    box=(np.stack([-box[::-1,0],box[:,2],box[:,1]],axis=1)+origin)/1000
    name=row['brepLocal']
    if name not in dedup:
        key='m'+str(len(dedup))
        shape=read_shape(name)
        mesher=BRepMesh_IncrementalMesh(shape,.18,False,.16,False)
        assert mesher.IsDone()
        vertices=[];triangles=[]
        for face in faces(shape):
            loc=TopLoc_Location()
            mesh=BRep_Tool.Triangulation_s(face,loc)
            assert mesh is not None
            start=len(vertices)-1
            vertices.extend(mesh.Node(j).Transformed(loc.Transformation()).Coord() for j in range(1,mesh.NbNodes()+1))
            for j in range(1,mesh.NbTriangles()+1):
                a,b,c=mesh.Triangle(j).Get()
                if face.Orientation()==TopAbs_REVERSED:b,c=c,b
                triangles.append([a+start,b+start,c+start])
        assets[key+'_v']=np.array(vertices)/1000
        assets[key+'_t']=np.array(triangles,dtype=np.int32)
        dedup[name]=key
    mat=np.asarray(row['fieldCadFromPart'])
    extras.append(dict(key='e'+str(i),mesh=dedup[name],name=row['path'][-1],rotation=(basis@mat[:,:3]).tolist(),
        translation=((basis@mat[:,3]+origin)/1000).tolist(),blueFieldBoundsM=box.tolist()))
result=root/'blue-speaker/extras.npz'
np.savez_compressed(result,**assets)
with result.open('rb') as f:h=hashlib.file_digest(f,'sha256').hexdigest()
(root/'blue-speaker/extras.json').write_text(json.dumps(dict(mesh=result.name,meshSha256=h,parts=extras)))
print(len(extras),'occurrences',len(dedup),'distinct meshes',h,flush=True)
