"""Reproducible actual-Fusion display checks; no physical validation claim."""
import argparse,hashlib,json,pathlib,time,struct,zlib
import numpy as np
import mujoco
from fusion_visual_display import FusionDisplay,poses,sha
from native_drive_interactive import InteractiveDrive,HERE
from fusion_visual_provenance import inventory,verify_inventory

def run(manifest,archive,out,image=None):
 source_root=HERE.parents[1];pins=inventory(HERE,source_root)
 start=time.perf_counter();v=FusionDisplay(manifest,archive);a=InteractiveDrive(HERE.parents[1]);ref=InteractiveDrive(HERE.parents[1])
 before={n:getattr(a.physics.m,n).copy() for n in ['body_mass','body_inertia','dof_armature']};rows=[]
 for i in range(30):
  keys=['W'] if i<10 else ['A'] if i<20 else ['B'];a.tick(keys);ref.tick(keys);v.update(a.physics)
  for n in ['qpos','qvel','qfrc_applied']:np.testing.assert_array_equal(getattr(a.physics.d,n),getattr(ref.physics.d,n))
  assert a.physics.work==ref.physics.work
  rows.append(dict(time=a.physics.d.time,qpos=a.physics.d.qpos.tolist(),motorWorkJ=a.physics.work))
 for n,x in before.items():np.testing.assert_array_equal(x,getattr(a.physics.m,n))
 # Check all source meshes' world bounds against MuJoCo's compiled mesh-frame handling.
 checks=[]
 for label,lift,pitch in [('floor',0,0),('rotated',.131982271537,np.pi/3),('handoff',.449433394924,np.pi/3)]:
  v.update(a.physics,lift,pitch);b=a.physics.m.body('chassis').id;targets=poses(a.physics.d.xpos[b],a.physics.d.xquat[b],lift,pitch)
  for row in v.meshes:
   group=row['group'];raw=row['path'].read_bytes();dt=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')]);vertices=np.frombuffer(raw,dtype=dt,count=row['triangles'],offset=84)['vertices'].reshape(-1,3).astype(float)*.001
   pos,q=targets[group];R=np.empty(9);mujoco.mju_quat2Mat(R,q);expected=vertices@R.reshape(3,3).T+pos
   name=row.get('name',group);gi=v.model.geom(name).id;mi=int(v.model.geom_dataid[gi]);adr=int(v.model.mesh_vertadr[mi]);num=int(v.model.mesh_vertnum[mi]);actual=v.model.mesh_vert[adr:adr+num].astype(float)@v.data.geom_xmat[gi].reshape(3,3).T+v.data.geom_xpos[gi]
   error=float(np.max(np.abs(np.array([actual.min(0),actual.max(0)])-np.array([expected.min(0),expected.max(0)]))))
   if error>2e-7:raise AssertionError(('compiled mesh frame',label,group,error))
   checks.append(dict(pose=label,group=group,mesh=name,maxBoundsErrorM=error))
 v.update(a.physics)
 if image:
  v.model.vis.global_.offwidth=960;v.model.vis.global_.offheight=720
  with mujoco.Renderer(v.model,height=720,width=960) as renderer:
   cam=mujoco.MjvCamera();cam.lookat[:]=[.05,0,.45];cam.distance=2.25;cam.azimuth=135;cam.elevation=-25;renderer.update_scene(v.data,camera=cam);rgb=renderer.render(); h,w,_=rgb.shape
   def chunk(tag,data):return struct.pack('>I',len(data))+tag+data+struct.pack('>I',zlib.crc32(tag+data)&0xffffffff)
   pathlib.Path(image).write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(b''.join(b'\0'+row.tobytes() for row in rgb)))+chunk(b'IEND',b''))
 verify_inventory(pins,HERE,source_root)
 result=dict(schema='fusion-display-verification/2',runtimeInputPins=pins,environment=dict(mujoco=mujoco.__version__,numpy=np.__version__),registrationScope='Internal mesh-frame checks using the shared poses() helper; not independent Fusion mechanism-pose agreement',sourceFusionSha256=sha(pathlib.Path(archive)),manifestSha256=sha(pathlib.Path(manifest)),meshHashes={r.get('name',r['group']):sha(r['path']) for r in v.meshes},triangles=int(v.model.nmeshface),displayDofs=int(v.model.nv),displayCollisionEnabled=bool(np.any(v.model.geom_contype)|np.any(v.model.geom_conaffinity)),physicalThirtyTickRowsExactlyEqual=True,registrationChecks=checks,trace=rows,elapsedSeconds=time.perf_counter()-start,scope='Source-bound meshes on separate display; synthetic fixtures do not establish native export evidence; mechanisms prescribed registration only, wheels static visually; no whole-robot physical validation.')
 if image:result['imageSha256']=sha(pathlib.Path(image))
 with pathlib.Path(out).open('x',encoding='utf-8') as f:json.dump(result,f,indent=2)
 print(json.dumps({k:r for k,r in result.items() if k not in ['trace','sourcePins','registrationChecks']},indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--manifest',required=True);p.add_argument('--archive',required=True);p.add_argument('--out',required=True);p.add_argument('--image');a=p.parse_args();run(a.manifest,a.archive,a.out,a.image)
