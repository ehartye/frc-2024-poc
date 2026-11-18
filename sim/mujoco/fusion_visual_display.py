"""Fusion meshes for display only. Never supplies collision, mass, or mechanism dynamics."""
import hashlib,json,math,pathlib,re,struct,xml.etree.ElementTree as ET
import numpy as np
import mujoco
GROUPS=('fixed','carriage','cradle')
PIVOT=np.array([.178,0,.0254])

def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def stl_bounds(path):
 raw=path.read_bytes()
 if len(raw)<84: raise ValueError('binary STL required')
 n=struct.unpack_from('<I',raw,80)[0]
 if not 0<n<=5000000 or len(raw)!=84+50*n: raise ValueError('invalid binary STL triangle count/length')
 dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
 vertices=np.frombuffer(raw,dtype=dtype,count=n,offset=84)['vertices'].reshape(-1,3)
 if not np.isfinite(vertices).all(): raise ValueError('nonfinite mesh')
 return n,np.array([vertices.min(axis=0),vertices.max(axis=0)],float)

def load_manifest(path,source_archive):
 path=pathlib.Path(path).resolve();m=json.loads(path.read_text(encoding='utf-8-sig'))
 if m.get('schema') not in ('fusion-visual-meshes/1','fusion-visual-meshes/2') or m.get('units')!='mm' or m.get('frame')!='robot-floor-XYZ': raise ValueError('unsupported manifest units/frame/schema')
 if m.get('sourceFormat')!='F3D' or len(m.get('sourceSha256',''))!=64: raise ValueError('source Fusion identity required')
 source_archive=pathlib.Path(source_archive)
 if source_archive.suffix.lower()!='.f3d' or sha(source_archive)!=m['sourceSha256']: raise ValueError('Fusion source archive hash mismatch')
 rows=m.get('meshes',[])
 if m['schema']=='fusion-visual-meshes/1':
  if sorted(x.get('group') for x in rows)!=sorted(GROUPS): raise ValueError('exactly three groups required')
  ids=[i for row in rows for i in row.get('sourceRootIndices',[])]
  if len(ids)!=711 or any(type(i) is not int for i in ids) or sorted(ids)!=list(range(1,712)): raise ValueError('incomplete or duplicate source-root coverage')
  if {r['group']:len(r['sourceRootIndices']) for r in rows}!={'fixed':525,'carriage':152,'cradle':34}: raise ValueError('source group counts mismatch')
 else:
  # V2 preserves each native body and appearance while retaining the same motion groups.
  bodies=m.get('bodyInventory',[]);keys=[b.get('key') for b in bodies]
  if not bodies or any(not isinstance(k,str) or not k for k in keys) or len(set(keys))!=len(keys): raise ValueError('invalid native body inventory')
  if m.get('sourceBodies')!=len(keys): raise ValueError('native body count mismatch')
  roots=[b.get('sourceRootIndex') for b in bodies if b.get('sourceRootIndex') is not None]
  if any(type(i) is not int for i in roots) or set(roots)!=set(range(1,712)): raise ValueError('incomplete native source roots')
  excluded=m.get('excludedBodyKeys',[])
  if not isinstance(excluded,list) or (excluded and not m.get('exclusionReason')): raise ValueError('missing native exclusion evidence')
  coverage=[k for row in rows for k in row.get('sourceBodyKeys',[])]+excluded
  if len(coverage)!=len(keys) or set(coverage)!=set(keys): raise ValueError('incomplete or duplicate native body coverage')
  names=[r.get('name') for r in rows]
  if any(not isinstance(n,str) or not re.fullmatch(r'[a-z][a-z0-9_-]*',n) for n in names) or len(set(names))!=len(names): raise ValueError('invalid mesh names')
  if set(r.get('group') for r in rows)!=set(GROUPS): raise ValueError('invalid motion groups')
  by_key={b['key']:b for b in bodies}
  for row in rows:
   if not row.get('sourceBodyKeys') or any(by_key[k].get('group')!=row['group'] for k in row['sourceBodyKeys']): raise ValueError('native body motion group mismatch')
 out=[]
 for row in rows:
  p=(path.parent/row['file']).resolve()
  if not p.is_relative_to(path.parent) or sha(p)!=row['sha256']: raise ValueError('mesh path/hash mismatch')
  n,b=stl_bounds(p);expected=np.asarray(row['boundsMm'],float)
  if expected.shape!=(2,3) or not np.isfinite(expected).all() or not np.allclose(b,expected,rtol=0,atol=.02) or n!=row['triangles']: raise ValueError('mesh count/bounds mismatch')
  if np.max(np.abs(b))>3000 or np.max(b[1]-b[0])<1: raise ValueError('implausible robot mm extent')
  rgba=np.asarray(row.get('rgba',[.6,.62,.65,1]),float)
  if rgba.shape!=(4,) or not np.isfinite(rgba).all() or np.any(rgba<0) or np.any(rgba>1): raise ValueError('invalid mesh color')
  out.append(dict(row,path=p,actualBoundsMm=b.tolist()))
 return m,out

def poses(chassis_position,chassis_quat,lift=0.,pitch=0.):
 p=np.asarray(chassis_position,float);q=np.asarray(chassis_quat,float)
 if p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-8: raise ValueError('invalid chassis pose')
 if not math.isfinite(lift) or not math.isfinite(pitch) or not 0<=lift<=.449434 or not 0<=pitch<=math.pi/3+1e-9: raise ValueError('unsupported visual mechanism pose')
 R=np.empty(9);mujoco.mju_quat2Mat(R,q);R=R.reshape(3,3)
 qp=np.array([math.cos(pitch/2),0,-math.sin(pitch/2),0]);qc=np.empty(4);mujoco.mju_mulQuat(qc,q,qp)
 Rp=np.array([[math.cos(pitch),0,-math.sin(pitch)],[0,1,0],[math.sin(pitch),0,math.cos(pitch)]])
 z=np.array([0,0,lift]);return {'fixed':(p,q),'carriage':(p+R@z,q),'cradle':(p+R@(PIVOT+z-Rp@PIVOT),qc)}

class FusionDisplay:
 def __init__(self,manifest,source_archive):
  self.manifest,self.meshes=load_manifest(manifest,source_archive)
  root=ET.Element('mujoco',model='Fusion visual-only robot');ET.SubElement(root,'compiler',angle='radian');asset=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody')
  ET.SubElement(world,'light',name='key',pos='1 -1 3',dir='-0.3 0.2 -1',directional='true',diffuse='.85 .85 .85',ambient='.35 .35 .35',specular='.3 .3 .3')
  ET.SubElement(world,'light',name='fill',pos='-2 2 2',dir='.4 -.3 -1',directional='true',diffuse='.5 .5 .5',ambient='.15 .15 .15',castshadow='false')
  ET.SubElement(world,'geom',name='display_floor',type='plane',size='3 3 .1',rgba='.20 .23 .25 1',contype='0',conaffinity='0')
  groups={g:ET.SubElement(world,'body',name=g,mocap='true') for g in GROUPS}
  for row in self.meshes:
   g=row['group'];name=row.get('name',g);ET.SubElement(asset,'mesh',name=name,file=str(row['path']),scale='.001 .001 .001')
   b=groups[g]
   ET.SubElement(b,'geom',name=name,type='mesh',mesh=name,contype='0',conaffinity='0',density='0',rgba=' '.join(map(str,row.get('rgba',[.6,.62,.65,1]))))
  self.model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));self.data=mujoco.MjData(self.model)
  self.ids={g:int(self.model.body(g).mocapid[0]) for g in GROUPS}
  if self.model.nq or self.model.nv or np.any(self.model.geom_contype) or np.any(self.model.geom_conaffinity): raise AssertionError('display isolation failed')
 def update(self,physics,lift=0.,pitch=0.):
  # Read-only mapping from the physically integrated chassis; no qpos assignments to physics.
  b=physics.m.body('chassis').id
  for group,(pos,quat) in poses(physics.d.xpos[b].copy(),physics.d.xquat[b].copy(),lift,pitch).items():
   i=self.ids[group];self.data.mocap_pos[i]=pos;self.data.mocap_quat[i]=quat
  self.data.time=physics.d.time;mujoco.mj_forward(self.model,self.data)
