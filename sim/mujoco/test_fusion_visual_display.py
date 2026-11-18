import unittest,tempfile,pathlib,json,struct,hashlib
import numpy as np
from fusion_visual_display import poses,FusionDisplay,load_manifest
from native_drive_interactive import InteractiveDrive

class VisualTests(unittest.TestCase):
 def fixture(self,d):
  rows=[]
  (d/'synthetic.f3d').write_bytes(b'synthetic identity only, not a Fusion archive')
  # Synthetic tetrahedra exercise parsing/compilation only, never robot-delivery evidence.
  v=np.array([[0,0,0],[20,0,0],[0,20,0],[0,0,20]],dtype=float)
  for group in ('fixed','carriage','cradle'):
   raw=b'synthetic-test-only'.ljust(80,b' ')+struct.pack('<I',4)
   for face in ((0,2,1),(0,1,3),(0,3,2),(1,2,3)):
    raw+=struct.pack('<12fH',0,0,0,*v[list(face)].ravel(),0)
   p=d/(group+'.stl');p.write_bytes(raw)
   indices={'fixed':list(range(1,526)),'carriage':list(range(526,678)),'cradle':list(range(678,712))}[group]
   rows.append(dict(group=group,sourceRootIndices=indices,file=p.name,sha256=hashlib.sha256(raw).hexdigest(),triangles=4,boundsMm=[[0,0,0],[20,20,20]]))
  m=dict(schema='fusion-visual-meshes/1',units='mm',frame='robot-floor-XYZ',sourceFormat='F3D',sourceSha256=hashlib.sha256((d/'synthetic.f3d').read_bytes()).hexdigest(),meshes=rows)
  p=d/'manifest.json';p.write_text(json.dumps(m));return p
 def fixture_v2(self,d):
  path=self.fixture(d);m=json.loads(path.read_text());m['schema']='fusion-visual-meshes/2';bodies=[]
  for row in m['meshes']:
   row['name']=row['group'];row['sourceBodyKeys']=[]
   for i in row.pop('sourceRootIndices'):
    key='root-'+str(i);bodies.append(dict(key=key,sourceRootIndex=i,group=row['group']));row['sourceBodyKeys'].append(key)
  added=dict(m['meshes'][0]);added.update(name='bumper-red',sourceBodyKeys=['native-bumper'],rgba=[.84,.1,.13,1]);m['meshes'].append(added)
  bodies.append(dict(key='native-bumper',sourceRootIndex=None,group='fixed'));m['bodyInventory']=bodies;m['sourceBodies']=len(bodies)
  path.write_text(json.dumps(m));return path
 def test_v2_color_segments(self):
  with tempfile.TemporaryDirectory() as td:
   d=pathlib.Path(td);path=self.fixture_v2(d);view=FusionDisplay(path,d/'synthetic.f3d')
   self.assertEqual(view.model.nmocap,3);self.assertEqual(len(view.meshes),4)
   np.testing.assert_allclose(view.model.geom('bumper-red').rgba,[.84,.1,.13,1],atol=1e-7)
 def test_v2_rejects_bad_coverage_and_color(self):
  for change in ('duplicate','missing','wrong-group','color'):
   with tempfile.TemporaryDirectory() as td:
    d=pathlib.Path(td);path=self.fixture_v2(d);m=json.loads(path.read_text())
    if change=='duplicate':m['meshes'][-1]['sourceBodyKeys']=['root-1']
    elif change=='missing':m['bodyInventory'][0]['sourceRootIndex']=None
    elif change=='wrong-group':m['meshes'][-1]['group']='cradle'
    else:m['meshes'][-1]['rgba']=[float('nan'),0,0,1]
    path.write_text(json.dumps(m))
    with self.assertRaises(ValueError):load_manifest(path,d/'synthetic.f3d')
 def test_v2_excluded_body_keeps_source_inventory(self):
  with tempfile.TemporaryDirectory() as td:
   d=pathlib.Path(td);path=self.fixture_v2(d);m=json.loads(path.read_text())
   m['meshes'][0]['sourceBodyKeys'].remove('root-1')
   m['excludedBodyKeys']=['root-1'];m['exclusionReason']='Hidden reference body retained in native archive'
   path.write_text(json.dumps(m));view=FusionDisplay(path,d/'synthetic.f3d')
   self.assertEqual(view.manifest['sourceBodies'],712)
   self.assertEqual(sum(len(r['sourceBodyKeys']) for r in view.meshes),711)
   self.assertEqual(view.manifest['excludedBodyKeys'],['root-1'])
 def test_v2_rejects_unaccounted_exclusions(self):
  for change in ('no-reason','duplicate','unknown','still-rendered'):
   with self.subTest(change=change),tempfile.TemporaryDirectory() as td:
    d=pathlib.Path(td);path=self.fixture_v2(d);m=json.loads(path.read_text())
    if change!='still-rendered':m['meshes'][0]['sourceBodyKeys'].remove('root-1')
    m['excludedBodyKeys']=['root-1'];m['exclusionReason']='Hidden native reference'
    if change=='no-reason':m.pop('exclusionReason')
    elif change=='duplicate':m['excludedBodyKeys'].append('root-1')
    elif change=='unknown':m['excludedBodyKeys']=['unknown-body']
    path.write_text(json.dumps(m))
    with self.assertRaises(ValueError):load_manifest(path,d/'synthetic.f3d')
 def test_v2_verification_checks_every_mesh(self):
  import contextlib,io
  from verify_fusion_visual_display import run
  with tempfile.TemporaryDirectory() as td:
   d=pathlib.Path(td);path=self.fixture_v2(d);out=d/'verification.json'
   with contextlib.redirect_stdout(io.StringIO()):run(path,d/'synthetic.f3d',out)
   result=json.loads(out.read_text());names={'fixed','carriage','cradle','bumper-red'}
   self.assertEqual(set(result['meshHashes']),names)
   for pose in ('floor','rotated','handoff'):
    self.assertEqual({r['mesh'] for r in result['registrationChecks'] if r['pose']==pose},names)
   self.assertEqual(len(result['registrationChecks']),12)
   self.assertTrue(result['physicalThirtyTickRowsExactlyEqual'])
 def test_drive_shortcuts_preserve_floor_and_solid_rendering(self):
  import mujoco
  from types import SimpleNamespace
  from fusion_visual_drive import restore_drive_display_options
  viewer=SimpleNamespace(opt=mujoco.MjvOption(),user_scn=mujoco.MjvScene())
  viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_STATIC]=0
  viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT]=1
  viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME]=1
  viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=0
  restore_drive_display_options(viewer)
  self.assertEqual(viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_STATIC],1)
  self.assertEqual(viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT],1)
  self.assertEqual(viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME],0)
  self.assertEqual(viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW],1)
  viewer.user_scn=None;restore_drive_display_options(viewer)
 def test_transform(self):
  pp=poses([1,2,3],[1,0,0,0],.2,np.pi/3)
  point=np.array([.3,.04,.1]);pos,q=pp['cradle']
  import mujoco
  R=np.empty(9);mujoco.mju_quat2Mat(R,q)
  actual=pos+R.reshape(3,3)@point
  expected=np.array([1+.178+.5*(.3-.178)-np.sin(np.pi/3)*(.1-.0254),2+.04,3+.0254+.2+np.sin(np.pi/3)*(.3-.178)+.5*(.1-.0254)])
  np.testing.assert_allclose(actual,expected,atol=1e-14)
 def test_isolation(self):
  with tempfile.TemporaryDirectory() as td:
   view=FusionDisplay(self.fixture(pathlib.Path(td)),pathlib.Path(td)/'synthetic.f3d')
   a=InteractiveDrive(pathlib.Path(__file__).resolve().parents[2]);p=a.physics
   a.tick(['W']);state=[p.d.qpos.copy(),p.d.qvel.copy(),p.m.body_mass.copy(),p.m.dof_armature.copy(),p.d.qfrc_applied.copy()];work=p.work
   view.update(p,.1,.2);view.data.mocap_pos[:]=99
   for before,after in zip(state,[p.d.qpos,p.d.qvel,p.m.body_mass,p.m.dof_armature,p.d.qfrc_applied]):np.testing.assert_array_equal(before,after)
   self.assertEqual(work,p.work);self.assertEqual(view.model.nv,0)
 def test_hash_and_units(self):
  with tempfile.TemporaryDirectory() as td:
   d=pathlib.Path(td);path=self.fixture(d);m=json.loads(path.read_text());m['units']='cm';path.write_text(json.dumps(m))
   with self.assertRaises(ValueError):load_manifest(path,path.parent/'synthetic.f3d')
   path=self.fixture(d);(d/'fixed.stl').write_bytes(b'invalid')
   with self.assertRaises(ValueError):load_manifest(path,path.parent/'synthetic.f3d')
 def test_bounds(self):
  with tempfile.TemporaryDirectory() as td:
   path=self.fixture(pathlib.Path(td));m=json.loads(path.read_text());m['meshes'][0]['boundsMm'][1][0]=200;path.write_text(json.dumps(m))
   with self.assertRaises(ValueError):load_manifest(path,path.parent/'synthetic.f3d')
 def test_missing_root(self):
  with tempfile.TemporaryDirectory() as td:
   path=self.fixture(pathlib.Path(td));m=json.loads(path.read_text());m['meshes'][0]['sourceRootIndices'].remove(43);path.write_text(json.dumps(m))
   with self.assertRaises(ValueError):load_manifest(path,path.parent/'synthetic.f3d')
 def test_invalid_pose(self):
  for lift,pitch in [(float('nan'),0),(-.1,0),(0,2)]:
   with self.assertRaises(ValueError):poses([0,0,0],[1,0,0,0],lift,pitch)
if __name__=='__main__':unittest.main()
