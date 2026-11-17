"""Real held-command adapter for bounded native XYZ/yaw fixture.
The viewer receives a separate display model/data: GUI reset/perturbation never changes physics.
"""
import argparse,copy,ctypes,hashlib,json,os,pathlib,time
import numpy as np
import mujoco
from native_drive import NativeDrive
HERE=pathlib.Path(__file__).resolve().parent
LIMITS='XYZ/yaw only; roll/pitch locked; native contact chatter; hypothetical mass/tire; fixed12V; not whole CAD.'

def command(keys=(),focused=True,quit_requested=False):
 keys=set(keys)
 if quit_requested or not focused:return ((0.,0.),('brake','brake'),'quit' if quit_requested else 'unfocused')
 if 'B' in keys or 'SPACE' in keys:return ((0.,0.),('brake','brake'),'brake')
 if 'C' in keys:return ((0.,0.),('coast','coast'),'coast')
 forward=int(bool(keys&{'W','UP'}))-int(bool(keys&{'S','DOWN'}));turn=int(bool(keys&{'A','LEFT'}))-int(bool(keys&{'D','RIGHT'}))
 if forward==turn==0:return ((0.,0.),('brake','brake'),'idle-brake')
 left=.3*forward-.2*turn;right=.3*forward+.2*turn
 return ((left,right),('drive','drive'),'drive')

class InteractiveDrive:
 def __init__(self,root):
  self.physics=NativeDrive(root,.0000625);p=self.physics;motor=2*p.c['motorRotorInertiaKgM2']*p.c['gearRatio']**2
  for s in range(2):
   for i in range(3):p.m.dof_armature[p.v[f'wheel_{s}_{i}']]-=motor/3
   p.m.dof_armature[p.v[f'wheel_{s}_1']]+=motor
  mujoco.mj_forward(p.m,p.d);p.initial_energy=float(sum(p.d.energy));self.last=command();self.closed=False
 def tick(self,keys=(),focused=True,quit_requested=False):
  if self.closed:raise ValueError('controller closed')
  self.last=command(keys,focused,quit_requested)
  for _ in range(160):self.physics.step(self.last[0],self.last[1])
  self.closed=quit_requested
  return dict(command=self.last,state=self.physics.observe())

def held_windows():
 """Documented Win32 held-key/focus polling; fail closed outside own GLFW window."""
 from ctypes import wintypes
 u=ctypes.WinDLL('user32',use_last_error=True);u.GetForegroundWindow.restype=wintypes.HWND;u.GetWindowThreadProcessId.argtypes=[wintypes.HWND,ctypes.POINTER(wintypes.DWORD)];u.GetClassNameW.argtypes=[wintypes.HWND,wintypes.LPWSTR,ctypes.c_int];u.GetAsyncKeyState.argtypes=[ctypes.c_int];u.GetAsyncKeyState.restype=ctypes.c_short
 hwnd=u.GetForegroundWindow();pid=wintypes.DWORD();u.GetWindowThreadProcessId(hwnd,ctypes.byref(pid));name=ctypes.create_unicode_buffer(256);u.GetClassNameW(hwnd,name,256)
 focused=pid.value==os.getpid() and name.value.startswith('GLFW')
 mapping={'W':87,'A':65,'S':83,'D':68,'UP':38,'DOWN':40,'LEFT':37,'RIGHT':39,'B':66,'C':67,'SPACE':32,'Q':81,'ESC':27}
 keys={k for k,v in mapping.items() if focused and u.GetAsyncKeyState(v)&0x8000}
 return keys,focused,bool(keys&{'Q','ESC'})

def scripted(root,out):
 out=pathlib.Path(out)
 if out.exists():raise ValueError('fresh output required')
 files=[HERE/'native_drive_interactive.py',HERE/'native_drive.py',HERE/'drivetrain_bridge.py',pathlib.Path(root)/'sim/drivetrain/config.json',pathlib.Path(root)/'sim/drivetrain/plant.py',pathlib.Path(root)/'cad/drivetrain/candidate.json'];pins={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
 a=InteractiveDrive(root);rows=[]
 for label,keys,focus,n in [('settle',[],True,10),('forward',['W'],True,10),('turn',['A'],True,10),('coast',['C'],True,10),('focus-loss',['W'],False,10),('brake',['B'],True,10)]:
  for _ in range(n):rows.append(dict(label=label,**a.tick(keys,focus)))
 rows.append(dict(label='quit',**a.tick(quit_requested=True)))
 if pins!={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files}:raise ValueError('source changed')
 r=dict(schema='native-drive-interactive/1',inputs=pins,dt=.0000625,controllerHz=100,limits=LIMITS,rows=rows);out.write_text(json.dumps(r,indent=2)+'\n');return r

def view(root):
 import mujoco.viewer
 a=InteractiveDrive(root);p=a.physics
 # Display-only copies isolate built-in viewer reset, sliders and mouse forces.
 vm=copy.deepcopy(p.m);vd=mujoco.MjData(vm)
 print(LIMITS,flush=True)
 with mujoco.viewer.launch_passive(vm,vd,show_left_ui=False,show_right_ui=False) as viewer:
  viewer.cam.distance=2.;viewer.cam.elevation=-30
  try:
   while viewer.is_running() and not a.closed:
    start=time.perf_counter();keys,focused,quit_requested=held_windows();a.tick(keys,focused,quit_requested)
    with viewer.lock():
     vd.qpos[:]=p.d.qpos;vd.qvel[:]=p.d.qvel;vd.time=p.d.time;vd.qfrc_applied[:]=0;vd.xfrc_applied[:]=0;mujoco.mj_forward(vm,vd)
    viewer.set_texts((None,None,'WASD/arrows drive | C coast | B/space brake | Q quit\nRelease/focus loss: brake; momentum retained\n'+LIMITS,f't={p.d.time:.2f}s {a.last[2]} duty={a.last[0]}'))
    viewer.sync(state_only=True);time.sleep(max(0,.01-(time.perf_counter()-start)))
  finally:
   if not a.closed:a.tick(quit_requested=True)
   print(json.dumps(dict(event='closed',lastCommand=a.last,state=p.observe(),note='One brake tick on quit; nonzero momentum is not reset.')),flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--source-root',type=pathlib.Path,default=HERE.parents[1]);g=p.add_mutually_exclusive_group(required=True);g.add_argument('--headless',type=pathlib.Path);g.add_argument('--view',action='store_true');a=p.parse_args();scripted(a.source_root,a.headless) if a.headless else view(a.source_root)
