"""Additive v3: callback tap latch + held polling, isolated display, live telemetry.
A tap commands a 100ms wall-clock pulse; held polling extends it. Focus loss
clears pulses on the next 100Hz controller tick. WASD tap input verified in v2; arrow callbacks are -1 and not mapped.
Arrow held polling remains available but has not been UI-verified.
"""
import argparse,copy,hashlib,json,pathlib,threading,time
import mujoco
from native_drive_interactive import InteractiveDrive,held_windows,LIMITS,HERE
KEYS={87:'W',65:'A',83:'S',68:'D',66:'B',67:'C',32:'SPACE',81:'Q',256:'ESC'}
class InputLatch:
 def __init__(self,pulse=.1):
  self.pulse=pulse;self.until={};self.lock=threading.Lock();self.events=0;self.accepted=0;self.last=None
 def event(self,code,focused,now=None):
  now=time.monotonic() if now is None else now
  with self.lock:
   self.events+=1;name=KEYS.get(code);self.last=dict(code=code,key=name,focused=bool(focused),wallMonotonic=now)
   if focused and name:
    self.accepted+=1;self.until[name]=max(self.until.get(name,now),now+self.pulse)
 def poll(self,held,focused,now=None):
  now=time.monotonic() if now is None else now
  with self.lock:
   if not focused:self.until.clear()
   else:self.until={k:v for k,v in self.until.items() if v>now}
   active=(set(held)|set(self.until)) if focused else set()
   return active, bool(active&{'Q','ESC'}),dict(eventCount=self.events,acceptedEventCount=self.accepted,lastInput=self.last,latchedKeys=sorted(self.until),heldKeys=sorted(held),focused=bool(focused))

class RunTiming:
 def __init__(self,sim_start=0.,now=None):
  self.start=time.monotonic() if now is None else now;self.sim_start=sim_start;self.wall_epoch=time.time()
 def sample(self,sim_time,now=None):
  now=time.monotonic() if now is None else now;elapsed=max(0.,now-self.start);sim_elapsed=sim_time-self.sim_start
  return dict(wallTimeUnixS=self.wall_epoch+elapsed,wallElapsedS=elapsed,simulatedElapsedS=sim_elapsed,realTimeFactor=sim_elapsed/elapsed if elapsed>0 else None)

def record(a,telemetry,event='tick',timing=None):
 p=a.physics
 return dict(**(timing.sample(p.d.time) if timing else {}),event=event,simTime=p.d.time,command=a.last,input=telemetry,qpos=p.d.qpos.tolist(),qvel=p.d.qvel.tolist(),motorAxleWorkJ=p.work,verticalLoadsN=p.observe()['wheelVerticalLoadsN'])

def view(root,trace):
 import mujoco.viewer
 a=InteractiveDrive(root);p=a.physics;latch=InputLatch();vm=copy.deepcopy(p.m);vd=mujoco.MjData(vm)
 def callback(code):
  _,focused,_=held_windows();latch.event(code,focused)
 trace=pathlib.Path(trace)
 with trace.open('x',encoding='utf-8',buffering=1) as f:
  f.write(json.dumps(dict(event='start',schema='native-interactive-v3/1',limits=LIMITS,tapPulseSeconds=.1,dt=.0000625,controllerHz=100,sourceSha256={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in ['native_drive_interactive_v3.py','native_drive_interactive.py','native_drive.py','drivetrain_bridge.py']}))+'\n')
  with mujoco.viewer.launch_passive(vm,vd,key_callback=callback,show_left_ui=False,show_right_ui=False) as viewer:
   viewer.cam.distance=2;viewer.cam.elevation=-30;timing=RunTiming(p.d.time)
   try:
    while viewer.is_running() and not a.closed:
     start=time.monotonic();held,focused,_=held_windows();keys,quit_requested,telemetry=latch.poll(held,focused,start);a.tick(keys,focused,quit_requested);row=record(a,telemetry,timing=timing);f.write(json.dumps(row)+'\n')
     with viewer.lock():
      vd.qpos[:]=p.d.qpos;vd.qvel[:]=p.d.qvel;vd.time=p.d.time;vd.qfrc_applied[:]=0;vd.xfrc_applied[:]=0;mujoco.mj_forward(vm,vd)
      if viewer.user_scn is not None:viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME]=0
     viewer.set_texts((None,None,'WASD: drive (arrows: held polling only, unverified) | C coast | B brake | Q quit\nTap=100ms pulse; hold extends; focus loss clears and brakes\n'+LIMITS,f'sim={p.d.time:.2f}s wall={row["wallElapsedS"]:.2f}s RTF={row["realTimeFactor"]:.3f} {a.last[2]} {a.last[0]}\nevents={telemetry["eventCount"]} accepted={telemetry["acceptedEventCount"]} keys={sorted(keys)} focused={focused}'))
     viewer.sync(state_only=True)
     with viewer.lock():
      if viewer.user_scn is not None:viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME]=0
     time.sleep(max(0,.01-(time.monotonic()-start)))
   finally:
    if not a.closed:a.tick(quit_requested=True)
    _,_,telemetry=latch.poll([],False);final=record(a,telemetry,'closed',timing);f.write(json.dumps(final)+'\n');print(json.dumps(final),flush=True)

def headless(root,out):
 a=InteractiveDrive(root);latch=InputLatch();rows=[];timing=RunTiming()
 for i in range(10):a.tick()
 start=a.physics.d.qpos.copy();latch.event(87,True,0.)
 for i in range(15):
  keys,quit,meta=latch.poll([],True,i*.01);a.tick(keys,True,quit);rows.append(record(a,meta,timing=timing))
 # Held extends after pulse; losing focus kills a newly queued pulse immediately at poll.
 latch.event(87,True,1.);keys,quit,meta=latch.poll(['W'],False,1.001);a.tick(keys,False,quit);rows.append(record(a,meta,'focus-loss',timing))
 a.tick(quit_requested=True);rows.append(record(a,meta,'closed',timing))
 out=pathlib.Path(out)
 with out.open('x',encoding='utf-8') as f:json.dump(dict(schema='native-interactive-v3-headless/1',startQpos=start.tolist(),rows=rows,limitations='Injected callback/held events only; not actual OS/viewer input verification.',pins={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in ['native_drive_interactive_v3.py','native_drive_interactive.py','native_drive.py','test_native_drive_interactive_v3.py']}),f,indent=2)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--source-root',type=pathlib.Path,default=HERE.parents[1]);p.add_argument('--trace',type=pathlib.Path);g=p.add_mutually_exclusive_group(required=True);g.add_argument('--view',action='store_true');g.add_argument('--headless',type=pathlib.Path);a=p.parse_args()
 if a.view:
  if a.trace is None:p.error('--view requires fresh --trace path')
  view(a.source_root,a.trace)
 else:headless(a.source_root,a.headless)
