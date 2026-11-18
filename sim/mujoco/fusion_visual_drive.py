"""Drive physics with an isolated Fusion-mesh display. Mechanisms held in floor pose.
No mesh participates in collision or inertia; this is not a full-robot dynamics model.
"""
import argparse,hashlib,json,pathlib,time
import mujoco
from native_drive_interactive_v3 import InteractiveDrive,InputLatch,RunTiming,held_windows,record,HERE
from fusion_visual_display import FusionDisplay,sha
from fusion_visual_provenance import inventory,verify_inventory
LIMITS='Fusion geometry: VISUAL ONLY, mechanisms held at floor. Physics: XYZ/yaw six-wheel drive; fixed12V; hypothetical tires/mass; contact chatter.'

def restore_drive_display_options(viewer):
 # MuJoCo processes its own shortcuts as well as the drive callback. Restore only
 # conflicting display flags; camera controls and unrelated view options remain usable.
 keys=set('WASDCBQ')
 for i,(_,default,key) in enumerate(mujoco.mjVISSTRING):
  if key in keys:viewer.opt.flags[i]=int(default)
 if viewer.user_scn is not None:
  for i,(_,default,key) in enumerate(mujoco.mjRNDSTRING):
   if key in keys:viewer.user_scn.flags[i]=int(default)

def view(root,manifest,source_archive,trace):
 import mujoco.viewer
 pins=inventory(HERE,root)
 a=InteractiveDrive(root);display=FusionDisplay(manifest,source_archive);latch=InputLatch();display.update(a.physics)
 def callback(code):
  _,focused,_=held_windows();latch.event(code,focused)
 with pathlib.Path(trace).open('x',encoding='utf-8',buffering=1) as f:
  f.write(json.dumps(dict(event='start',schema='fusion-drive-visual/1',limits=LIMITS,manifestSha256=sha(pathlib.Path(manifest)),fusionSha256=sha(pathlib.Path(source_archive)),runtimeInputPins=pins))+'\n')
  with mujoco.viewer.launch_passive(display.model,display.data,key_callback=callback,show_left_ui=False,show_right_ui=False) as viewer:
   viewer.cam.distance=2.5;viewer.cam.elevation=-25;viewer.cam.lookat[:]=[0,0,.4];timing=RunTiming(a.physics.d.time)
   try:
    while viewer.is_running() and not a.closed:
     start=time.monotonic();held,focused,_=held_windows();keys,quit_requested,telemetry=latch.poll(held,focused,start);a.tick(keys,focused,quit_requested);row=record(a,telemetry,timing=timing);f.write(json.dumps(row)+'\n')
     with viewer.lock():
      display.update(a.physics)
      restore_drive_display_options(viewer)
     viewer.set_texts((None,None,'WASD drive | C coast | B brake | Q quit\n'+LIMITS,f'sim={a.physics.d.time:.2f}s wall={row["wallElapsedS"]:.2f}s RTF={row["realTimeFactor"]:.3f}\n{a.last[2]} duty={a.last[0]} events={telemetry["eventCount"]} accepted={telemetry["acceptedEventCount"]}'))
     viewer.sync(state_only=True)
     with viewer.lock():restore_drive_display_options(viewer)
     time.sleep(max(0,.01-(time.monotonic()-start)))
   finally:
    if not a.closed:a.tick(quit_requested=True)
    verify_inventory(pins,HERE,root)
    _,_,telemetry=latch.poll([],False);f.write(json.dumps(record(a,telemetry,'closed',timing))+'\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--source-root',type=pathlib.Path,default=HERE.parents[1]);p.add_argument('--manifest',type=pathlib.Path,required=True);p.add_argument('--fusion-archive',type=pathlib.Path,required=True);p.add_argument('--trace',type=pathlib.Path,required=True);a=p.parse_args();view(a.source_root,a.manifest,a.fusion_archive,a.trace)
