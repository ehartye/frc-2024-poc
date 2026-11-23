"""Continue the free floor NOTE through a driven pickup and a commanded rotating cassette."""
from pathlib import Path
import sys,json,math
from stroke import scheduled
folder=Path(__file__).resolve().parent
label,n=sys.argv[1:3];resp=json.loads((folder/'response.json').read_bytes())['nominal']
sys.argv=[__file__,label,n,str(resp['final'][0]+50),'0','0','0','.45','8']
source=(folder.parent/'bounded-risk-01/pickup.py').read_text()
patch=r"""
sub('round(2.82/dt)','round(7.0/dt)')
sub('fixed+tape,f\'euler=', '\'<joint name="elevation" type="hinge" axis="0 -1 0" /><inertial pos="0 0 0" mass="5.5" diaginertia=".4 2.6 2.6" />\'+fixed+tape,f\'euler=')
sub('<joint name="elevation" type="hinge"',f'<joint name="truck" type="slide" axis="{m.cos(m.radians(deploy))} 0 {-m.sin(m.radians(deploy))}" /><joint name="elevation" type="hinge"')
sub("arm=body(f'{X} 0 {Z}'","arm=body(f'{initialTruckX} 0 {Z}'")
sub('t=step*dt', '''t=step*dt
        u=max(0.,min(1.,(t-2.82)/3.4));blend=u*u*(3-2*u);rate=6*u*(1-u)/3.4
        theta=m.radians(deploy+(125-deploy)*blend);w=m.radians(125-deploy)*rate
        ej=jid('elevation');d.qpos[model.jnt_qposadr[ej]]=theta-m.radians(deploy);d.qvel[model.jnt_dofadr[ej]]=w
        C=m.cos(theta);S=m.sin(theta)
        # Horizontal slide and hinge share an arm parent. The truck coordinate is a world X DOF.
        robotX=g['pickPivot'][0]/1000
        start=scheduled(deploy)
        delta=(scheduled(m.degrees(theta))-start)*(1-blend)+(scheduled(m.degrees(theta))-robotX*1000)*blend
        X=robotX+delta/1000
        qd=m.degrees(theta);degvel=m.degrees(w)
        vx=((scheduled(qd+.01)-scheduled(qd-.01))/.02*degvel+(start-robotX*1000)*rate)/1000
        tj=jid('truck');d.qpos[model.jnt_qposadr[tj]]=delta/1000;d.qvel[model.jnt_dofadr[tj]]=vx
        if t>=2.82:commanded=0.''')
sub('-[d.qvel[model.jnt_dofadr[drive]],0,0]', '-[d.qvel[model.jnt_dofadr[drive]],0,0]-np.cross([0,-w,0],pts[i]-[X+advance,0,Z])-[vx,0,0]')
sub('fm=mat(-angle,','fm=mat(m.degrees(theta),')
sub('return dict(minimumContactSpeedMps','return dict(endPieceQpos=d.qpos[qi:qi+7].tolist(),endPieceQvel=d.qvel[v0:v0+6].tolist(),endBeltSpeed=speed,endFlapRad=d.qpos[model.jnt_qposadr[jid("flap")]],minimumContactSpeedMps')
"""
# Apply modifications to the assembled MuJoCo simulation before compiling it.
initialTruckX=g0=json.loads((folder.parent/'pickup-lip/layout.json').read_bytes())['pickPivot'][0]/1000
source=source.replace('exec(src);r=run',patch+'\nexec(src);r=run')
exec(compile(source,'joined-floor-to-carry','exec'))
values=np.array([[rr[0],rr[8],rr[9],rr[2]] for rr in r['trajectory'] if rr[0]>=2.82])
r['carryResult']=dict(startS=2.82,arriveS=6.22,heldUntilS=7.,minCassetteS=float(min(values[:,1])),maxCassetteS=float(max(values[:,1])),endLocal=r['trajectory'][-1][8:10],retainedThroughout=bool(np.all((values[:,1]>.12)&(values[:,1]<.35)&(values[:,2]>.018)&(values[:,2]<.09)&(abs(values[:,3])<.043))),scope='Pitch is commanded; the ring keeps the original six dynamic DOFs and velocities through the lift. Robot translation ends at the pickup stop. Release and field travel are separate.')
loc.write_text(json.dumps(r,separators=(',',':')));print(r['carryResult'])

