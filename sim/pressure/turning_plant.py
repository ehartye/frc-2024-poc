"""Coupled rigid-body plant for differential drive, lift, pitch, and a free NOTE."""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

from run_axial_shear_lift import ROOT, EVIDENCE, load
from mechanics import ActuatorMechanics
from plant import DrivePlant
from native_pressure_arrays import prepare
from retained_route import controls, short_brake
from signed_pressure_acquisition import motor_electrical
from turning_contact import belt_wrenches, floor_wrench


class TurningPlant:
    def __init__(self, acquired, grid):
        self.cad=load(ROOT/"cad/assembly/intake_components.json")
        self.nc=load(ROOT/"sim/acquisition/free-planar-config.json")["model"]
        self.ac=load(ROOT/"sim/actuation/config.json")
        self.drive=DrivePlant(load(ROOT/"sim/drivetrain/config.json"))
        self.act=ActuatorMechanics(self.ac,dict(timeS=0.,q=[0.,0.,0.],qd=[0.,0.,0.]))
        self.binding=load(EVIDENCE/"native-signed-20261118/native-pressure-binding.json")["effectiveGeometryMm"]
        self.contract=load(EVIDENCE/"native-lift-20261118/native-pressure-lift-contract.json")
        self.grid=grid
        self.z0=self.cad["floorCenterZ"]/1000
        self.x0=self.cad["stageCenterX"]/1000
        self.major=self.cad["note"]["majorRadius"]/1000
        self.tube=self.cad["note"]["tubeRadius"]/1000
        angle=np.arange(self.contract["floorPoints"])*2*np.pi/self.contract["floorPoints"]
        self.ring=self.major*np.column_stack((np.cos(angle),np.sin(angle),np.zeros_like(angle)))
        self.initial=acquired
        self.robot=mujoco.MjModel.from_xml_string(self.xml())
        self.data=mujoco.MjData(self.robot)
        self.body={name:self.robot.body(name).id for name in ("chassis","cradle","note")}
        c,n=acquired["chassis"],acquired["note"]
        self.q=np.array([c["x"],c["y"],c["heading"],0.,0.,*n["p"],self.z0,
                         math.cos(n["yaw"]/2),0.,0.,math.sin(n["yaw"]/2)])
        self.v=np.array([c["vx"],c["vy"],c["yaw_rate"],0.,0.,*n["v"],0.,0.,0.,n["spin"]])
        self.rotors=np.array(c["wheel"]+n["w"])
        self.shear=np.zeros(4)
        self.loads=[self.drive.c["massKg"]*self.ac["gravityMps2"]/2]*2
        self.gains=np.array([self.ac["liftGearRatio"]/self.ac["liftRadiusM"],self.ac["pitchGearRatio"]])

    def xml(self):
        ac=self.ac
        mc,mr=ac["carriageMassKg"],ac["rotatingMassKg"]
        mb=self.drive.c["massKg"]-mc-mr
        cx,cz=ac["comLocalXZ_M"]
        counter=-(mc*self.x0+mr*(self.x0+cx))/mb
        ic,ir=.06,ac["comPitchInertiaKgM2"]
        iz=self.drive.c["yawInertiaKgM2"]-mb*counter**2-ic-mc*self.x0**2-ir-mr*(self.x0+cx)**2
        m=self.nc["pieceMassKg"]
        transverse=m*(self.major**2/2+5*self.tube**2/8)
        lift_arm=ac["liftRotorInertiaKgM2"]*(ac["liftGearRatio"]/ac["liftRadiusM"])**2
        pitch_arm=ac["pitchRotorInertiaKgM2"]*ac["pitchGearRatio"]**2
        return f"""<mujoco><compiler angle="radian"/><option gravity="0 0 -{ac['gravityMps2']}">
            <flag energy="enable"/></option><worldbody>
            <body name="chassis"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 1 0"/>
            <joint type="hinge" axis="0 0 1"/>
            <inertial pos="{counter} 0 0" mass="{mb}" diaginertia="{iz} {iz} {iz}"/>
            <body name="carriage" pos="{self.x0} 0 {self.z0}">
            <joint type="slide" axis="0 0 1" armature="{lift_arm}"/>
            <inertial pos="0 0 0" mass="{mc}" diaginertia="{ic} {ic} {ic}"/>
            <body name="cradle"><joint type="hinge" axis="0 -1 0" armature="{pitch_arm}"/>
            <inertial pos="{cx} 0 {cz}" mass="{mr}" diaginertia="{ir} {ir} {ir}"/>
            </body></body></body><body name="note"><freejoint/>
            <inertial pos="0 0 0" mass="{m}" diaginertia="{transverse} {transverse} {self.nc['yawInertiaKgM2']}"/>
            </body></worldbody></mujoco>"""

    def contact(self,q,v,rotors,shear):
        m,d=self.robot,self.data
        d.qpos[:]=q; d.qvel[:]=v
        mujoco.mj_fwdPosition(m,d); mujoco.mj_fwdVelocity(m,d)
        c,n,b=(self.body[k] for k in ("cradle","note","chassis"))
        cr=d.xmat[c].reshape(3,3); nr=d.xmat[n].reshape(3,3); yr=d.xmat[b].reshape(3,3)
        cp=d.xpos[c]; npiece=d.xpos[n]
        cw=np.array([0.,0.,v[2]])-v[4]*yr[:,1]
        cv=np.array([v[0],v[1],v[3]])+v[2]*self.x0*yr[:,1]
        nw=nr@v[8:11]
        p=prepare(self.cad,self.binding,note_center=npiece.tolist(),note_basis=nr.tolist(),
                cradle_origin=cp.tolist(),cradle_basis=cr.tolist(),resolution=(self.grid,self.grid),k_area=1e7)
        con=belt_wrenches(p,note_v=v[5:8],note_w=nw,cradle_v=cv,cradle_w=cw,belt_w=rotors[2:],
                cradle_basis=cr,forward=yr[:,0],shear_m=shear,motor_radius=self.nc["radiusM"],
                mu=self.nc["beltMu"],slip_scale=self.nc["slipRegularizationMps"])
        floor,moment,potential,heat=floor_wrench(npiece,nr,v[5:8],nw,self.ring,self.tube,
                self.contract,self.nc["groundMu"],self.nc["slipRegularizationMps"])
        con.update(floor=floor,floorMoment=moment,floorPotential=potential,floorHeat=heat,
                depth=p["maximumPenetrationM"],normalRobot=yr.T@nr[:,2],
                centerRobot=yr.T@(npiece-d.xpos[b]),gap=npiece[2]-self.tube-self.major*np.linalg.norm(nr[2,:2]))
        return con

    def command(self,tick,con,sign,lead,speed,yaw):
        q,v=self.q,self.v
        yr=self.data.xmat[self.body["chassis"]].reshape(3,3)
        nr=self.data.xmat[self.body["note"]].reshape(3,3)
        local=con["centerRobot"]
        relative=yr.T@(v[5:8]-np.r_[v[:2],0.])-np.cross([0.,0.,v[2]],local)
        nz=nr[:,2]; nw=nr@v[8:11]; nzrate=np.cross(nw,nz)[2]
        sine=np.linalg.norm(nz[:2])
        sense=-1. if con["normalRobot"][0]>0 else 1.
        pitch=sense*math.atan2(sine,nz[2])
        pitch_rate=-sense*nzrate/sine if sine>1e-10 else -float(nw@yr[:,1])
        forward=float(v[:2]@yr[:2,0])
        state=dict(chassis=dict(x=0.,vx=forward),note=dict(p=[float(local[0])],
                v=[forward+float(relative[0])],w=self.rotors[2:]),zPivot=float(self.z0+q[3]),
                vPivot=float(v[3]),theta=float(q[4]),thetaRate=float(v[4]),noteZ=q[7],noteVz=v[7],
                notePitch=pitch,notePitchRate=pitch_rate)
        obs=dict(q=dict(cradleReactionWorldN=yr.T@(-con["force"]),cradleMomentWorldNm=yr.T@con["cradleMoment"]),
                 floorForce=yr.T@con["floor"],pressure=dict(normalN=con["normal"]))
        view=SimpleNamespace(s=state,z0=self.z0,cad=self.cad,ac=self.ac,act=self.act,nc=self.nc,
                contact=lambda s,cache:obs,initial=self.initial)
        cmd=controls(view,max(0,tick-lead),self.bus)
        tau=min(1.,tick/100/1.0)
        ramp=tau*tau*(3.-2*tau)
        want_v=speed*ramp; want_yaw=sign*yaw*ramp
        # Use actual chassis speed and yaw. Differential effort must overcome six tire scrub forces.
        common=.8*(want_v-forward)+want_v*self.drive.c["motorK"]*self.drive.c["gearRatio"]/self.drive.c["wheelRadiusM"]/12.
        diff=.65*(want_yaw-v[2])+.20*want_yaw
        cmd["drive"]=[float(np.clip(common-diff,-.27,.27)),float(np.clip(common+diff,-.27,.27))]
        cmd["driveModes"]=["drive","drive"] if speed else ["brake","brake"]
        return cmd
    def electrical_bus(self,v,rotors,cmd):
        """Solve the shared supply exactly on the unsaturated DC motor branches.

        Reject a clipped current or a cycling diode branch. Such steps require a
        different electrical solve. A passive brake draws no battery power.
        """
        dc,ac,nc=self.drive.c,self.ac,self.nc
        duty=np.array([*cmd["drive"],*cmd["actuator"],*cmd["pinch"]])
        if cmd["driveModes"][0]=="brake":
            duty[:2]=0.
        emf=np.r_[rotors[:2]*dc["gearRatio"]*dc["motorK"],v[3:5]*self.gains*ac["motorK"],rotors[2:]/nc["kvRadpsPerV"]]
        resist=np.array([dc["motorResistanceOhm"]]*2+[ac["resistanceOhm"]]*2+[nc["motorResistanceOhm"]]*2)
        copies=np.array([2,2,1,1,1,1])
        bus=12.
        for _ in range(5):
            active=duty.copy()
            active[4:]=np.where(duty[4:]*(duty[4:]*bus-emf[4:])>0,duty[4:],0.)
            a=np.sum(copies*active**2/resist)
            b=np.sum(copies*active*emf/resist)
            new=(12.+.02*b)/(1.+.02*a)
            if abs(new-bus)<1e-12:break
            bus=new
        else:raise ValueError("SUPPLY_REGIME_DID_NOT_SETTLE")
        if not 0.<bus<=12.+1e-8:raise ValueError("UNSUPPORTED_BATTERY_REGEN_OR_COLLAPSE")
        return float(bus)

    def rates(self,q,v,rotors,shear,cmd,con):
        d,m=self.data,self.robot
        bus=self.electrical_bus(v,rotors,cmd)
        drive_state=dict(t=0.,x=float(q[0]),y=float(q[1]),heading=float(q[2]),
                vx=float(v[0]),vy=float(v[1]),yaw_rate=float(v[2]),wheel=rotors[:2].tolist())
        dr=self.drive._rates(drive_state,bus,cmd["drive"],cmd["driveModes"],self.loads,[0.,0.],0.)
        ae=self.act.electrical(bus,cmd["actuator"],cmd["actuatorModes"],[0.,float(q[3])+self.z0,float(q[4])],
                               [0.,float(v[3]),float(v[4])])
        pinch=[]
        for duty,fraction,omega in zip(cmd["pinch"],cmd["pinchBrakeFraction"],rotors[2:]):
            pinch.append(short_brake(float(omega),fraction,self.nc) if fraction
                         else motor_electrical(duty,bus,float(omega),self.nc))
        de=dr["electrical"]
        power=ae["busPowerW"]+de["busPowerW"]+sum(x["terminalPowerW"] for x in pinch)
        current=(12.-bus)/.02
        if max(abs(bus*current-power),abs(12.*current-power-.02*current**2))>1e-8:
            raise ValueError("SUPPLY_POWER_MISMATCH_OR_CURRENT_LIMIT")
        if ae["infeasibleCurrentLimitVoltage"] or de["infeasibleCurrentLimitVoltage"]:
            raise ValueError("MOTOR_VOLTAGE_DOMAIN")
        yaw_basis=d.xmat[self.body["chassis"]].reshape(3,3)
        d.qfrc_applied[:]=0.
        d.qfrc_applied[:2]=yaw_basis[:2,:2]@dr["contactForceBodyN"]
        d.qfrc_applied[2]=dr["contactYawTorqueNm"]
        d.qfrc_applied[3:5]=ae["generalizedMotorForce"][1:]
        for name,f,t in (("note",con["force"]+con["floor"],con["noteMoment"]+con["floorMoment"]),
                         ("cradle",-con["force"],con["cradleMoment"])):
            ident=self.body[name]
            mujoco.mj_applyFT(m,d,f,t,d.xpos[ident],ident,d.qfrc_applied)
        mujoco.mj_fwdActuation(m,d)
        mujoco.mj_fwdAcceleration(m,d)
        energy=dr["energyRates"]
        heat=(.02*current**2+de["windingHeatW"]+de["controllerBrakeHeatW"]
              +energy["motorDragHeat"]+energy["gearDragHeat"]+energy["tireSlipHeat"]
              +sum(x["windingHeatW"] for x in ae["motors"])+sum(x["windingHeatW"] for x in pinch)
              +con["floorHeat"]+con["heat"])
        rotor_acc=np.r_[dr["dw"],(np.array([p["torqueNm"] for p in pinch])+con["rotor"])/self.nc["rotorInertiaKgM2"]]
        return d.qacc_smooth.copy(),rotor_acc,con["shearRate"],bus,np.array([12.*current,heat])

    def mechanical(self,contact):
        mujoco.mj_energyPos(self.robot,self.data)
        mujoco.mj_energyVel(self.robot,self.data)
        return (sum(self.data.energy)+.5*self.drive.axle_inertia*sum(self.rotors[:2]**2)
                +.5*self.nc["rotorInertiaKgM2"]*sum(self.rotors[2:]**2)
                +contact["potential"]+contact["floorPotential"])

    def advance(self,dt,cmd,start):
        def integrate(position,velocity,step):
            out=position.copy()
            mujoco.mj_integratePos(self.robot,out,velocity,step)
            return out
        a,b,h,_,_=self.rates(self.q,self.v,self.rotors,self.shear,cmd,start)
        half=dt/2
        qm=integrate(self.q,self.v,half); vm=self.v+half*a
        wm=self.rotors+half*b; sm=self.shear+half*h
        cm=self.contact(qm,vm,wm,sm)
        a,b,h,self.bus,work=self.rates(qm,vm,wm,sm,cmd,cm)
        self.q=integrate(self.q,vm,dt)
        self.v+=dt*a;self.rotors+=dt*b;self.shear+=dt*h
        final=self.contact(self.q,self.v,self.rotors,self.shear)
        return final,dt*work
