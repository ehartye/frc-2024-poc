"""Prescribed-state transport feasibility, not an attained transport trajectory."""
import argparse,hashlib,json,math,pathlib,sys
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def evaluate(project):
    sys.path.insert(0,str(project/'sim/pressure'))
    import fast_geometry,finite_transport
    cp=project/'cad/assembly/intake_components.json';mp=project/'sim/acquisition/free-planar-config.json'
    cad=json.loads(cp.read_bytes());motor=json.loads(mp.read_bytes())['model'];rows=[]
    for x in [.20119016209167434,.185]:
        prepared=fast_geometry.prepare(cad,note_center=[x,0,cad['floorCenterZ']/1000],cradle_origin=[cad['stageCenterX']/1000,0,cad['floorCenterZ']/1000],resolution=(1024,1024),k_area=1e7)
        for speed in [0.,.01,.05,.1]:
            ground=motor['groundMu']*motor['pieceMassKg']*9.80665*math.tanh(speed/motor['slipRegularizationMps'])
            def query(omega):return finite_transport.rates(prepared,note_velocity=[speed,0,0],cradle_velocity=[speed,0,0],rotor_omega=[omega,omega],pitch_radius_m=motor['radiusM'],mu=motor['beltMu'],slip_scale=motor['slipRegularizationMps'])
            lo,hi=-20.,20.;qlo=query(lo);qhi=query(hi)
            assert qlo['noteForceWorldN'][0]>=ground>=qhi['noteForceWorldN'][0]
            for _ in range(42):
                mid=(lo+hi)/2
                if query(mid)['noteForceWorldN'][0]>ground:lo=mid
                else:hi=mid
            omega=(lo+hi)/2;q=query(omega);currents=[-torque*motor['kvRadpsPerV'] for torque in q['rotorContactTorqueNm']];volts=[omega/motor['kvRadpsPerV']+i*motor['motorResistanceOhm'] for i in currents]
            no_power=query(0.)
            rows.append({'relativeXM':x,'translationSpeedMps':speed,'groundDragN':ground,'normalPerSideN':[sum(c['normalN'] for c in prepared['contacts'] if c['motor']==i) for i in range(2)],'requiredRotorOmegaRadps':omega,'requiredMotorCurrentA':currents,'requiredTerminalV':volts,'requiredTerminalPowerW':[i*v for i,v in zip(currents,volts)],'forceBalanceResidualN':q['noteForceWorldN'][0]-ground,'contactWorkResidualW':q['workResidualW'],'zeroRelativeBeltSpeedTractionN':no_power['noteForceWorldN'][0],'positiveDriveOrCoastCanSupplyRequiredCurrent':all(i>=-1e-10 for i in currents),'withinAssumedCurrentLimit':all(abs(i)<=motor['currentLimitA'] for i in currents),'requiresBatteryRegeneration':sum(i*v for i,v in zip(currents,volts))< -1e-10})
            print(json.dumps(rows[-1]),flush=True)
    assert all(r['positiveDriveOrCoastCanSupplyRequiredCurrent']==(r['translationSpeedMps']==0) for r in rows)
    assert max(abs(r['forceBalanceResidualN']) for r in rows)<1e-9
    return {'schema':'pressure-transport-control-authority/1','acceptance':False,'scriptSha256':sha(pathlib.Path(__file__)),'inputs':{p:sha(project/p) for p in ['cad/assembly/intake_components.json','sim/acquisition/free-planar-config.json','sim/pressure/fast_geometry.py','sim/pressure/finite_transport.py','sim/pressure/finite_area.py','cad/assembly/belt_contact_geometry.py','sim/pressure/pressure_acquisition.py']},'rows':rows,'limits':['Prescribed symmetric steady-translation states, not actual acquisition or transport trajectories','Original pressure geometry and material hypotheses, not the later relieved native candidate','Signed motoring is a proposed model/controller capability; no electrical or hardware change was made','A passive short brake has zero torque at zero rotor speed and is not an ideal position lock','Does not rule out finite transient transport or support by end-wrap normal forces outside these straight-contact states']}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--project',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();assert not a.output.exists();r=evaluate(a.project);a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n')
