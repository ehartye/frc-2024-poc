"""Verify preserved state joins and bounded controls without promoting the cycle."""
import argparse,hashlib,json,math,pathlib
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads(p.read_bytes())
def verify(a):
    manifest=read(a.manifest)
    for path,digest in manifest['files'].items():assert sha(a.packet/path)==digest,path
    sources={sha(a.packet/p) for p in manifest['executedSources']}
    original=read(a.packet/manifest['historical']);prefix=read(a.packet/manifest['prefix']);resume=read(a.packet/manifest['resume']);index=read(a.packet/manifest['index']);parity=read(a.packet/manifest['parity']);local=read(a.packet/manifest['localHalf']);coarse_local=read(a.packet/manifest['localCoarse']);registration=read(a.packet/manifest['registration']);authority=read(a.packet/manifest['authority'])
    assert index['scriptSha256'] in sources and prefix['scriptSha256'] in sources
    assert not index['partial'] and index['outcome']=='ATTAINED_GROUND_INDEX' and index['index'] is not None
    assert prefix['startCheckpoint']==original['endCheckpoint'] and resume['startCheckpoint']==original['endCheckpoint']
    assert index['startCheckpoint']==resume['endCheckpoint'] and index['resumeSha256']==sha(a.packet/manifest['resume'])
    assert prefix['trace'][:len(resume['trace'])]==resume['trace']
    overlap=len(prefix['trace'])-len(resume['trace']);assert overlap>=100 and prefix['trace'][-overlap:]==index['trace'][:overlap]
    assert parity['exactlyEqual'] and parity['sharedSteps']>=100
    assert index['cullingSourceSha256'] in sources
    for report in [original,prefix,resume,index,local,coarse_local]:
        for path,digest in report['inputs'].items():assert sha(a.project/path)==digest,path
    combined=original['trace']+resume['trace']+index['trace']
    times=[r['timeS'] for r in combined];assert all(abs(t-(i+1)*.001)<1e-9 for i,t in enumerate(times))
    assert combined[-1]['chassis']==index['endCheckpoint']['state']['chassis'] and combined[-1]['note']==index['endCheckpoint']['state']['note']
    assert index['index']['physicalState']==index['endCheckpoint']['state']
    def observation(row):
        d=row['chassis'];n=row['note'];c,s=math.cos(d['heading']),math.sin(d['heading']);dx=n['p'][0]-d['x'];dy=n['p'][1]-d['y'];x=c*dx+s*dy;y=-s*dx+c*dy;ux=n['v'][0]-d['vx'];uy=n['v'][1]-d['vy'];vx=c*ux+s*uy+d['yaw_rate']*y;vy=-s*ux+c*uy-d['yaw_rate']*x
        return x,y,vx,vy,math.hypot(d['vx'],d['vy'])
    def eligible(row):
        x,y,vx,vy,speed=observation(row);return .180<=x<=.190 and abs(y)<=.001 and math.hypot(vx,vy)<=.003 and speed<=.003
    last_bad=max(i for i,r in enumerate(combined) if not eligible(r));dwell=combined[last_bad+1]['timeS'];assert abs(dwell-index['dwellStartS'])<1e-9
    assert index['index']['timeS']-dwell>=.1-1e-10 and abs(index['index']['timeS']/.01-round(index['index']['timeS']/.01))<1e-8
    assert all(eligible(r) for r in combined[last_bad+1:])
    max_res=max(abs(r['energyJ']['residual']) for r in combined)
    assert all(abs(r['energyJ']['residual'])<=.01+.01*abs(r['energyJ']['sourceWork']) for r in combined)
    assert max(abs(r['pressureWorkResidualW']) for r in combined)<1e-8
    assert not registration['automaticNativeQualificationPermitted'] and registration['baseConfigHashMatches'] and registration['nominalTotalGapDifferenceMm']==-1
    assert all(r['positiveDriveOrCoastCanSupplyRequiredCurrent']==(r['translationSpeedMps']==0) for r in authority['rows'])
    assert max(abs(r['forceBalanceResidualN']) for r in authority['rows'])<1e-9
    local_lookup={round(r['timeS'],8):r for r in local['trace']};local_dp=local_dv=0.
    for row in coarse_local['trace']:
        other=local_lookup[round(row['timeS'],8)];local_dp=max(local_dp,math.dist(row['relativeRobotXY'],other['relativeRobotXY']));local_dv=max(local_dv,math.dist(observation(row)[2:4],observation(other)[2:4]))
    transports={name:read(a.packet/path) for name,path in manifest['transport'].items()}
    for name,r in transports.items():
        assert r['scriptSha256'] in sources and r['sourceIndexSha256']==sha(a.packet/manifest['index']) and r['startCheckpoint']==index['endCheckpoint']
        assert r['outcome']=='TRANSPORT_CORRIDOR_EXIT' and r['trace'][-1]['relativePlanarSpeedMps']>.01
        assert all(.180<=x['relativeRobotXY'][0]<=.190 and abs(x['relativeRobotXY'][1])<=.001 and x['relativePlanarSpeedMps']<=.01 for x in r['trace'][:-1])
        assert r['events'][0]['timeS']==index['index']['timeS']
        assert max(abs(x['pressureWorkResidualW']) for x in r['trace'])<1e-8
    medium=transports['half'];fine=transports['quarter'];lookup={round(r['timeS'],8):r for r in fine['trace']};dp=dv=0.;count=0
    for row in medium['trace']:
        other=lookup.get(round(row['timeS'],8))
        if other is None:continue
        count+=1;dp=max(dp,math.dist(row['relativeRobotXY'],other['relativeRobotXY']));dv=max(dv,math.dist(observation(row)[2:4],observation(other)[2:4]))
    assert count>5 and dp<=.001 and dv<=.003 and abs(medium['endTimeS']-fine['endTimeS'])<=.001
    result={'schema':'acquisition-continuity-verification/1','evidenceChecksPassed':True,'acceptance':False,'hardwareValidated':False,'nativeRegistrationAccepted':False,'continuousAcquisitionToLaunch':False,'scriptSha256':sha(pathlib.Path(__file__)),'manifestSha256':sha(a.manifest),'indexTimeS':index['index']['timeS'],'indexDwellStartS':dwell,'indexObservation':index['index']['observation'],'continuousAcceptedAcquisitionSteps':len(combined),'maximumCumulativeEnergyResidualJ':max_res,'exactResumeParitySteps':overlap,'localAcquisitionHalfStep':{'windowS':.02,'maxRelativePositionDifferenceM':local_dp,'maxRelativeVelocityDifferenceMps':local_dv},'transport':{name:{k:r[k] for k in ['stepS','outcome','startTimeS','endTimeS','chassisTravelM','noteWorldTravelM','maximumRelativeSpeedMps']} for name,r in transports.items()},'localTransportFineComparison':{'sharedStates':count,'maxRelativePositionDifferenceM':dp,'maxRelativeVelocityDifferenceMps':dv,'exitTimeDifferenceS':abs(medium['endTimeS']-fine['endTimeS'])},'nativeFaceDifferenceMm':registration['perSideFaceDifferenceMm'],'limits':['Full acquisition startup/index not rerun at all required finer timesteps or spatial grids','Transport refinement starts from one identical attained index; it is local evidence only','Historical pressure geometry is not the relieved native revision despite matching base configuration hash','No lift/pitch/feed or shooting was joined; no hardware acceptance or full loading-contract verdict']}
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['packet','project','manifest','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
    a=p.parse_args();assert not a.output.exists();r=verify(a);a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(r))
