"""Verify numerical convergence without granting native/hardware/cycle acceptance."""
import argparse,hashlib,json,math,pathlib,sys
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def read(path):
    return json.loads(path.read_text(encoding='utf-8'))

def response_check(report,first,second,limits):
    one=[r for r in report['rows'] if (r['segments'],r['phaseDeg'])==first]
    two=[r for r in report['rows'] if (r['segments'],r['phaseDeg'])==second]
    assert len(one)==len(two)==75
    result={k:0. for k in ['normalForcesN','noteForceWorldN','rotorReactionNm']}
    for a,b in zip(one,two):
        assert all(a[k]==b[k] for k in ['advanceM','normalOffsetM','noteSpeedMps','rimSpeedMps','mu'])
        for field in result:
            floor=limits['torqueAbsoluteFloorNm'] if field=='rotorReactionNm' else limits['forceAbsoluteFloorN']
            for x,y in zip(a[field],b[field]):result[field]=max(result[field],abs(x-y)/max(floor,limits['forceRelativeTolerance']*max(abs(x),abs(y))))
    return result

def verify(a):
    sys.path.insert(0,str(a.project/'sim/mujoco'))
    from verify_native_launcher_contact import verify as integrity
    manifest=read(a.manifest);contract=read(a.packet/manifest['trajectoryContract'])
    # Reuse the already reviewed checks for provenance, free-flight, reaction,
    # current/force bounds and electrical identities; retain all old rejected runs.
    checked=integrity(a.packet,a.project,a.manifest,a.binding)
    names=['grid-8192','grid-16384','half-step','phase-16384','drive-launch','drive-grid-8192','drive-half-step','zero-drive','zero-friction','no-contact','floating']
    runs={name:read(a.packet/manifest['runs']['quadrature/'+name]/'receipt.json') for name in names}
    source_hashes={sha(a.packet/p) for p in manifest['executedSources']}
    contract_hashes={sha(a.packet/p) for p in manifest['contracts']}
    reference=runs['grid-16384'];parameters=reference['parameters']
    for name,run in runs.items():
        assert run['contractSha256'] in contract_hashes and run['scriptSha256']==reference['scriptSha256']
        p=run['parameters'];assert p['contactLaw']=='uniform-ring-constraint-quadrature' and p['noslipIterations']==0
        impedance=.9/(.9+(p['segments']/128)*.1)
        assert abs(p['impedance']-impedance)<1e-14 and abs(p['quadratureWeight']-128/p['segments'])<1e-14
        assert p['referenceStiffness']==parameters['referenceStiffness'] and p['referenceDamping']==parameters['referenceDamping']
        assert abs(p['fittedStiffness']/impedance-parameters['referenceStiffness']/.9)<1e-10
        assert abs(p['fittedDamping']/impedance-parameters['referenceDamping']/.9)<1e-10
        assert run['maxNoteWorkResidualJ']<contract['trajectoryFineComparison']['workResidualLimitJ']
    response=read(a.packet/manifest['refinedResponse']);duplicate=read(a.packet/manifest['duplicateControl']);handoff=read(a.packet/manifest['handoff']);coarse=read(a.packet/manifest['coarseResponse'])
    assert response['scriptSha256'] in source_hashes and response['runnerSha256'] in source_hashes
    assert duplicate['scriptSha256'] in source_hashes and handoff['scriptSha256'] in source_hashes
    assert response['contractSha256'] in contract_hashes
    static={
        'spatial':response_check(response,(8192,0.),(16384,0.),contract['staticFineComparison']),
        'phase':response_check(response,(16384,0.),(16384,180/16384),contract['staticFineComparison']),
    }
    assert max(v for check in static.values() for v in check.values())<=1
    assert max(response_check(coarse,(2048,0.),(4096,0.),contract['staticFineComparison']).values())>1
    weighted=[x for x in duplicate['comparisons'] if x['weighted']]
    unweighted=[x for x in duplicate['comparisons'] if not x['weighted']]
    assert max(x['maximumAbsoluteDifferences']['qfrcConstraint'] for x in weighted)<1e-9
    assert min(x['maximumAbsoluteDifferences']['noteForceWorldN'] for x in unweighted)>5
    def trace(name):
        p=a.packet/manifest['runs']['quadrature/'+name]/'trace.jsonl'
        return {round(r['timeS'],8):r for r in map(json.loads,p.read_text().splitlines())}
    comparisons={};limits=contract['trajectoryFineComparison']
    for basename,name in [('grid-16384','grid-8192'),('grid-16384','half-step'),('grid-16384','phase-16384'),('drive-launch','drive-grid-8192'),('drive-launch','drive-half-step')]:
        base=trace(basename);comparison_reference=runs[basename]
        other=trace(name);times=sorted(base.keys()&other.keys());assert len(times)>250
        delta_p=max(math.dist(base[t]['notePositionM'],other[t]['notePositionM']) for t in times)
        delta_v=max(math.dist(base[t]['noteVelocityMps'],other[t]['noteVelocityMps']) for t in times)
        speed=math.dist(comparison_reference['lastContact']['noteVelocityMps'],[0,0,0]);other_speed=math.dist(runs[name]['lastContact']['noteVelocityMps'],[0,0,0])
        delta_speed=abs(speed-other_speed);delta_time=abs(comparison_reference['lastContact']['timeS']-runs[name]['lastContact']['timeS'])
        assert runs[name]['status']==comparison_reference['status']=='PASSED_LOCAL_LAUNCHER_DOMAIN'
        assert delta_p<=limits['maximumSharedTimeNotePositionDifferenceM'] and delta_v<=limits['maximumSharedTimeNoteVelocityDifferenceMps']
        assert delta_speed<=limits['lastContactSpeedRelativeTolerance']*max(speed,other_speed) and delta_time<=limits['lastContactTimeToleranceS']
        comparisons[name]={'sharedTimes':len(times),'maxNotePositionDifferenceM':delta_p,'maxNoteVelocityDifferenceMps':delta_v,'lastContactSpeedDifferenceMps':delta_speed,'lastContactTimeDifferenceS':delta_time}
    assert runs['half-step']['maxNoteWorkResidualJ']<.6*reference['maxNoteWorkResidualJ']
    assert runs['drive-half-step']['maxNoteWorkResidualJ']<.6*runs['drive-launch']['maxNoteWorkResidualJ']
    assert runs['drive-launch']['status']=='PASSED_LOCAL_LAUNCHER_DOMAIN' and runs['drive-launch']['final']['chassisPositionM'][0]>.015
    assert runs['drive-launch']['minimumBusV']<reference['minimumBusV']
    assert runs['no-contact']['status']=='REVERSED_80MM'
    for name in ['zero-drive','zero-friction']:
        assert runs[name]['final']['advanceM']<-.03 and runs[name]['status']!='PASSED_LOCAL_LAUNCHER_DOMAIN'
    assert handoff['identityControl']['directSplicePermitted'] and not handoff['comparison']['directSplicePermitted']
    assert handoff['launcher']['receiptSha256']==sha(a.packet/manifest['runs']['quadrature/grid-16384']/'receipt.json')
    return {'schema':'launcher-quadrature-verification/1','numericalGatePassed':True,'acceptance':False,'hardwareValidated':False,'continuousCycle':False,'scriptSha256':sha(pathlib.Path(__file__)),'baseVerifierSha256':sha(a.project/'sim/mujoco/verify_native_launcher_contact.py'),'manifestSha256':sha(a.manifest),'contractSha256':sha(a.packet/manifest['trajectoryContract']),'nativeArchiveSha256':reference['nativeArchiveSha256'],'runnerSha256':reference['scriptSha256'],'preservedPreviousIntegrityRunCount':checked['runCount']-len(names),'newIntegrityRunCount':len(names),'staticResponseMaxToleranceRatios':static,'duplicateControl':duplicate['comparisons'],'trajectoryComparisons':comparisons,'runs':{name:checked['runs']['quadrature/'+name] for name in names},'actualInitialNormalPerWheelN':reference['initial']['normalForcesN'],'handoff':handoff['comparison'],'limits':['Numerical checks apply to this fixed uncalibrated solver-law hypothesis and the declared staged launch scenarios','Reference acceleration gains are not physical spring/damper constants; initial fine-grid load differs from the original coarse fit','No full native hard-part/continuous clearance or uninterrupted acquisition-lift-launch state has been demonstrated','Passing these checks does not rehabilitate the preserved original unconverged contact law']}

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['packet','project','manifest','binding','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
    a=p.parse_args();assert not a.output.exists();r=verify(a);a.output.write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({k:v for k,v in r.items() if k not in ('runs','duplicateControl')}))
