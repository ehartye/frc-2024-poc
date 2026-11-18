"""Check saved standard-contact experiments without fitting retention outcomes."""
import argparse,hashlib,json,pathlib

def verify(packet):
    names=['pilot','half-step','finer-torus','freefall','low-friction','zero-friction','downward-kick','without-noslip','long-dwell','long-dwell-fine','long-without-noslip']
    runs={name:json.loads((packet/name/'receipt.json').read_text()) for name in names}
    reference=runs['pilot']
    for name,r in runs.items():
        assert r['complete'] and not any(r['warningCounts'])
        for field in ['nativeArchiveSha256','sourceReceiptSha256','sourceConfigSha256','scriptSha256','mujocoVersion']:assert r[field]==reference[field],(name,field)
        assert hashlib.sha256((packet/name/'trace.jsonl').read_bytes()).hexdigest()==r['traceSha256']
        assert r['maxFrictionConeExcessN']<1e-8
        target=r['parameters']['initialNormalForceTargetPerSideN']
        assert max(abs(n-target) for n in r['parameters']['matchedGravityFreeNormalN'])<target*.001
    for name in ['pilot','half-step','finer-torus','long-dwell','long-dwell-fine']:
        r=runs[name];assert r['status']=='HORIZON' and r['maxAbsVerticalDisplacementM']<1e-7
        assert abs(r['final']['forceWorldN'][2]-.235*9.80665)<1e-7
        assert r['maxXYDisplacementM']<1e-6 and r['maxRotationRad']<1e-4
    for name in ['freefall','low-friction','zero-friction']:
        assert runs[name]['status']=='DROPPED_50MM' and runs[name]['final']['timeS']<.3
    kick=runs['downward-kick'];assert kick['status']=='HORIZON' and .001<kick['maxAbsVerticalDisplacementM']<.006 and abs(kick['final']['qvel'][2])<1e-7
    assert runs['without-noslip']['maxAbsVerticalDisplacementM']>.001
    assert runs['long-without-noslip']['maxAbsVerticalDisplacementM']>.01
    dt=runs['freefall']['parameters']['dtS'];error=0.
    for line in (packet/'freefall/trace.jsonl').read_text().splitlines():
        row=json.loads(line);n=row['step'];error=max(error,abs(row['qpos'][2]-(.3-9.80665*dt*dt*n*(n+1)/2)))
        assert row['contacts']==0
    assert error<1e-12
    return {'schema':'native-static-contact-controls/1','allChecksPassed':True,'acceptance':False,'analyticFreefallPositionErrorM':error,'nativeArchiveSha256':reference['nativeArchiveSha256'],'sourceReceiptSha256':reference['sourceReceiptSha256'],'sourceConfigSha256':reference['sourceConfigSha256'],'executedScriptSha256':reference['scriptSha256'],'runs':runs,'limits':['Checks establish bounded model behavior and numerical controls, not measured hardware validity','Point-contact normal response and capsule geometry differ from the earlier finite-area model; only initial normal force is matched','No acquisition, dynamic cradle, motor coupling, drive, launcher or scoring test']}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--packet',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();assert not a.output.exists();r=verify(a.packet);a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='runs'}))
