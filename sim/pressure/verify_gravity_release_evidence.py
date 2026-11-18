"""Verify numerical controls and source bindings for preserved release experiments."""
import argparse,hashlib,json,pathlib

def verify(folder):
    names=['control','1mm-final','1mm-half','1p5','1p5-half','1p5-grid','1p5-stable-coarse','1p5-stable-fine','1p5-stable-grid']
    receipts={n:json.loads((folder/('release-'+n)/'receipt.json').read_text()) for n in names}
    expected_pins=receipts['control']['inputs'];scripts={r['scriptSha256'] for r in receipts.values()};assert len(scripts)==1
    for name,r in receipts.items():
        assert r['complete'] and r['status']=='DROPPED_50MM' and r['error'] is None
        assert r['inputs']==expected_pins
        trace=folder/('release-'+name)/'trace.jsonl';assert hashlib.sha256(trace.read_bytes()).hexdigest()==r['traceSha256']
        assert r['maxAbsWorkResidualW']<1e-10
    control=receipts['control'];dt=control['parameters']['dtS'];position_error=velocity_error=0.
    for line in (folder/'release-control/trace.jsonl').read_text().splitlines():
        row=json.loads(line);n=row['step'];position_error=max(position_error,abs(row['qpos'][2]-(.3-9.80665*dt*dt*n*(n+1)/2)))
        velocity_error=max(velocity_error,abs(row['qvel'][2]+9.80665*dt*n))
    assert position_error<1e-12 and velocity_error<1e-12
    coarse,fine,grid=[receipts['1p5-stable-'+n] for n in ['coarse','fine','grid']]
    step_delta=abs(fine['lastState']['timeS']-coarse['lastState']['timeS']);grid_delta=abs(grid['lastState']['timeS']-coarse['lastState']['timeS'])
    assert step_delta<.001 and grid_delta<.005
    assert fine['maxAbsEnergyBalanceErrorJ']<.6*coarse['maxAbsEnergyBalanceErrorJ']
    result={'schema':'gravity-release-numerical-controls/1','controlPositionErrorM':position_error,'controlVelocityErrorMps':velocity_error,'refinedStepDropTimeDifferenceS':step_delta,'refinedGridDropTimeDifferenceS':grid_delta,'sameInputPins':True,'sameExecutedScript':next(iter(scripts)),'verifiedTraceCount':len(receipts),'runs':[{ 'name':n,'parameters':r['parameters'],'status':r['status'],'dropTimeS':r['lastState']['timeS'],'maxAbsEnergyBalanceErrorJ':r['maxAbsEnergyBalanceErrorJ'],'maxAbsWorkResidualW':r['maxAbsWorkResidualW'],'traceSha256':r['traceSha256'],'seconds':r['seconds']} for n,r in receipts.items()],'acceptance':False,'limits':['Convergence of this isolated uncalibrated model only; not validation against hardware','The 1 ms contact runs are retained as unconverged diagnostics; use stable-coarse/fine/grid for the 1.5 mm conclusion']}
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--packet',type=pathlib.Path,required=True);p.add_argument('--output',type=pathlib.Path,required=True);a=p.parse_args();assert not a.output.exists();r=verify(a.packet);a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='runs'}))
