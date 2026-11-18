"""Fresh source-fix adoption; no re-hashing or conversion of archived checkpoints."""
import json,pathlib,sys
import checkpoint as c
R=c.R
NEW='sim/pressure/corrected-entry-680ms-evidence.json'
OLD=['sim/pressure/acquisition-entry-evidence.json','sim/pressure/continuation-100ms-evidence.json']
FILES=['sim/pressure/verify_corrected_lineage.py','tests/test_pressure_corrected_lineage.py']
def verify():
 new=json.loads((R/NEW).read_text());old=[json.loads((R/f).read_text()) for f in OLD];p,pins=c.factory.setup()
 if new['inputs']!=pins or new['schema']!='fresh-pressure-acquisition/1' or new['outcome']!='HORIZON':raise ValueError('fresh source inventory/scope')
 changed=sorted(f for f,h in old[0]['inputs'].items() if new['inputs'].get(f)!=h)
 if changed!=['sim/pressure/fast_geometry.py','tests/test_pressure_fast_geometry.py']:raise ValueError('unexpected source change')
 if old[1]['startCheckpoint']['identity']['inputs']['sim/pressure/fast_geometry.py']!=old[0]['inputs']['sim/pressure/fast_geometry.py']:raise ValueError('old ancestry mismatch')
 if old[1]['ancestrySha256']!=c.factory.r.sha(R/OLD[0]):raise ValueError('old report hash mismatch')
 ignored={'commands','contactTiming','contactSampleTimeS'}
 normalize=lambda row:{k:v for k,v in row.items() if k not in ignored}
 previous=old[0]['trace']+old[1]['trace']
 if len(new['trace'])!=680 or [normalize(r) for r in new['trace']]!=[normalize(r) for r in previous]:raise ValueError('fresh trajectory physical-byte mismatch')
 restored,origin=c.from_pressure_report(R/NEW)
 if restored.t!=new['final']['timeS'] or restored.note!=new['final']['note']:raise ValueError('new checkpoint mismatch')
 for f in OLD+[NEW]+FILES:pins[f]=c.factory.r.sha(R/f)
 return dict(schema='pressure-fresh-source-adoption/1',inputs=pins,changedSourceFiles=changed,physicalRowsEqual=680,newCheckpoint=c.export(restored),freshReportSha256=origin,acceptance=False,limits=['Fresh0-to.68 trajectory under corrected source, not checkpoint migration.','Archived receipts retain their historical source hashes.','Only metadata recording commands/contact-sample labels differs between old segmented and fresh single-run trace; every physical row field equals exactly.','No capture or calibration claim.'])
if __name__=='__main__':
 out=pathlib.Path(sys.argv[1])
 if out.exists():raise ValueError('fresh output required')
 result=verify()
 if any(c.factory.r.sha(R/f)!=h for f,h in result['inputs'].items()):raise ValueError('source changed')
 out.parent.mkdir(parents=True,exist_ok=True);tmp=out.with_suffix('.tmp');tmp.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8',newline='\n');tmp.replace(out)
