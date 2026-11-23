import json,math
from pathlib import Path
q=Path(__file__).parent;cases=json.loads((q/'loads.json').read_bytes())['cases'];out={}
for key,row in cases.items():
 for i,axis in enumerate(['side','rear']):
  force=row['peakDriveN'][i];radial=force/math.cos(math.radians(20));span=24;d=4.88
  # For side pinion, F is total tangential force even when opposed contacts cancel transverse force. Scalar upper bound conservatively adds magnitudes.
  tor=force*11.25;moment=radial*span/4
  out[key+'/'+axis]=dict(netTangentialUpperN=force,simpleSpanMm=span,radialResultantUpperN=radial,perSupportN=radial/2,torqueNm=tor/1000,nominalShearMPa=16*tor/(math.pi*d**3),nominalBendMPa=32*moment/(math.pi*d**3),elasticDeflectionMm=radial*span**3/(48*200000*math.pi*d**4/64))
(q/'shaft-demand.json').write_text(json.dumps(out,indent=1))
