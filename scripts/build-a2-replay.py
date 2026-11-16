"""Render a portable offline replay from a fresh A2 mission trace. Does not execute physics."""
import argparse,csv,hashlib,json,math,pathlib
ROOT=pathlib.Path(__file__).resolve().parents[1]
REQUIRED={'timeS','state','robotX','robotY','robotHeadingRad','intakeAngleDeg','intakeLengthM','piecePhase','pieceXWorld','pieceYWorld','pieceZWorld','shooterRpm','shotCount','fault','event'}
NUMERIC={'timeS','robotX','robotY','robotHeadingRad','intakeAngleDeg','intakeLengthM','pieceXWorld','pieceYWorld','pieceZWorld','shooterRpm','shotCount','exitVx','exitVy','exitVz','bodyPitchRad','bodySpinRadPerS','energyResidualJ','outgoingBodyEnergyJ'}
def render(trace,output):
 trace=pathlib.Path(trace);output=pathlib.Path(output)
 if output.exists():raise ValueError('Fresh output required')
 with trace.open(newline='',encoding='utf-8') as f:
  reader=csv.DictReader(f)
  if not REQUIRED.issubset(reader.fieldnames or []):raise ValueError('Trace schema')
  rows=list(reader)
 if not rows:raise ValueError('Empty trace')
 previous=-1
 for row in rows:
  for key in NUMERIC:
   if key not in row:continue
   row[key]=None if row[key]=='' else float(row[key])
   if row[key] is not None and not math.isfinite(row[key]):raise ValueError('Nonfinite trace')
  if any(row.get(k) is None for k in ('timeS','robotX','robotY','robotHeadingRad','intakeAngleDeg','intakeLengthM','shooterRpm','shotCount')):raise ValueError('Missing required numeric state')
  if row['timeS'] is None or row['timeS']<previous:raise ValueError('Trace clock')
  previous=row['timeS']
 contract_path=ROOT/'cad/cycle-contract-A2.json'
 contract=json.loads(contract_path.read_text())
 if contract.get('units')!='m':raise ValueError('Contract units')
 payload={'contract':contract,'contractSha256':hashlib.sha256(contract_path.read_bytes()).hexdigest(),'traceSha256':hashlib.sha256(trace.read_bytes()).hexdigest(),'rows':rows}
 template=(ROOT/'scripts/a2-replay-template.html').read_text(encoding='utf-8')
 output.parent.mkdir(parents=True,exist_ok=True)
 output.write_text(template.replace('__DATA__',json.dumps(payload,allow_nan=False).replace('</','<\\/')),encoding='utf-8')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--trace',required=True);p.add_argument('--output',required=True);a=p.parse_args();render(a.trace,a.output)
