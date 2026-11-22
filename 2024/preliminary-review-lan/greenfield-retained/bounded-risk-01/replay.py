"""Reproduce selected risk cases in a new sibling folder, preserving the original evidence."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil,subprocess,sys
p=Path(__file__).resolve().parent
runs={
'pickup.py':[
'p-original 48 0 -.5 -2 0 .45 8','p-baseline 48 0 0 0 0 .75 8','p-default 48 .75 0 0 0 .75 8',
'p-low 48 .75 -.5 -2 -.05 .45 8','p-high 48 .75 .5 3 .05 .45 8','p-weak 48 .75 .5 3 .05 .45 2',
'p-off 48 .75 0 0 0 .75 0','p-fine 96 .75 .5 3 .05 .45 8'],
'brake.py':['b-center .001 0 180 .03','b-forward .001 .04 180 .03','b-rear .001 -.04 180 .03','b-low .001 .04 80 .03',
'b-delay .001 .04 180 .12','b-none .001 .04 0 .03','b-fine .0005 .04 180 .03'],
'power.py':['s-armed guarded 48','s-fine guarded 96','s-bypass bypass 48','s-stuck stuck 48']}
cases={a.split()[0]:(tool,a.split()) for tool,items in runs.items() for a in items}
args=argparse.ArgumentParser(description=__doc__)
args.add_argument('case',nargs='*',help='optional case IDs, default all 19')
selected=args.parse_args().case or list(cases)
if set(selected)-set(cases):args.error('unknown case ID')
out=p.parent/('bounded-risk-01-replay-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
out.mkdir()
for filename in (*runs,'PLAN.md','report.py'):shutil.copyfile(p/filename,out/filename)
original=json.loads((p/'results.json').read_bytes())
def sha(file):return hashlib.sha256(file.read_bytes()).hexdigest()
comparison={}
for name in selected:
    tool,argv=cases[name]
    subprocess.run([sys.executable,str(out/tool),*argv],cwd=out,check=True)
    actual=sha(out/(name+'.json'));expected=original['cases'][name]['sha256']
    comparison[name]=dict(matches=actual==expected,expected=expected,actual=actual)
    (out/'comparison.json').write_text(json.dumps(comparison,indent=2))
if set(selected)==set(cases):subprocess.run([sys.executable,str(out/'report.py')],cwd=out,check=True)
print(out,comparison)
if not all(v['matches'] for v in comparison.values()):sys.exit('Reproduction differed; inspect the new evidence.')

