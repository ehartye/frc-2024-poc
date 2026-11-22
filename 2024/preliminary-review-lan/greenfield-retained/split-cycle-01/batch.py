"""Bounded check with the selected geometry and controller frozen before running the cases."""
from pathlib import Path
import json,subprocess,sys,hashlib
from concurrent.futures import ThreadPoolExecutor
here=Path(__file__).resolve().parent
cases=[('R01','inner',96),('R02','center-high',96)]+[('dev',k,n) for n in (48,96) for k in ('outer','inner','center-high','center-mid','center-low','middle')]
def run(case):
    attempt,k,n=case
    label=f'{attempt}-{k}'
    with (here/f'{label}-{n}-full.log').open('w') as log:
        subprocess.run([sys.executable,'split_intake.py',k,attempt,str(n)],cwd=here,stdout=log,stderr=log,check=True)
        carry=here/f'{label}-{n}.json'
        d=json.loads(carry.read_bytes())
        row={'attempt':attempt,'id':k,'segments':n,'seated':d['seated'],'held':d['held'],'carrySHA':hashlib.sha256(carry.read_bytes()).hexdigest()}
        if d['held'] and d['seated']:
            subprocess.run([sys.executable,'split_fire.py',label,str(n),'ready','8.05'],cwd=here,stdout=log,stderr=log,check=True)
            fn=here/f'split-ready-{label}-{n}-8.05.json';shot=json.loads(fn.read_bytes());assert shot['sourceCarry']==row['carrySHA']
            row.update(scored=shot['passShot'],both=shot['both'],volts=shot['minBusV'],opening=shot['opening'],shotSHA=hashlib.sha256(fn.read_bytes()).hexdigest())
        (here/f'result-{label}-{n}.json').write_text(json.dumps(row,indent=2))
        print(row,flush=True)
with ThreadPoolExecutor(max_workers=5) as pool:list(pool.map(run,cases))

