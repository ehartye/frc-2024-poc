from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import subprocess,sys,json,time
p=Path(__file__).resolve().parent
def run(args):
    piece,n=args;k=f'dev-{piece}-{n}'
    with (p/(k+'.log')).open('w') as log:
        for script in ('loaded.py','index_shoot.py'):
            job=subprocess.run([sys.executable,script,piece,'dev',str(n)],cwd=p,stdout=log,stderr=log)
            if job.returncode:return [piece,n,script,job.returncode]
    res=json.loads((p/('indexed-'+k+'.json')).read_bytes());assert res['robotX']==1.53
    return [piece,n,res['passShot'],res['both']]
with ThreadPoolExecutor(max_workers=4) as pool:
    tasks=[pool.submit(run,(k,n)) for n in map(int,sys.argv[1:]) for k in ['middle','outer','inner','center-high','center-mid','center-low']]
    for job in as_completed(tasks):print(job.result(),flush=True)

