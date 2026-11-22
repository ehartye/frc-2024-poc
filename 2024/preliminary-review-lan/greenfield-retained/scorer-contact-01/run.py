from pathlib import Path
import json,sys,numpy as np,hashlib,math
from patch import belt_patch,coefficients
p=Path(__file__).resolve().parent;root=p.parent/'six-piece-02';method=p.parent/'follower-01'
key,segments,methodName=sys.argv[1:4];n=int(segments);src=root/f'dev-{key}-{n}.json';incoming=json.loads(src.read_bytes());priorHash=hashlib.sha256(src.read_bytes()).hexdigest()
assert incoming['held'] and incoming['seated']
s=(method/'fire.py').read_text();s=s[s.index('old=(p.parent/'):s.index('incoming=a;')]
s=s.replace("name,mode,n=sys.argv[1:]", "name,mode,n=sys.argv[1:]")
s=s.replace("old=(", "old=(")
# Adopt same station and rear-stop indexing policy for the two contact hypotheses.
ix=(root/'index_shoot.py').read_text();indexPatch=ix.split('patch="""')[1].split('"""')[0]
s=s.replace("old=old.replace('env.update(policy=',extra+", "old=old.replace('env.update(policy=',extra+indexPatch+additional+")
s=s.replace("sys.argv=[__file__,'fire'+n", "sys.argv=[__file__,methodName+'-'+key+'-'+str(n)")
s+= "\nold=old.replace(\"'125','1.43'\",\"'125','1.53'\");exec(compile(old,'bounded return station','exec'))\n"
additional="""
start=src.index('            diff=local-nearest;ds=np.linalg.norm')
end=src.index('        maxCurrent=',start)
if methodName=='patch':
    src=src[:start]+'''            diff=local-nearest
            wrp,N,F,arc,slipW=belt_patch(diff,pts,center,v,omega,R,speed,dt,[*params[:4],18 if active else 9])
            wr+=wrp;normal+=N;peak=max(peak,abs(F));work+=abs(F)*abs(speed)
            belts[jj]+=dt*(Fmotor-F-.4*speed)/inertia
            maxArc=max(maxArc,arc);maxSlipW=max(maxSlipW,slipW)
'''+src[end:]
sub('    permitFor=0.', '    permitFor=0.') if False else None
sub('    rows=[];coarse=[]','    maxArc=0.;maxSlipW=0.;rows=[];coarse=[]')
sub('result=dict(indexFirst=', 'result=dict(maxContactArcM=maxArc,peakSlipW=maxSlipW,contactModel=choice,resultingCoefficients=params,indexFirst=')
env.update(choice=methodName,params=coefficients(),belt_patch=belt_patch)
"""
exec(compile(s,'field contact sensitivity','exec'))


