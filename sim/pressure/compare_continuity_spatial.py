import argparse,copy,hashlib,json,math,pathlib,sys
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
p=argparse.ArgumentParser()
for name in ['project','index','transport','contract','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
a=p.parse_args();assert not a.output.exists();sys.path.insert(0,str(a.project/'sim/pressure'))
import run_fresh_rest_controller as fresh
import pressure_tiled_geometry as tiled
fresh.c.factory.r.PressureAcquisition.prepared.__globals__['fast_geometry'].prepare=tiled.prepare
model,controller,pins,bus,ancestry=fresh.load();contract=json.loads(a.contract.read_bytes());hp=a.project/'sim/pressure/fresh-rest-controller-3s-evidence.json';history=json.loads(hp.read_bytes());index=json.loads(a.index.read_bytes());transport=json.loads(a.transport.read_bytes())
assert history['inputs']==index['inputs']==transport['inputs']==pins
states={'first-contact':next(r for r in history['trace'] if r['contacts']),'index':index['trace'][-1],'peak-transport':max(transport['trace'],key=lambda r:math.hypot(*r['reactionForceWorldN']))}
rows=[]
for n in contract['resolutionsPerDimension']:
    model.resolution=(n,n)
    for name,row in states.items():
        cache={};q=model.acquisition_rates(row['note'],row['chassis'],row['busV'],row['command']['pinchDemands'],cache);prepared=model.prepared(row['chassis'],row['note'],cache)
        r={'state':name,'timeS':row['timeS'],'resolution':n,'normalPerSideN':[sum(c['normalN'] for c in prepared['contacts'] if c['motor']==i) for i in range(2)],'reactionForceWorldN':q['reactionForceWorldN'],'potentialJ':prepared['potentialJ'],'contacts':len(prepared['contacts']),'workResidualW':q['pressureWorkResidualW']};rows.append(r);print(json.dumps(r),flush=True)
comparisons=[]
for name in states:
    for one,two in [(512,1024),(1024,2048)]:
        x=next(r for r in rows if r['state']==name and r['resolution']==one);y=next(r for r in rows if r['state']==name and r['resolution']==two);ratios={}
        for field in ['normalPerSideN','reactionForceWorldN','potentialJ']:
            aa=x[field] if isinstance(x[field],list) else [x[field]];bb=y[field] if isinstance(y[field],list) else [y[field]];floor=contract['potentialAbsoluteFloorJ'] if field=='potentialJ' else contract['forceAbsoluteFloorN'];ratios[field]=max(abs(u-v)/max(floor,contract['relativeTolerance']*max(abs(u),abs(v))) for u,v in zip(aa,bb))
        comparisons.append({'state':name,'first':one,'second':two,'maximumToleranceRatios':ratios,'passed':max(ratios.values())<=1})
result={'schema':'pressure-continuity-spatial/1','acceptance':False,'fineChecksPassed':all(r['passed'] for r in comparisons if r['first']==1024),'scriptSha256':sha(pathlib.Path(__file__)),'cullingSha256':sha(pathlib.Path(tiled.__file__)),'contractSha256':sha(a.contract),'sourceHashes':{'historical':sha(hp),'index':sha(a.index),'transport':sha(a.transport)},'rows':rows,'comparisons':comparisons,'limits':contract['limits']}
a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({'fineChecksPassed':result['fineChecksPassed'],'comparisons':comparisons}))
