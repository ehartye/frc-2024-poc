"""Denser sampled audit layered on the source-bound moving/fixed screen."""
import pathlib,json,math,hashlib
import audit_intake_motion as audit

def dense_poses(c):
 count=max(1,math.ceil(abs(c['pitchDeg'])))
 for j in range(count+1):
  deg=c['pitchDeg']*j/count;yield 'rotate',deg,c['floorCenterZ']+c['note']['majorRadius']*math.sin(math.radians(deg))
 start=c['floorCenterZ']+c['note']['majorRadius']*math.sin(math.radians(c['pitchDeg']));count=max(1,math.ceil(abs(c['handoffCenterZ']-start)/5))
 for j in range(1,count+1):yield 'lift',c['pitchDeg'],start+(c['handoffCenterZ']-start)*j/count
if __name__=='__main__':
 audit.poses=dense_poses;q=audit.run();q['density']={'maximumPitchStepDeg':1,'maximumLiftStepMm':5};q['limits'][0]=f"Finite {len(q['samples'])}-pose screen, not continuous sweep";q['inputs']['cad/assembly/audit_dense_motion.py']=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest();p=audit.a.ROOT/'artifacts/intake-component-candidate/motion-dense.json';p.write_text(json.dumps(q,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps({'samples':len(q['samples']),'collisionSamples':sum(bool(x['hits']) for x in q['samples']),'pairKinds':len({(h['moving'],h['fixed']) for x in q['samples'] for h in x['hits']})}))
