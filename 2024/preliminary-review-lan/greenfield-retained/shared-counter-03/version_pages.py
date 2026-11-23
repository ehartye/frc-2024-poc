from pathlib import Path
import gzip,json,shutil,sys,hashlib
repo=Path(sys.argv[1]);root=repo/'docs/preliminary-design'
current=root/'releases/002';old=root/'releases/001'
current.mkdir(exist_ok=True)
for fn in ('review.js','review.css','resources.js','index.html','presentation.md'):
    shutil.copyfile(old/fn,current/fn)
raw=gzip.decompress((old/'scene.json.gz').read_bytes())
def make(folder,ids,title,history):
    d=json.loads(raw);g=d['concepts']['guided'];field=d['concepts']['field']
    g['cases']=[c for c in g['cases'] if c['key'] in ids];assert len(g['cases'])==len(ids)
    g['defaultCase']=ids[0];g['inherit']=[];g['issue']=title
    field['cases']=[];d['cases']=[];d['concepts']={'guided':g,'field':field}
    if history:
        d.pop('slideService',None)
        g['status']='Historical preliminary release'
        g['focus']='Archived bounded-lift trial. Its earlier carry visualization is superseded by release 002. The saved individual-shot run is preserved with its original scope.'
        g['notes']=[g['focus']]
    cut=current/'scene.json.gz' if not history else old/'scene.json.gz'
    data=json.dumps(d,ensure_ascii=False,separators=(',',':'),allow_nan=False).encode()
    cut.write_bytes(gzip.compress(data,compresslevel=5,mtime=0))
    manifest=dict(id=folder.name,title=title,defaultCase=ids[0],sceneSha256=hashlib.sha256(cut.read_bytes()).hexdigest(),geometry=g['step'],cases=ids,scope=g['focus'],fixtureRepresentation='Dimensioned presentation primitives. The numerical results use the original field contact model.')
    (folder/'release.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return {k:manifest[k] for k in ['id','title','defaultCase']}
latest=make(current,['continuous-cycle-96','continuous-cycle-48','score-to-climb-front','score-to-climb-rear'],'Continuous six-NOTE cycle and climb',False)
prior=make(old,['loaded-cycle-48','observed-reset-96'],'Bounded lift and empty-cassette reset',True)
(root/'releases.json').write_text(json.dumps(dict(latest=latest['id'],releases=[latest,prior]),indent=2)+'\n')

