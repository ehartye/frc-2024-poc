from pathlib import Path
here=Path(__file__).resolve().parent
method=here.parent/'chain-entry-01'
s=(method/'root_approach.py').read_text(encoding='utf8')
s=s.replace("base=Path(__file__).resolve().parent","base=method")
import sys;sys.path.insert(0,str(method))
s=s.replace("root/'follower-01/paired-follower.step'","here/'keeper-drive.step'")
s=s.replace("P=mat(", """pkg=json.loads((here/'package.json').read_bytes());r=json.loads((here/'result.json').read_bytes())
for n in pkg['removed']:roster.pop(n,None)
roster.update({n:'arm' for n in pkg['added']});roster.update(r['rig']);d['step']=r['step']
P=mat(""")
s=s.replace("if n.startswith('Hook keeper')","if role=='keeper'").replace("(x,35) for x in np.linspace(238,180,15)","(x,44) for x in np.linspace(238,180,30)").replace('np.linspace(35,0,9)','np.linspace(44,0,23)')
s=s.replace("nk='Hook keeper · '+side","nk='Common keeper yoke and sliding rack'").replace("p/'entry-progress.json'","here/'entry-progress.json'").replace("p/'root-entry.json'","here/'root-entry.json'")
exec(compile(s,'complete revised chassis against chain','exec'))
