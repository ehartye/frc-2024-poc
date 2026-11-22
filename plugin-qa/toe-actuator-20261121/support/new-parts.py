from pathlib import Path
import urllib.request as U,zipfile,hashlib,re,sys
from urllib.parse import urljoin
q=Path(__file__).parent
for code,url in [('switch','https://www.gobilda.com/micro-limit-switch-bump-lever-2-pack/'),('bearing','https://www.revrobotics.com/rev-41-1683/')]:
 s=U.urlopen(url).read().decode()
 links=[urljoin(url,v) for v in re.findall(r'href="([^"]+)"',s) if ('.zip' in v.lower() or '.step' in v.lower()) and ('3103' in v or '1683' in v)]
 v=links[0];print(v);out=q/(code+Path(v).suffix);out.write_bytes(U.urlopen(v).read())
 if zipfile.is_zipfile(out):
  z=zipfile.ZipFile(out);fname=next(n for n in z.namelist() if n.lower().endswith('.step'));raw=z.read(fname)
 else:raw=out.read_bytes()
 (q/(code+'.step')).write_bytes(raw);print(len(raw),hashlib.sha256(raw).hexdigest(),raw[:240])
sys.path.insert(0,str(Path.home()/'repos/agent-frc/scripts'))
from showcase_contacts import load_solids
from mechanical_motion import bounds
for code in ['switch','bearing']:
 g,_=load_solids(q/(code+'.step')); print([(n,[round(v,2) for v in bounds(sh)]) for n,sh in g.items()][:45])
