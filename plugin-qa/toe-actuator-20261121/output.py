from pathlib import Path
import urllib.request as u,re,hashlib,sys
from urllib.parse import urljoin
q=Path(__file__).parent
for code in ['REV-41-1615','REV-21-1651']:
 url='https://www.revrobotics.com/'+code.lower()+'/';s=u.urlopen(url,timeout=15).read().decode()
 urls=re.findall(r'href="([^"]+)"',s)
 candidates=[urljoin(url,w) for w in urls if code in w.upper() and ('.STEP' in w.upper() or '.ZIP' in w.upper())];print(candidates)
 for addr in candidates[:1]:
  b=u.urlopen(addr,timeout=15).read();print(len(b),hashlib.sha256(b).hexdigest(),b[:190]);(q/(code+Path(addr).suffix)).write_bytes(b)
