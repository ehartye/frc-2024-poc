from pathlib import Path
import urllib.request as u,re,hashlib,zipfile
q=Path(__file__).parent
for url,code in [('https://www.revrobotics.com/rev-41-1503/','REV-41-1503'),('https://www.revrobotics.com/5mm-Plastic-Hex-Bore-Gears/','REV-41-1332')]:
 s=u.urlopen(url,timeout=15).read().decode()
 urls=re.findall(r'href="([^"]+)"',s)
 addr=next(w for w in urls if code in w.upper() and '.STEP' in w.upper())
 if addr.startswith('/'): addr='https://www.revrobotics.com'+addr
 b=u.urlopen(addr,timeout=15).read();(q/(code+'.step')).write_bytes(b);print(code,addr,len(b),hashlib.sha256(b).hexdigest(),b[:280])
z=u.urlopen('https://www.revrobotics.com/content/cad/REV-41-1600-ALL-PARTS.zip',timeout=20).read();(q/'REV-41-1600-ALL-PARTS.zip').write_bytes(z)
with zipfile.ZipFile(q/'REV-41-1600-ALL-PARTS.zip') as f:
 for n in f.namelist():
  print(n)
