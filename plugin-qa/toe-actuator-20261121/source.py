import urllib.request as u, pathlib, hashlib,zipfile
q=pathlib.Path(__file__).parent
urls={'REV-41-1665.STEP':'https://www.revrobotics.com/content/cad/REV-41-1665.STEP',**{k+'.zip':'https://www.gobilda.com/content/step_files/'+k+'.zip' for k in ['2311-0001-0131','2311-0001-0118']}}
for fn,url in urls.items():
 p=q/fn; data=u.urlopen(u.Request(url,headers={'User-Agent':'Mozilla/5.0'}),timeout=18).read();p.write_bytes(data)
 if fn.endswith('.zip'):
  for z in zipfile.ZipFile(p).infolist():
   if z.filename.lower().endswith(('.step','.stp')):
    b=zipfile.ZipFile(p).read(z); (q/(fn[:-4]+'.step')).write_bytes(b);print(z.filename,z.date_time,b[:210],hashlib.sha256(b).hexdigest())
 else: print(fn,data[:180],hashlib.sha256(data).hexdigest())
