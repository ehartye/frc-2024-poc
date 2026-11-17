import hashlib,json,pathlib,urllib.request,zipfile
ROOT=pathlib.Path(__file__).resolve().parent;TOOL=ROOT/'.tools';TOOL.mkdir(exist_ok=True)
def fetch(url,path):
 with urllib.request.urlopen(url,timeout=180) as r,path.open('wb') as f:
  while b:=r.read(1024*1024):f.write(b)
def extract(path):
 with zipfile.ZipFile(path) as z:
  for n in z.namelist():
   if not (TOOL/n).resolve().is_relative_to(TOOL.resolve()):raise ValueError('zip path escape')
  z.extractall(TOOL)
a={'release_name':'jdk-17.0.21+9'}; url='https://github.com/adoptium/temurin17-binaries/releases/download/jdk-17.0.21%2B9/OpenJDK17U-jdk_x64_windows_hotspot_17.0.21_9.zip'; checksum=urllib.request.urlopen(url+'.sha256.txt',timeout=60).read().decode().split()[0]; items=[('temurin17.zip',url,checksum)]
u='https://services.gradle.org/distributions/gradle-8.14.3-bin.zip';checksum=urllib.request.urlopen(u+'.sha256',timeout=60).read().decode().split()[0];items.append(('gradle-8.14.3-bin.zip',u,checksum));receipts=[]
for name,url,expected in items:
 p=TOOL/name;print('download',name,flush=True);fetch(url,p);actual=hashlib.sha256(p.read_bytes()).hexdigest()
 if actual!=expected:raise ValueError('checksum mismatch '+name)
 print('verified',name,actual,flush=True);extract(p);receipts.append({'file':name,'url':url,'sha256':actual,'bytes':p.stat().st_size})
(ROOT/'toolchain-source.json').write_text(json.dumps({'jdkRelease':a['release_name'],'artifacts':receipts,'isolation':'module .tools and .gradle-home only; no global settings'},indent=2)+'\n');print('toolchain ready',flush=True)
