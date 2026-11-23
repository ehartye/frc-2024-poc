from pathlib import Path
import contextlib, functools, http.server, subprocess, tempfile, threading, shutil, json, hashlib
src=Path(__file__).resolve().parent
root=Path(tempfile.mkdtemp(prefix='frc-2024-pages-'))
clone=root/'checkout'
subprocess.run(['git','clone','--quiet','--depth','1','--filter=blob:none','--sparse','--single-branch','--branch','feature/preliminary-design-review','https://github.com/ehartye/frc-2024-poc.git',str(clone)],check=True)
subprocess.run(['git','sparse-checkout','set','docs/preliminary-design'],cwd=clone,check=True)
dest=root/'site'/'frc-2024-poc'
shutil.copytree(clone/'docs',dest)
class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self,*args):pass
with http.server.ThreadingHTTPServer(('127.0.0.1',0),functools.partial(Quiet,directory=dest.parent)) as server:
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        js=(src/'test_pages.cjs').read_text().replace('18763/preliminary-design',str(server.server_port)+'/frc-2024-poc/preliminary-design')
        test=root/'verify.cjs';test.write_text(js,encoding='utf8')
        subprocess.run(['node',str(test)],check=True)
    finally:server.shutdown();thread.join()
out={i.name:hashlib.sha256((i/'scene.json.gz').read_bytes()).hexdigest() for i in (dest/'preliminary-design/releases').iterdir() if i.is_dir()}
assert all(json.loads((dest/'preliminary-design/releases'/i/'release.json').read_bytes())['sceneSha256']==sha for i,sha in out.items())
print('Verified',subprocess.check_output(['git','rev-parse','HEAD'],cwd=clone,text=True).strip(),out)

