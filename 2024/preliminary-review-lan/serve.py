from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
import sys
root=Path(__file__).resolve().parent/'www'
files={'/':('index.html','text/html; charset=utf-8'),'/resources.js':('resources.js','text/javascript; charset=utf-8'),'/review.js':('review.js','text/javascript'),'/review.css':('review.css','text/css'),'/data.json':('data.json','application/json'),'/presentation.md':('presentation.md','text/plain; charset=utf-8')}
class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        route=urlsplit(self.path).path
        if route=='/version':
            content=str(max((root/p).stat().st_mtime_ns for p,_ in files.values())).encode()
            typ='text/plain'
        elif route in files:
            name,typ=files[route]
            content=(root/name).read_bytes()
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type',typ)
        self.send_header('Content-Length',str(len(content)))
        self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff')
        self.end_headers()
        self.wfile.write(content)
    def log_message(self,*args):pass
ThreadingHTTPServer((sys.argv[1],int(sys.argv[2])),Handler).serve_forever()
