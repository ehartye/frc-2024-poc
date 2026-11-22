"""Instantiate the original, source-registered shooter model as a stepwise free-NOTE process."""
from pathlib import Path
import builtins,sys,json,hashlib
import numpy as np
from types import SimpleNamespace
p=Path(__file__).resolve().parent;old=p.parent/'counted-reset-power-02'
def read(f):return json.loads(f.read_bytes())
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
class Ready(Exception):pass
def make(tag,n,duration):
    sf=old/f'split-tail_.25_0-{tag}-{n}-6.88.json'
    s=read(sf);assert s['sourceCarry']==sha(old/f'{tag}-{n}.json')
    saveArgs=sys.argv[:];compiler=builtins.compile;ctx={}
    def capture(source,name,*args,**kw):
        if name=='bounded shot':
            ctx.update(src=source,env=sys._getframe(1).f_locals['env'].copy(),args=sys.argv[:]);raise Ready
        return compiler(source,name,*args,**kw)
    try:
        builtins.compile=capture;sys.argv=[str(old/'counter_tail.py'),tag,str(n),'.25','0']
        exec(compiler((old/'counter_tail.py').read_text(),'load registered firing model','exec'),{'__file__':str(old/'counter_tail.py')})
    except Ready:pass
    finally:builtins.compile=compiler;sys.argv=saveArgs
    src=ctx['src'];env=ctx['env'];args=ctx['args']
    src=src[:src.index("if __name__=='__main__'")]
    assert src.count('(7 if drive else .55)')==1
    src=src.replace('(7 if drive else .55)',str(duration))
    marker='        maxfloor=max(maxfloor,center[2]);d.xfrc_applied[b]=wr;'
    assert src.count(marker)==1
    src=src.replace(marker,"        extra=yield State(center=center.copy(), points=pts, rotation=world.copy(), velocity=v.copy(), omega=omega.copy(), q=q, t=d.time, bus=bus, release=release)\n        if extra is not None:wr+=extra\n"+marker)
    src=src[:src.index('    opening=opening_passage(rows)')]+'    return\n'
    env.update(State=SimpleNamespace,sys=SimpleNamespace(argv=args))
    # The source imports the real sys module; bind the frozen arguments after module definitions are read.
    exec(compiler(src,'same solver with force port','exec'),env);env['sys']=SimpleNamespace(argv=args)
    n=int(args[6]);gen=env['shot'](args[1],float(args[2]),float(args[3]),bool(int(args[4])),float(args[5]),n,withfield=bool(int(args[7])))
    return gen,env,s

