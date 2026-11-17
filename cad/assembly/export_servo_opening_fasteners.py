"""Export only original authored components, never supplier shapes/envelopes."""
import pathlib,json,hashlib
import servo_opening_fasteners as m
def run():
 parts=[(n,p,k) for n,p,k in m.local_parts(0) if k in ['original-stock','original-cam']]
 if len(parts)!=4:raise ValueError('unexpected export inventory')
 out=m.s.a.ROOT/'artifacts/spring-cassette/m3-opening-original.step';out.parent.mkdir(parents=True,exist_ok=True)
 w=m.h.STEPControl_Writer()
 for n,p,k in parts:
  if not m.h.BRepCheck_Analyzer(p).IsValid():raise ValueError(n)
  w.Transfer(p,m.h.STEPControl_AsIs)
 w.Write(str(out));print(out.name,out.stat().st_size,hashlib.sha256(out.read_bytes()).hexdigest(),flush=True)
if __name__=='__main__':run()
