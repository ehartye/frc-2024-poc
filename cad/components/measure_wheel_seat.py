"""Measure private native wheel seat and a nominal tread/rivet grip stack."""
import pathlib,json,hashlib,sys,math,argparse
from build_cartridge import *
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_IN
from OCP.TopoDS import TopoDS
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepClass3d import BRepClass3d_SolidClassifier

def measure(cache):
 c=json.loads((R/'cad/components/a2-wheel-cartridge.json').read_text());src=c['sources']['wheel'];p=cache/(src['part']+'.step');raw=p.read_bytes()
 if hashlib.sha256(raw).hexdigest()!=src['sha256']:raise ValueError('wheel source mismatch')
 reader=STEPControl_Reader();reader.ReadFile(str(p));reader.TransferRoots();shape=reader.OneShape();faces=[];e=TopExp_Explorer(shape,TopAbs_FACE)
 while e.More():
  face=TopoDS.Face(e.Current());s=BRepAdaptor_Surface(face)
  if s.GetType()==GeomAbs_Cylinder:
   cy=s.Cylinder();loc=cy.Location()
   if cy.Radius()>40 and abs(loc.X())<1e-5 and abs(loc.Y())<1e-5:
    b=Bnd_Box();BRepBndLib.Add_s(face,b);faces.append({'radiusMm':cy.Radius(),'zMinMm':b.CornerMin().Z(),'zMaxMm':b.CornerMax().Z()})
  e.Next()
 mid=c['layout']['wheelWidth']/2;atmid=[f for f in faces if f['zMinMm']<mid<f['zMaxMm']];outer=max(atmid,key=lambda f:f['radiusMm']);inner=min(atmid,key=lambda f:f['radiusMm']);witness=[]
 for theta in [15,30,45]:
  for radius in [42,42.3,45.5,45.8,46.8]:
   a=math.radians(theta);cl=BRepClass3d_SolidClassifier(shape,gp_Pnt(radius*math.cos(a),radius*math.sin(a),mid),1e-7);witness.append({'thetaDeg':theta,'radiusMm':radius,'zMm':mid,'inside':cl.State()==TopAbs_IN})
 tx=json.loads((R/'cad/components/a2-transmission.json').read_text());t=tx['serviceInterfaces']['tread'];stack=outer['radiusMm']-inner['radiusMm']+t['thicknessMm'];grip=t['rivetGripRangeMm']
 return {'schema':'frc-wheel-seat-evidence/1','source':src,'sha256':hashlib.sha256(raw).hexdigest(),'concentricFaces':faces,'midwidthSeatRadiusMm':outer['radiusMm'],'midwidthInnerRadiusMm':inner['radiusMm'],'seatAxialBoundsMm':[outer['zMinMm'],outer['zMaxMm']],'treadCenterOffsetFromAssemblyMm':(outer['zMinMm']+outer['zMaxMm'])/2-c['layout']['wheelWidth']/2,'pointWitnesses':witness,'finishedRadiusMm':outer['radiusMm']+t['thicknessMm'],'treadNominalThicknessMm':t['thicknessMm'],'rivetPart':t['rivetCandidate'],'rivetSource':t['rivetSource'],'nominalGripStackMm':stack,'rivetGripRangeMm':grip,'nominalGripScreenPass':grip[0]<=stack<=grip[1],'limits':['Native nominal faces, not manufactured metrology','Gray tread thickness/compression/wear not measured','Rivet head shape, upset, backing washer and grip tolerances not modeled','No fastening or operating acceptance']}
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--vendor-cache',required=True,type=pathlib.Path);v=a.parse_args();r=measure(v.vendor_cache);print(json.dumps(r,indent=2));(R/'cad/components/wheel-seat-evidence.json').write_text(json.dumps(r,indent=2)+'\n')
