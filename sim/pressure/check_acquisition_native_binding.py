"""Reject native qualification when additive geometry overrides are omitted."""
import argparse,hashlib,json,pathlib
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
p=argparse.ArgumentParser()
for name in ['project','native-archive','output']:p.add_argument('--'+name,type=pathlib.Path,required=True)
a=p.parse_args();assert not a.output.exists()
cp=a.project/'cad/assembly/intake_components.json';rp=a.project/'sim/integrated/evidence/track-tread-candidate-20261118.json';hp=a.project/'sim/pressure/fresh-rest-controller-3s-evidence.json'
config=json.loads(cp.read_bytes());native=json.loads(rp.read_bytes());history=json.loads(hp.read_bytes());archive_sha=sha(a.native_archive)
assert archive_sha==native['native']['finalArchiveSha256']
assert sha(cp)==history['inputs']['cad/assembly/intake_components.json']==native['staticCapacity']['onePointFiveMm']['inputs']['cad/assembly/intake_components.json']
old_face=config['pinch']['innerFaceYAbs'];new_face=native['native']['parameters']['nominalContactFaceAbsYmm'];delta=new_face-old_face
assert abs(delta+.5)<1e-10
result={'schema':'acquisition-native-registration-boundary/1','acceptance':False,'status':'REGISTRATION_REQUIRED','scriptSha256':sha(pathlib.Path(__file__)),'nativeArchiveSha256':archive_sha,'nativeReceiptSha256':sha(rp),'acquisitionReportSha256':sha(hp),'baseConfigSha256':sha(cp),'baseConfigHashMatches':True,'historicalAcquisitionInnerFaceAbsYmm':old_face,'nativeCandidateInnerFaceAbsYmm':new_face,'perSideFaceDifferenceMm':delta,'nominalTotalGapDifferenceMm':2*delta,'nativeOverrides':native['native']['parameters'],'automaticNativeQualificationPermitted':False,'limits':['A matching base configuration hash does not prove equality after additive native geometry overrides','Do not relabel or repin the historical pressure trajectory as the relieved native candidate','A distinct newly bound model must include the native offsets, treads, relief and installed transforms and repeat geometry/contact checks','This read-only evidence comparison does not access Fusion, validate its current session, or mutate any native file']}
a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8',newline='\n');print(json.dumps(result))
