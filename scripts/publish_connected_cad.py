"""Normalize only the public STEP copy; preserve source-export receipt hashes."""
import hashlib,json,pathlib
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
ROOT=pathlib.Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 source=ROOT/'artifacts/connected-intake-candidate/floor/original-robot.step'
 receipt=ROOT/'cad/assembly/evidence/connected-report.json'
 report=json.loads(receipt.read_text());floor=next(s for s in report['states'] if s['state']=='floor')
 assert sha(source)==floor['stepSha256'],'raw export differs from receipt'
 dest=ROOT/'cad/assembly/connected-floor.step'
 dest.write_bytes(b'\n'.join(line.rstrip(b' \t') for line in source.read_bytes().splitlines())+b'\n')
 reader=STEPControl_Reader()
 assert reader.ReadFile(str(dest))==IFSelect_RetDone
 reader.TransferRoots();assert reader.NbRootsForTransfer()==floor['reopenedRoots']==711
 p=ROOT/'cad/assembly/connected-visual.json';m=json.loads(p.read_text())
 m['sourceExportStepSha256']=sha(source);m['originalStepSha256']=sha(dest)
 m['publication']={'operation':'LF line endings and trailing space/tab removal only; raw source export preserved','script':'scripts/publish_connected_cad.py','scriptSha256':sha(pathlib.Path(__file__)),'reopenedRoots':reader.NbRootsForTransfer()}
 p.write_text(json.dumps(m,indent=2)+'\n',encoding='utf-8',newline='\n')
 print(json.dumps(m['publication']))
if __name__=='__main__':main()
