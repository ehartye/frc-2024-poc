import json,hashlib
from pathlib import Path
q=Path(__file__).parent;f=q/'fit-decision.json';r=json.loads(f.read_bytes())
r['freePieceObserver']={}
for n in [96,192]:
 g=q/f'free-ring-{n}.json';s=json.loads(g.read_bytes());row=s['limitingEnvelopePoseCheck']
 r['freePieceObserver'][n]={'receipt':g.name,'sha256':hashlib.sha256(g.read_bytes()).hexdigest(),'samplesPerPiece':s['poses'],'pieceIDs':['blue-source/1','blue-middle/1'],'equipmentArticleEnvelopesPerPose':19,'globalSampledLowerBoundMm':row['certifiedSampledLowerBoundMm'],'worstTimeS':row['t'],'worstPiece':row['ring'],'worstArticle':row['part'],'exactBRepDistanceAtThisPoseMm':row['exactTorusToSolidMm'],'exactCommonMm3':row['exactCommonMm3']}
r['limits'].append('Free pieces checked against purchased articles on unchanged archived dynamics. New supports, wires, guards, sensing, coupled carriage and changed moving masses/inertias have not yet been modeled or qualified.');f.write_text(json.dumps(r,indent=2));print(hashlib.sha256(f.read_bytes()).hexdigest())
