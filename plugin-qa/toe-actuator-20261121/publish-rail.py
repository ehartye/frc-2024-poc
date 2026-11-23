from pathlib import Path
import hashlib,subprocess,json
q=Path(__file__).parent;site=Path.home()/'repos/frc-design-gallery-site/dist';c=Path.home()/'repos/frc-2024-poc-worktrees/engineering-iterations/iterations/002/design/cad';src=c/'front-perimeter';a=json.loads((c/'fusion-authority.json').read_bytes());l=json.loads((src/'first-party-lineage/lineage.json').read_bytes());v=json.loads((src/'active-project-check.json').read_bytes())
assert a['source']['archive']==l['archive'] and v['status']=='pass' and l['archive']['sha256']==v['archiveSha256'] and l['step']['sha256']==v['stepSha256'] and v['archiveSha256'].startswith('037a6d49')
arc=src/'002-front-pickup-perimeter.f3d'
assert hashlib.file_digest(arc.open('rb'),'sha256').hexdigest()==v['archiveSha256']
assert hashlib.file_digest((src/'first-party-lineage/assembly.step').open('rb'),'sha256').hexdigest()==v['stepSha256']
b=subprocess.check_output(['py','-3.14','-c','import sys,zipfile;sys.stdout.buffer.write(zipfile.ZipFile(sys.argv[1]).read(sys.argv[2]))',str(arc),'FusionAssetName[Active]/Previews/big.png']);assert b==(src/'original-preview.png').read_bytes()
(site/'images/2024-direct-front-rail.png').write_bytes(b)
f=site/'index.html';h=f.read_text(encoding='utf8');a0=h.index('<article data-season="2024">');a1=h.index('<article data-season="2025">');piece=h[a0:a1]
old=next(line for line in piece.splitlines() if 'class="drawing"' in line);past=old.strip().split(' aria-label=')[0].replace('class="drawing"','class="extra"')+'>Keyed scoring-wheel shaft</a>'
new='<a class="drawing" href="./images/2024-direct-front-rail.png" data-issue="037a6d49" data-sequence="110" data-title="2024 &middot; Front pickup rail" data-note="The new cross tube spans the two front sill ends. Four screws reach it through access holes in its front face. The intake and the rail’s load qualification are still open. Parked Fusion 037a6d49, placed STEP 39dae40b." aria-label="Enlarge the 2024 frame with its front pickup rail"><img src="./images/2024-direct-front-rail.png" alt="A new transverse rail with access holes bridges the front of the four-module frame, ahead of the scoring wheel and angled feeder"><span class="expand" aria-hidden="true">&#x26F6;</span></a>'
piece=piece.replace(old,'        '+new).replace('Iteration 002 · Keyed scoring wheel','Iteration 002 · Front pickup rail').replace('1,239 solids &middot; <code>f83c78cb','1,244 solids &middot; <code>037a6d49')
st=piece.index('<h3>');en=piece.index('</p>',st)+4
piece=piece[:st]+'<h3>Front pickup rail</h3><p>A cross tube now joins the front ends of the two frame rails. Its front face sets the pickup’s reach datum. The four access holes are visible along the near edge of the model.</p>'+piece[en:]
tag='<div class="archives">';piece=piece.replace(tag,tag+past+'\n          ');f.write_text(h[:a0]+piece+h[a1:],encoding='utf8')
ref=hashlib.sha256(b).hexdigest()
r=(q/'gallery-qa.cjs').read_text(encoding='utf-8-sig').replace("'Keyed scoring-wheel shaft'","'Front pickup rail'").replace("'First left scoring wheel'","'Keyed scoring-wheel shaft'").replace('2024-keyed-scorer.png','2024-direct-front-rail.png').replace('532449c23f0e4c35a5f835a24689b972162dafe0646005da43aa7adbecbc0f05',ref)
(q/'gallery-qa-v30.cjs').write_text(r,encoding='utf8');print(ref)

