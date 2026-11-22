from pathlib import Path
import shutil,zipfile,subprocess,hashlib,json
q=Path(__file__).parent;site=Path.home()/'repos/frc-design-gallery-site/dist'
native=Path.home()/'repos/frc-2024-poc-worktrees/engineering-iterations/iterations/002/design/cad/output-key'
a=json.loads((native/'active-project-check.json').read_bytes());assert a['archiveSha256'].startswith('f83c78cb') and a['stepSha256'].startswith('869021c2')
p=native/'002-supported-output-key.f3d'; assert hashlib.file_digest(p.open('rb'),'sha256').hexdigest()==a['archiveSha256']
entry='FusionAssetName[Active]/Previews/big.png'
b=subprocess.check_output(['py','-3.14','-c','import sys,zipfile;sys.stdout.buffer.write(zipfile.ZipFile(sys.argv[1]).read(sys.argv[2]))',str(p),entry])
assert b==(native/'original-preview.png').read_bytes()
(site/'images/2024-keyed-scorer.png').write_bytes(b);shutil.copy2(q/'2024-keyed-shaft-detail.png',site/'images');print('overview',hashlib.sha256(b).hexdigest())
f=site/'index.html';h=f.read_text(encoding='utf8');a0=h.index('<article data-season="2024">');a1=h.index('<article data-season="2025">');piece=h[a0:a1]
old=next(line for line in piece.splitlines() if 'class="drawing"' in line);history=old.strip().split(' aria-label=')[0].replace('class="drawing"','class="extra"')+'>First left scoring wheel</a>'
new='<a class="drawing" href="./images/2024-keyed-scorer.png" data-issue="f83c78cb" data-sequence="100" data-title="2024 &middot; Keyed scoring-wheel shaft" data-note="The new key fits in a closed pocket on the output shaft. In Fusion, the wheel, shaft and key turned together through ±13°. The close-up shows the key highlighted in gold. The motor coupling remains open. Fusion f83c78cb, placed STEP 869021c2." aria-label="Enlarge the 2024 assembly with its keyed scoring-wheel shaft"><img src="./images/2024-keyed-scorer.png" alt="Parked swerve chassis, feeder support and left scoring wheel with its keyed shaft"><span class="expand" aria-hidden="true">&#x26F6;</span></a>'
piece=piece.replace(old,'        '+new).replace('Iteration 002 · Left scoring wheel','Iteration 002 · Keyed scoring wheel').replace('1,238 solids &middot; <code>f43e12d1','1,239 solids &middot; <code>f83c78cb')
start=piece.index('<h3>');stop=piece.index('</p>',start)+4
piece=piece[:start]+'<h3>Keyed scoring-wheel shaft</h3><p>A steel key now sits in a milled pocket on the left scoring-wheel shaft. Open the close-up to see the two pieces. The motor coupling is still in design.</p>'+piece[stop:]
sub='<div class="archives">'; assert piece.count(sub)==1
detail='<a class="extra" href="./images/2024-keyed-shaft-detail.png" data-related-to="f83c78cb" data-view-label="Shaft close-up" data-title="2024 &middot; Steel key and shaft pocket" data-note="Close-up of the two installed solids from the parked Fusion export. Gold highlights the purchased key. Moving it 0.30 mm in either axial direction engaged the pocket ends in the geometry check.">See the key and shaft pocket</a>'
piece=piece.replace(sub,sub+detail+'\n          '+history+'\n          ')
f.write_text(h[:a0]+piece+h[a1:],encoding='utf8')

