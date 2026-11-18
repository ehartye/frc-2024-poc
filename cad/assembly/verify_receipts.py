"""Byte-level consistency gate for exported pose artifacts; not physical acceptance."""
import hashlib,json,pathlib
STATES=('floor','rotated','handoff')
def sha(path):return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()
def verify_exports(root,expected_inputs):
 root=pathlib.Path(root);reasons=[];hashes={}
 try:r=json.loads((root/'report.json').read_text(encoding='utf-8'))
 except (OSError,ValueError):return {'valid':False,'reasons':['missing or unreadable build receipt'],'stepHashes':{}}
 if r.get('schema')!='2024-intake-component-candidate/1':reasons.append('wrong build schema')
 if r.get('inputs')!=expected_inputs:reasons.append('source dependency mismatch')
 rows=r.get('states',[])
 if not isinstance(rows,list) or len(rows)!=3 or sorted(x.get('state','') for x in rows if isinstance(x,dict))!=sorted(STATES):reasons.append('exact pose inventory required')
 else:
  for x in rows:
   state=x['state'];p=root/state/'original-robot.step'
   if not p.is_file():reasons.append('missing STEP '+state);continue
   digest=sha(p)
   if digest!=x.get('stepSha256'):reasons.append('STEP hash mismatch '+state)
   else:hashes[state]=digest
   if type(x.get('parts')) is not int or x['parts']<1 or x.get('reopenedRoots')!=x['parts']:reasons.append('invalid STEP root count '+state)
 return {'valid':not reasons,'reasons':reasons,'stepHashes':hashes if not reasons else {}}
def source_stable(before,after):return before==after
