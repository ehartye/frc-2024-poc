import pathlib,json,hashlib
import opening_support_compliance as c
import cassette_spring_law as spring
ROOT=pathlib.Path(__file__).resolve().parents[2]
def run():
 rows=[]
 for label,force in [('nominal',spring.spring_state(0)['pairInwardForceN']),('seat_plus1mm_rate_plus10percent',spring.spring_state(0,.001,.1)['pairInwardForceN'])]:
  for rev in ['slender','wide']:rows.append({'case':label,**c.screen(force,revision=rev)})
 pins={}
 for n in ['opening_support_compliance.py','servo_opening_relocation.py','servo_opening_fasteners.py','servo_opening_wide_support.py','cassette_spring_law.py','check_opening_support_compliance.py']:
  p=ROOT/'cad/assembly'/n;pins['cad/assembly/'+n]=hashlib.sha256(p.read_bytes()).hexdigest()
 return {'inputs':pins,'correctsHistoricalSnapshot':'c564644 lower-face datum replaced by arm centroid; prior receipt preserved','loadApplicationLocalXmm':146,'rows':rows,'screeningBudgetMm':.15,'adopted':False,'limits':['Moduli69GPa/26GPa are illustrative hypotheses, not sourced stock selection','Centerline beam idealization includes finite joint zones as line elements; it omits weld/anchor/frame compliance and dynamic impact','Square torsion constant0.1406 is an approximation','Free-length tolerance and actual contact load envelope unknown; selected spring cases not worst-case proof','0.15mm budget is provisional10percent of1.5mm travel, not validated mechanism tolerance']}
if __name__=='__main__':
 r=run();(ROOT/'cad/assembly/evidence/opening-support-compliance.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf-8',newline='\n');print([(x['case'],x['revision'],x['totalEstimatedMm']) for x in r['rows']])
