from pathlib import Path
import json,math
p=Path(__file__).resolve().parent
web=p.parent.parent/'www'
def change(f,before,after):
    s=f.read_text(encoding='utf8')
    if after in s:return
    assert s.count(before)==1,(f,before)
    f.write_text(s.replace(before,after),encoding='utf8')
change(web/'review.js',"const conceptKey=new URLSearchParams(location.search).get('concept'),variant=data.concepts?.[conceptKey];","const query=new URLSearchParams(location.search), requested=query.get('case');\n const conceptKey=query.get('concept')||(!requested||data.concepts?.field?.cases.some(c=>c.key===requested)?'field':null),variant=data.concepts?.[conceptKey];")
change(web/'review.js',"if(!variant)link.searchParams.delete('concept');","if(variant)link.searchParams.set('concept',conceptKey);else link.searchParams.delete('concept');")
change(web/'review.js',"""const notes=data.notes||[];""","""if(variant?.validationText)$('validation').textContent=variant.validationText;
 const notes=data.notes||[];""")
change(web/'index.html','/?concept=split&amp;case=split-96','/?concept=field&amp;case=field-96')
change(web/'index.html','Split-feeder trial','Field review')
change(p/'scene.py',"new=copy.deepcopy(old);","""parkCase=rigs(copy.deepcopy(next(k for k in doc['cases'] if k['key']=='park-route')))
parkCase.update(key='field-park',id='PARK fallback route',basis=parkCase['basis']+' Earlier field route with the arm lowered. Clearance with the latest keeper and motor brackets remains to be rechecked.')
cases.append(parkCase)
new=copy.deepcopy(old);""")
change(p/'scene.py',"new['required']=copy.deepcopy(doc['required'])","""new['validationText']=f"{sweep['kernelComparisons']:,} narrow-phase geometry comparisons over 122 sampled robot poses. {approach['exactChainChecks']} field-chain entry comparisons. The live contact model and its held-open control cover the climb. Continuous clearance between sampled poses remains open."
duty=read(p/'four-wheel-duty.json')
for row in new['power']:
    if row['case'].startswith('Loaded drive'):
        aux=row['lines'][0]['amps'][0];case=next(r for r in duty['groups'] if r['gear']==5.5 and r['loadedAuxA']==aux)
        bus=case['minBusV'];amps=(11.8-bus)/.027
        row.update(loadedBusV=[bus,11.8-.027*aux],batterySideA=[aux,amps],marginV=bus-8,status='pass' if not case['failed'] else 'hold')
        row['lines'][1]['amps']=[0,amps-aux]
new['required']=copy.deepcopy(doc['required'])""")
s=(p.parent/'split-cycle-01/swerve_duty.py').read_text(encoding='utf8').replace("six-piece-02/plan.json","split-field-02/plan.json").replace("(p/'budget.json').read_bytes())['runningKg']","(p/'review.json').read_bytes())['running']")
exec(compile(s,'updated route duty','exec'),dict(__file__=str(p/'latest-drive.py')))


