"""Display the serviceable drive on current replays and preserve each older replay's own membership."""
from pathlib import Path
import json
p=Path(__file__).resolve().parent;web=p.parent.parent/'www';f=web/'data.json'
d=json.loads(f.read_bytes());g=d['concepts']['guided'];s=d['slideService'];gross=json.loads((p.parent/'mounted-slide-02/gross-power.json').read_bytes())
group=[c['key'] for c in g['cases'] if c['key'].startswith(('continuous-cycle-','score-to-climb-'))]
assert len(group)==4
b=min((x for x in gross['cases'] if x['auxA']==25),key=lambda v:v['weakBusMinV'])
row=next(x for x in g['power'] if x['case'].startswith('Loaded lift and drive'))
import copy
row=copy.deepcopy(row);v=b['weakBusMinV'];peak=(11.8-v)/.027
row.update(case='Loaded lift and drive · installed cartridge',loadedBusV=[v,11.125],batterySideA=[25,peak],marginV=v-8,basis=f'At the center-low return the simultaneous bus load is {peak:.1f} A. The limiting motor retains {gross["loaded25A"]["motorReserve"]:.3f} V. Applying 39 A of other loads loses {abs(gross["negative39A"]["motorReserve"]):.3f} V at the motor even though the bus stays above 8 V.')
for line,k in zip(row['lines'],['accessories','fourDrives','pivot','slide']):
    line['amps'][-1]=b['batteryBreakdownAtPeakA'][k]
assert abs(sum(x['amps'][1] for x in row['lines'])-peak)<1e-6
s['currentKeys']=group;s['currentPower']=row
tmp=f.with_suffix('.current-scene');tmp.write_text(json.dumps(d,separators=(',',':')),encoding='utf8');tmp.replace(f)
js=web/'review.js';text=js.read_text(encoding='utf8')
old=" if(conceptKey==='guided' && slideService){"
if 'const sceneGroup=' not in text:
    text=text.replace(old," const activeCase=requested||variant?.defaultCase;\n const sceneGroup=k=>k===slideService?.case.key?'service':slideService?.currentKeys?.includes(k)?'operating':'historical';\n"+old)
    text=text.replace("if(requested===slideService.case.key){","if(sceneGroup(activeCase)!=='historical'){")
    target="     variant={...variant,add:{...variant.add,...slideService.add},mass:[...originalMass,slideService.mass],"
    assert text.count(target)==1
    text=text.replace(target,"""     if(sceneGroup(activeCase)==='operating'){
       const members=Object.fromEntries(Object.keys(slideService.add).map(n=>[n,'frame']));
       variant={...variant,add:{...variant.add,...slideService.add},mass:[...originalMass,slideService.mass],
         cases:variant.cases.map(c=>slideService.currentKeys.includes(c.key)?{...c,rig:{...c.rig,...members}}:c),
         power:[slideService.currentPower,...variant.power],inspection:combine(variant.inspection),running:combine(variant.running),
         status:'Preliminary design · frozen-start campaign in progress',
         validationText:'The complete service package is charged to the chassis mass budget. The free-piece screen covers twelve nominal pickups and shots. The motor can be removed with the cassette locked at its service angle.',
         notes:['Six nominal scores at both simulation resolutions. Acceptance requires 19 of 20 fixed, perturbed starts.',
                'The inspection estimate includes 6 kg design reserve, the cable take-up allowance and the gross slide-motor service package. Grip, loaded tracking and thermal duty remain provisional.']};
     }else"""+target.lstrip())
    a="const service=slideService?.case.key;if(service && (requested===service)!==(next===service))"
    assert text.count(a)==1
    text=text.replace(a,"if(slideService && sceneGroup(activeCase)!==sceneGroup(next))")
    js.write_text(text,encoding='utf8')
print(group,peak,gross['inspectionMarginKg'])

