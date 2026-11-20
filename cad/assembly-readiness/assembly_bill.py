"""Reconcile pre-export engineering records with independently reopened native body properties."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import re

from article_ids import HERE, identify

ROOT=HERE.parents[1]
CONFIG='R9 parked robot, red bumper, installed battery'
GRADE={
    'Aluminum 6061':('6061 structural stock, T6 before welding',[2650,2750]),
    'Aluminum 7075':('REV-21-1135 black anodized 7075 rounded hex stock',[2770,2850]),
    'Steel ASTM A36':('ASTM A36 plate prior to welding',[7750,7950]),
    'Plywood, Finish':('Commercial nominal 3/4 in plywood; veneer species and lot unresolved',[450,750]),
    'Rubber, Nitrile':('NBR battery insulation pad; compound and lot unresolved',[1000,1400]),
    'TIVAR 1000 natural UHMW-PE (mass model)':('TIVAR 1000 virgin natural UHMW-PE, nominal 930 kg/m3',[920,950]),
    'PE closed-cell foam, AndyMark am-5580 (nominal 2 lb per ft3)':('am-5580 closed-cell PE, approximately 2 lb/ft3',[24,42]),
}
AUTH='https://firstfrc.blob.core.windows.net/frc2024/Manual/HTML/2024GameManual.htm#R103'


def write(path,value):
    path.write_text(json.dumps(value,indent=1,allow_nan=False)+'\n',encoding='utf8',newline='\n')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build(archive,raw_file,planning_file,out):
    out.mkdir(parents=True,exist_ok=True)
    digest_value=digest(archive)
    raw=json.loads(raw_file.read_bytes())
    if raw['archiveSha256']!=digest_value:raise ValueError('Native receipt does not identify archive')
    electrical={c['id']:c for c in json.loads((ROOT/'cad/electrical/manifest.json').read_bytes())['components']}
    planrows=json.loads(planning_file.read_bytes())
    observed={(r['occurrence'],r['body']):r for r in raw['rows']}
    bases={}; catalog={}; specs={}; ranges={}; grouped=defaultdict(list)
    plan=[];observation=[];names=[];assignments=[];owners={};stock_inputs=defaultdict(list)
    excluded=[];descriptions={};provisional=defaultdict(list);selected_skus={}
    def purchase(sku,kg,url):
        if sku not in catalog:
            catalog[sku]=dict(sku=sku,unit='kg per installed unit',rangeKg=[kg,kg],source=url)
        return sku
    for row in planrows:
        occ,body=row['occurrence'],row['body']
        actual=observed.pop((occ,body));comp=actual['component']
        identity,owner,name=identify(comp,body)
        if (row['identity'],row['instance'])!=(identity,owner) or (actual['identity'],actual['instance'])!=(identity,owner):
            raise ValueError('Native ownership mismatch '+body)
        if abs(row['designVolumeCm3']-actual['volumeCm3'])>max(.025,actual['volumeCm3']*.0001):
            raise ValueError('Reopened volume disagreement '+body)
        measurement={k:actual[k] for k in ('occurrence','body','identity','instance','materialId','volumeCm3','massKg')}
        slot=dict(occurrence=occ,body=body,identity=identity)
        token=occ+' / '+body
        names.append(token)
        role='article-proxy';sku=None;stock=None
        if owner is None:
            slot.update(role='display',reason='Applied team numeral; cloth/marking mass needs weighing')
            observation.append(measurement);plan.append(slot)
            assignments.append(dict(export='robot',occurrence=token,role='external'))
            continue
        if owner.startswith('maxswerve-'):
            sku=purchase('REV-21-3005 with REV-21-1650 and REV-21-1651',1.720,'https://www.revrobotics.com/rev-21-3005/')
        elif owner.startswith('electrical-') and name in electrical:
            c=electrical[name]
            per_unit=c['massKg']
            if per_unit is not None:
                part=c['partNumber'] if name!='main-breaker' else 'CB185-120'
                sku=purchase(part,per_unit,c['sourceUrl'])
        elif ' neo ' in (' '+name.lower()+' ') and '550' not in name:
            sku=purchase('REV-21-1650 V1.1',.425,'https://www.revrobotics.com/rev-21-1650/')
        elif body.startswith('REV-21-2120 '):
            sku=purchase('REV-21-2120 assembly and installed REV-21-2122 adapter',.3275,'https://www.revrobotics.com/rev-21-2120/')
        sel=row['selection']
        idx=int(body.split()[1]) if body.startswith('R7 ') else -1
        selected=None
        if 'cover staple' in body: selected='Arrow T50 506'
        if body.startswith('MISUMI CB8-100'): selected='MISUMI CB8-100'
        if body.startswith('Accu HPN-M8-C8-Z'): selected='Accu HPN-M8-C8-Z'
        if body.startswith('JTD FW0010-375031-SS'): selected='JTD FW0010-375031-SS'
        if body.startswith('REV-21-1916 '): selected='REV-21-1916-PK4'
        if body.startswith('REV-21-2016 '): selected='REV-21-2016'
        if idx in (593,598,604,606,608,610): selected='MISUMI CB6-25'
        if idx in (600,602): selected='MISUMI CB6-20'
        for text,article in [('Panduit LCAX6-14-L','Panduit LCAX6-14-L'),('Pacer TBCA2-R','Pacer TBCA2-R'),('Southwire 104110604','Southwire 104110604 by cut length')]:
            if text in body:selected=article
        if selected: selected_skus[owner]=selected
        if sku:
            basis='supplier'
        elif sel in GRADE and not any(s in name.lower() for s in ('envelope','reservation','representation','staple','washer','screw','bolt','nut','socket head','locknut')) and not owner.startswith('maxswerve-'):
            basis='stock';role='stock';stock=sel
            grade,bounds=GRADE[sel]
            specs.setdefault(stock,dict(id=stock,grade=grade,materialIds=[],densityKgM3=bounds))
            if row['materialId'] not in specs[stock]['materialIds']:specs[stock]['materialIds'].append(row['materialId'])
            stock_inputs[owner].append([row['designVolumeCm3']*v/1e6 for v in bounds])
            measurement['stock']=stock;slot['stock']=stock
        elif (sel=='Steel, Alloy' or selected=='Arrow T50 506' or body.startswith('JTD ')) and not any(
                t in name.lower() for t in ('bearing reservation','sprocket reservation','housing envelope','motor envelope','gearbox envelope')):
            # Solid hardware dimensions omit threads, locknut nylon and some wrench recesses.
            # This provisional engineering allowance cannot qualify inspection weight.
            basis='allowance'
            provisional[owner].append([row['designVolumeCm3']*v/1e6 for v in (6200,8050)])
        else:
            basis='unknown'
        if owner in bases and bases[owner]!=(basis,sku):
            raise ValueError('Mixed physical article '+owner)
        bases[owner]=(basis,sku);descriptions.setdefault(owner,name)
        grouped[owner].append(body)
        index=int(body.split()[1]) if body.startswith('R7 ') else -1
        exempt=(name=='battery' or comp in ('R9 bumper cover envelope','R9 removable bumper cores','R9 bumper washers')
                or index in (593,594,*range(598,612)))
        if exempt and owner not in excluded: excluded.append(owner)
        slot.update(role=role,instance=owner)
        observation.append(measurement);plan.append(slot)
        assignments.append(dict(export='robot',occurrence=token,
             role='installed' if len(grouped[owner])==1 else 'included',instance=owner))
    if observed:raise ValueError('Engineering plan misses native rows')
    loose={'wiring-and-connectors':'Harnesses, lugs, retainers and battery disconnect',
           'additional-controllers':'Four independent controllers for existing motor gaps, plus climber control pending selection',
           'welds-and-finishes':'Added weld metal, coating and shop substitutions',
           'lift-input-interface':'MAXPlanetary input hardware, adapter mounting and final spacers',
           'climber-load-hardware':'Transmission, lock spring, purchased bearings, loaded fasteners and retention'}
    for owner,desc in loose.items():bases[owner]=('unknown',None);descriptions[owner]=desc
    evidence=dict(occurrences=names,sku=catalog,stocks=specs,designRangesKg=ranges,
                  exclusion=dict(criterion='2024 R103 A/B removable bumpers and battery',instances=excluded,source=AUTH),
                  limitKg=125.5*.45359237,authority=AUTH)
    instances=[];engineering=[]
    for owner,(basis,sku) in sorted(bases.items()):
        item=dict(id=owner,disposition='buy' if (sku or owner in selected_skus) else 'make' if basis=='stock' else 'undecided',
                  presence='unmodeled' if owner in loose else 'modeled',mass=None)
        ref=dict(id=owner,basis=basis,inspection='counted')
        if owner in excluded:
            ref.update(inspection='excluded',reason='R103 removable bumper hardware or battery',
                       inspectionEvidence=dict(source='evidence',pointer='/exclusion'))
        if sku:
            item['article']=sku
            pointer='/sku/'+sku.replace('~','~0').replace('/','~1')
            ref['supplier']=dict(source='evidence',pointer=pointer)
            item['mass']=dict(source='evidence',pointer=pointer+'/rangeKg',basis='estimated',rangeKg=catalog[sku]['rangeKg'])
        elif basis in ('stock','allowance'):
            entries=stock_inputs[owner] if basis=='stock' else provisional[owner]
            ranges[owner]=[sum(v[i] for v in entries) for i in range(2)]
            item['mass']=dict(source='evidence',pointer='/designRangesKg/'+owner.replace('/','~1'),basis='estimated',rangeKg=ranges[owner])
        if owner in selected_skus:item['article']=selected_skus[owner]
        instances.append(item);engineering.append(ref)
    planfile=dict(schema='agent-frc/engineering-material-plan@1',configuration=CONFIG,archiveSha256=digest_value,
                  bodies=plan,instances=engineering,stocks=[dict(id=k,source='evidence',pointer='/stocks/'+k.replace('/','~1')) for k in specs])
    actualfile=dict(schema='agent-frc/native-physical-properties@1',configuration=CONFIG,archiveSha256=digest_value,opened=True,
                    nativeBodyCount=len(observation),rawBodyTotalKg=sum(r['massKg'] for r in observation),bodies=observation)
    for file,value in [('plan',planfile),('actual',actualfile),('evidence',evidence)]:write(out/(file+'.json'),value)
    import os
    pins=[dict(id='archive',file=Path(os.path.relpath(archive,out)).as_posix(),sha256=digest_value)]
    pins.extend(dict(id=k,file=k+'.json',sha256=digest(out/(k+'.json'))) for k in ['plan','actual','evidence'])
    mapping=dict(schema='agent-frc/assembly-accounting@1',configuration=CONFIG,revision=digest_value,scope='partial',
                 omissions=list(loose.values()),sources=pins,exports=[dict(id='robot',source='evidence',pointer='/occurrences')],
                 articles=[dict(id=k,sku=k) for k in sorted(set(catalog)|set(selected_skus.values()))],assignments=assignments,instances=instances,changes=[],selected=[],
                 mass=dict(configuration=CONFIG,revision=digest_value,unit='kg',limitKg=evidence['limitKg'],
                           limitEvidence=dict(source='evidence',pointer='/limitKg'),baseline=None,omissions=list(loose.values())),
                 nativeMass=dict(archive='archive',plan='plan',observations='actual'))
    write(out/'mapping.json',mapping)
    bill=[dict(id=k,description=descriptions[k],basis=v[0],sku=v[1] or selected_skus.get(k),bodyCount=len(grouped[k])) for k,v in sorted(bases.items())]
    write(out/'articles.json',bill)
    reviewed=[dict(body=r['body'],lowVolumeCm3=r['lowProxyVolumeCm3'],veryHighVolumeCm3=r['nativeVeryHighVolumeCm3'],
                   registeredVolumeCm3=r['volumeCm3'],veryHighKg=r['nativeVeryHighKg'],registeredKg=r['massKg'],
                   physicalDensityKgM3=r['densityKgM3']) for r in raw['rows']
              if r['massMethod']=='registered solid volume times physical material density']
    write(out/'native-property-check.json',dict(archiveSha256=digest_value,cachedAssemblyKg=raw['totalNativeKg'],
            registeredBodyKg=raw['registeredBodyKg'],replacedCachedValues=reviewed,
            method='Registered solid copy in robot coordinates times applied physical density where the native very-high properties disagree; all volumes checked against pre-export body geometry.'))
    return bill


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('archive','native','design','output'):parser.add_argument(key,type=Path)
    a=parser.parse_args();build(a.archive,a.native,a.design,a.output)
