from pathlib import Path
p=Path(__file__).resolve().parents[2]/'www/review.js'
s=p.read_text(encoding='utf8')
old=" const conceptKey=query.get('concept')||(!requested||data.concepts?.field?.cases.some(c=>c.key===requested)?'field':null),variant=data.concepts?.[conceptKey];"
new=""" const guided=data.concepts?.guided,field=data.concepts?.field;
 const conceptKey=query.get('concept')||(!requested||guided?.inherit.includes(requested)||guided?.cases.some(c=>c.key===requested)?'guided':field?.cases.some(c=>c.key===requested)?'field':null);
 let variant=data.concepts?.[conceptKey];
 if(conceptKey==='guided' && guided && field){
   const carry=Object.fromEntries(Object.keys(guided.add).map(n=>[n,'arm']));
   const inherited=guided.inherit.map(key=>{
     const c=field.cases.find(item=>item.key===key);
     const scope=key==='field-park'?' Added hoist stock clears the PARK approach.':' Earlier free-piece trial. Hoist stock was checked against the recorded piece path; updated dynamic scoring remains open.';
     return {...c,rig:{...c.rig,...carry},basis:c.basis+scope};
   });
   variant={...field,...guided,add:{...field.add,...guided.add},cases:[...inherited,...guided.cases]};
 }"""
if old in s:s=s.replace(old,new)
else:assert new in s
tmp=p.with_suffix('.guided-next');tmp.write_text(s,encoding='utf8');tmp.replace(p)
