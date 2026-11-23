from pathlib import Path
import json
q=Path(__file__).parent
v3=(q/'support-v3.py').read_text()
key='fix=fix.replace(\'screen-v2.json\',\'screen-v3.json\')'
extra=r"""
fix=fix.replace('p+n*1.31,n,3.25,1.71','p,n,2.45,3.02')
key="ns={'__name__'"
i=fix.index(key)
fix=fix[:i]+'''source=source.replace("add(label+' stationary POM guide '","if rear:gib=max(entities(gib,TopAbs_SOLID),key=lambda a: GP(a)[0])\\n   add(label+' stationary POM guide '")\n'''+fix[i:]
fix=fix.replace("plate['shape']=built;plate['name']", "built=un([built,box([-125,-15,98],[-82,-2,104])]);built=cut(built,box([-182,29.7,88],[-84.7,34.3,102]));plate['shape']=built;plate['name']")
fix=fix.replace("'roster-v3.json'","'roster-v4.json'").replace("'screen-v2.json'","'screen-v4.json'")
mark="(q/'roster-v4.json').write_text"
fix=fix.replace(mark,'''splits={row['name']:[{'v':w['GP'](so)[0],'bbox':w['bounds'](so)} for so in w['entities'](row['shape'],TopAbs_SOLID)] for row in r if len(list(w['entities'](row['shape'],TopAbs_SOLID)))!=1}
 (q/'splits-v4.json').write_text(json.dumps(splits,indent=1))
 from OCP.STEPControl import STEPControl_Writer,STEPControl_AsIs
 wr=STEPControl_Writer()
 for row in r:
  if ' leaf ' not in row['name']:wr.Transfer(row['shape'],STEPControl_AsIs)
 wr.Write(str(q/'support-only-v4.step'))
 (q/'roster-v4.json').write_text''')
"""
assert v3.count(key)==1;v3=v3.replace(key,extra)
exec(compile(v3,'<v4>','exec'),{'__file__':str(q/'support-v3.py'),'__name__':'__main__','json':json})

