from pathlib import Path
import json,runpy,hashlib,sys
q=Path(__file__).parent
fix=(q/'revised.py').read_text()
start=fix.index("ns={'__name__'")
extra=r"""
source=source.replace('5/math.sqrt(3)','4.88/math.sqrt(3)')
source=source.replace("for name,s in bearing.items():add(k+' bearing '+str(sign)+' '+name,xf(s,rr,base),'steel')","for name,s in bearing.items():\n    yaw=math.radians(-21.5);idx=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]]) if name=='REV-41-1527' else np.eye(3);add(k+' bearing '+str(sign)+' '+name,xf(s,rr@idx,base),'6061' if '1683-1' in name else 'steel')")
source=source.replace('3,6)\n  # Standoffs','3,10 if rear else 6)\n  # Standoffs')
source=source.replace("cyl(p+n*.25,n,3.25,2.77)","cyl(p+n*1.31,n,3.25,1.71)").replace('cyl(p-n*.25,n,1.7,7)','cyl(p-n*.25,n,1.7,13)')
source=source.replace('box([-177,-16,zh-3],[-169,-13,zh+16]),box([-177,-16,zh+10],[-169,2,zh+16]),box([-177,-2,82],[-169,2,zh+16]),box([-177,-3,82],[-72,3,87])','box([-146,-15.7,zh-3],[-138,-13.7,zh+16]),box([-146,-15.7,zh+10],[-138,34,zh+16]),box([-146,30,82],[-138,34,zh+16]),box([-146,28,82],[-72,34,87]),box([-79,-3,82],[-72,34,87])')
source=source.replace('gib=piece(u-5,u+5,-7,7,1.6,10)','gib=piece(u-5,u+5,-7,7,1.6,14 if rear else 10)').replace('piece(u-6,u+6,-4.7,4.7,2.8,6.2)','piece(u-6,u+6,-4.7,4.7,2.8,10.2 if rear else 6.2)').replace("add(label+' stationary POM guide '","if rear:gib=cut(gib,piece(u-6,u+6,4.4,8,3.05,5.55))\n   add(label+' stationary POM guide '")
source=source.replace('101-zh,7,10,13','101-zh,7,14 if rear else 10,17 if rear else 13').replace('[-7.92,-10.93,75]','[-7.92,-10.93,62]').replace('center[1]-7,78.25','center[1]-7,65.25 if label=="rear" else 78.25').replace('site[:2],78]','site[:2],65 if label=="rear" else 78]').replace('1.1,24','1.1,38')
"""
fix=fix[:start]+extra+fix[start:]
fix=fix.replace('[-109,-9,100],[-86,-6,104]','[-109,-6,100],[-86,-2,104]').replace('[-109,-9,101],[-97,-6,227]','[-109,-6,101],[-97,-2,227]').replace('[-109,-9,224],[-82,-6,228]','[-109,-6,224],[-82,-2,228]')
fix=fix.replace("return r,old","""from OCP.TopAbs import TopAbs_SOLID
 volumes=[]
 for row in r:
  v,c=w['GP'](row['shape']);m=row['m'] if row['m'] is not None else v*{'6061':.00270,'POM':.00141,'PA6':.00113,'steel':.00785}[row['material']]
  volumes.append(dict(name=row['name'],j=row['j'],cgMm=c.tolist(),massG=m,bbox=w['bounds'](row['shape']),solids=len(list(w['entities'](row['shape'],TopAbs_SOLID))),valid=w['BRepCheck_Analyzer'](row['shape']).IsValid()))
 (q/'roster-v3.json').write_text(json.dumps(volumes,indent=1))
 return r,old""")
fix=fix.replace('screen-v2.json','screen-v3.json')
exec(compile(fix,'<support-03>','exec'),dict(__name__='__main__',__file__=str(q/'revised.py'),json=json))


