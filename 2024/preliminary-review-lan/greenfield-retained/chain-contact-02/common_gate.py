"""One spring-loaded yoke closes both sliding keepers. The modeled actuator opens it via an 8 mm pinion."""
from pathlib import Path
import sys
here=Path(__file__).resolve().parent
source=(here/'dynamic.py').read_text(encoding='utf8')
mark='    return src\ndef intercept'
insert="""    # Gate friction and reflected rotor mass include a preliminary backdrivable geared motor.
    import xml.etree.ElementTree as ET
    def configure(text):
        tree=ET.fromstring(text);cassette=tree.find(".//body[@name='cassette']")
        gatesXml=[node for node in cassette.findall('body') if node.find('joint') is not None and node.find('joint').get('name','').startswith('gate')]
        assert len(gatesXml)==2
        yoke=ET.SubElement(cassette,'body',name='linked-keepers',pos='.2035 0 .2405')
        ET.SubElement(yoke,'joint',name='closedYoke',type='slide',axis='0 0 1',range='0 .044',limited='true',stiffness='360',damping='2',springref='-.010',frictionloss='2.0',armature='.13',solreflimit='.002 1')
        ET.SubElement(yoke,'inertial',pos='0 0 .034',mass='.16',diaginertia='.001 .0002 .001')
        for node in gatesXml:
            yy=node.get('pos').split()[1];g=node.find('geom');g.set('pos','0 '+yy+' 0');yoke.append(g);cassette.remove(node)
        # Added/stiffened drive hardware high estimate plus closure package is placed on the moving arm.
        inert=cassette.find('inertial');mass=float(inert.get('mass'));cg0=np.fromstring(inert.get('pos'),sep=' ');inertia=np.fromstring(inert.get('fullinertia'),sep=' ')
        ax=np.diag(inertia[:3]);ax[0,1]=ax[1,0]=inertia[3];ax[0,2]=ax[2,0]=inertia[4];ax[1,2]=ax[2,1]=inertia[5]
        dm=1.45;site=np.array([.42,0,.10]);cg=(mass*cg0+dm*site)/(mass+dm)
        for mm,v in [(mass,cg0-cg),(dm,site-cg)]:ax+=mm*(v@v*np.eye(3)-np.outer(v,v))
        inert.set('mass',str(mass+dm));inert.set('pos',' '.join(map(str,cg)));inert.set('fullinertia',' '.join(map(str,[*np.diag(ax),ax[0,1],ax[0,2],ax[1,2]])))
        return ET.tostring(tree,encoding='unicode')
    globals()['configureCommon']=configure
    src=src.replace("mj.MjModel.from_xml_string(xml)","mj.MjModel.from_xml_string(configureCommon(xml))")
    src=src.replace("gates=[m.joint('gate'+str(i)) for i in range(2)]","gates=[m.joint('closedYoke')]*2")
    src=src.replace("np.clip(5000*(.044-d.qpos[g.qposadr[0]])-5*v+9.72,0,25)","np.clip(8000*(.044-d.qpos[g.qposadr[0]])-9*v+19.44,0,45)")
    src=src.replace("r.update(physicalHookNormalPeakN=", "r.update(totalExplicitMassKg=m.body_mass.sum()-24*.085,commonGate=True,reflectedGateKg=.13,physicalHookNormalPeakN=")
    return src
def intercept"""
assert source.count(mark)==1
source=source.replace(mark,insert)
source=source.replace("src=convert(src);sys.argv", "src=convert(src);env['configureCommon']=globals()['configureCommon'];sys.argv")
exec(compile(source,'shared rack and spring','exec'))
