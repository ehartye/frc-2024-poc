"""Screen R8 in its lowered-intake configuration using measured native bounds.

Run with the private R7 physical survey and the FE-2024 installed leaf bounds.
Only derived measurements, counts and analytic load estimates are released.
"""
import argparse
from hashlib import sha256
from itertools import product
import json
import math
from pathlib import Path
import numpy as np
from trajectory import run as trajectory
from chain_path import screen_chain

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
HOOK = np.array([.5,-30.,480.])
CORNERS = np.array(list(product((0,1),repeat=3)))


def load(path):
    return json.loads(path.read_bytes())


def cat(span, high, low):
    half = span / 2
    lo, hi = 100., 10000.
    for _ in range(65):
        a = (lo+hi)/2
        if a*(math.cosh(half/a)-1) > high-low:
            lo = a
        else:
            hi = a
    return a, a*math.sinh(half/a)


def mass(row):
    name = row['body']
    for term,kg in [('motor',.46),('gearbox',.38),('Travelling nut',.08),('Spring return housing',.20),('Thrust bearing',.12)]:
        if term in name:
            return kg
    steel = any(word in name for word in ('pin','fastener','Screw','roof','lip','spine','bracket','Captive nut'))
    return row['volumeMm3']*(7850 if steel else 2700)*1e-9


def bounds(rows, key):
    box = np.array([row[key] for row in rows]).reshape(-1,2,3)
    return box[:,0,None,:] + (box[:,1]-box[:,0])[:,None,:]*CORNERS[None,:,:]


def orientation(cg):
    # Shortest rotation mapping the vector from the mass center to the seat onto world +Z.
    u = HOOK-cg
    u /= np.linalg.norm(u)
    x,y,z = np.cross(u,[0.,0.,1.])
    k = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    rot = np.eye(3)+k+k@k/(1+u[2])
    return rot, math.degrees(math.acos(u[2]))


def study(survey, field, receipt):
    mass_case = load(ROOT/'cad/drivetrain/r7-floor-mass.json')
    if sha256(survey.read_bytes()).hexdigest() != mass_case['surveySha256']:
        raise ValueError('Incorrect R7 physical survey')
    field_data = load(field)
    if field_data['source'] != '42c91b3efb9521d059c13cf25e2d7dda6315bdd6b7eb1202847bfe84ffb43c1a':
        raise ValueError('Incorrect FE-2024 source')
    first = load(survey)['rows']
    new = receipt['climberBodies']
    parts = [(mass(row),np.mean(np.array(row['boundsMm']).reshape(2,3),axis=0)) for row in new]
    parts.append((.60,np.array([58.,-24.,260.])))  # Unmodeled brackets, pads, harness and drive hardware.
    addition = sum(m for m,_ in parts)
    center = sum(m*p for m,p in parts)/addition
    base = mass_case['cases']['nominal']['robot']
    bounds_r7 = load(ROOT/'cad/drivetrain/r7-independent-bounds.json')
    def combined(m,c,scale=1):
        return (m*np.asarray(c)+scale*addition*center)/(m+scale*addition)
    nominal = combined(base['massKg'],np.array(base['centerM'])*1000)
    variants = [combined(m,c,s) for c in product(*(np.array(bounds_r7['centerIntervalsM'])*1000))
                for m in bounds_r7['massIntervalKg'] for s in (.75,1.35)]
    heavy = bounds_r7['massIntervalKg'][1]+1.35*addition
    weight = heavy*9.80665
    model = np.concatenate((bounds(first,'bboxMm'),bounds(new,'boundsMm')))
    stage = [p for p in field_data['parts'] if p['path'][1:2]==['GE-24200_Blue']]
    chain = next(p for p in stage if p['path'][-1]=='GE-24205_hanging' and p['bounds'][0][0]>5.7)
    lo,hi = np.array(chain['bounds'])*1000
    x,y = (lo[:2]+hi[:2])/2
    a,half_length = cat(hi[1]-lo[1],hi[2],lo[2])
    half_span = (hi[1]-lo[1])/2
    # Each AABB remains attached to its native component. Omit the three flexible
    # hanging spans only; their rectangular hulls include large volumes of air.
    fixed_field = [p for p in stage if p['path'][-1]!='GE-24205_hanging']
    field_boxes = np.array([p['bounds'] for p in fixed_field])*1000
    good = (field_boxes[:,1,0]>x-900)&(field_boxes[:,0,0]<x+1000)
    field_boxes = field_boxes[good]
    field_names=[p['path'][-1] for p,take in zip(fixed_field,good) if take]
    body_names=[r['name'] for r in first]+[r['body'] for r in new]

    def locate(rot,side,height):
        return (model-HOOK)@rot.T + [x,y+side,height]

    def field_gap(cloud):
        lower,upper = np.min(cloud,axis=1),np.max(cloud,axis=1)
        best = 1e6
        hits = []
        for name,box in zip(field_names,field_boxes):
            gap = np.maximum(np.maximum(box[0]-upper,lower-box[1]),0)
            dist = np.linalg.norm(gap,axis=1)
            best = min(best,float(min(dist)))
            hits.extend([body_names[i],name] for i in np.flatnonzero(dist<1e-5))
        return best,hits

    # R7's one-piece bumper visualization encloses the hollow robot in its AABB.
    # Physical bumper backing and cover parts from the survey are checked independently.
    physical=[r for r in first if r['name']!='Continuous bumper native display v2']
    r7boxes=np.array([r['bboxMm'] for r in physical]).reshape(-1,2,3)
    internal_hits=[]
    for r in new:
        lower,upper=np.array(r['boundsMm']).reshape(2,3)
        if r['component']=='R8 sliding saddle': upper=upper+[0,0,380]
        if r['component']=='R8 transverse lock': upper=upper+[52,0,0]
        inside=np.all(np.minimum(upper,r7boxes[:,1])-np.maximum(lower,r7boxes[:,0])>.05,axis=1)
        for idx in np.flatnonzero(inside):
            internal_hits.append([r['body'],physical[idx]['name']])
    findings = []
    for offset in (0.,500.):
        rise = math.sqrt((half_length-offset)**2-(half_span-offset)**2)
        height = hi[2]-rise
        unloaded = lo[2]+a*(math.cosh(offset/a)-1)
        rot,tilt=orientation(nominal)
        seats = [-offset,offset] if offset else [0.]
        cloud = [locate(rot,seat,height) for seat in seats]
        field_checks = [field_gap(c) for c in cloud]
        # Include 25 mm lower loaded height, independent adverse R7 mass intervals,
        # and independent robot tilts for the shared-seat clearance bound.
        adverse = []
        for cg in variants:
            mat,angle = orientation(cg)
            points = (model-HOOK)@mat.T
            adverse.append((height-25+points[:,:,2].min(),points[:,:,1].min(),points[:,:,1].max(),angle))
        innery = max(c[2] for c in adverse) - min(c[1] for c in adverse)
        tension = weight*math.hypot(half_span-offset,rise)/rise/(1 if offset else 2)
        horizontal = tension*(half_span-offset)/math.hypot(half_span-offset,rise)
        findings.append(dict(seatOffsetMm=offset,unloadedLowMm=unloaded,loadedSeatMm=height,
            raisedLipClearanceOverChainAllowanceMm=833-unloaded-27,nominalTiltDeg=tilt,
            nominalFloorClearMm=float(min(c[:,:,2].min() for c in cloud)),adverseFloorClearMm=min(c[0] for c in adverse),
            adverseTiltDeg=max(c[3] for c in adverse),minFixedFieldBoxGapMm=min(c[0] for c in field_checks),
            possibleFixedFieldPairs=[pair for _,pairs in field_checks for pair in pairs],
            nominalPartnerBoxGapMm=float(cloud[1][:,:,1].min()-cloud[0][:,:,1].max()) if offset else None,
            adversePartnerBoxGapMm=2*offset-innery if offset else None,
            outerChainTensionN=tension,innerChainTensionN=horizontal if offset else None,
            linkTensionDifferenceN=tension-horizontal if offset else None))
    # Crossmember conservative simple span, central design point force.
    F = 2*weight
    inertia = (50*38.35**3-44*32.35**3)/12
    return dict(schema='stage-static-screen/1',nativeSha256=receipt['sha256'],r7SurveySha256=mass_case['surveySha256'],
        fieldStepSha256=field_data['source'],nativePose='R7 intake lowered, R8 slide and pin at zero, single bearing roof',
        internalSweepBoxOverlaps=internal_hits,chainFit=dict(halfSpanMm=half_span,halfLengthMm=half_length,anchorApproxMm=hi[2],catenaryAmm=a),
        addedNominalKg=addition,addedCenterMm=center.tolist(),newNominalKg=base['massKg']+addition,
        newNominalCenterMm=nominal.tolist(),upperMassKg=heavy,stageChecks=findings,
        loads=dict(upperStaticN=weight,designForceN=F,designFactor=2,
            pinDoubleShearMPa=F/(2*math.pi*5**2),innerTubeBearingMPa=F/(2*2.5*10),
            beamBendingMPa=F*658/4*(38.35/2)/inertia,roofBendingMPa=6*F*24/(34*13**2),
            screwTorqueNmAt4mmLead30pctEfficiency=weight*.004/(2*math.pi*.30)),
        limitations=['Mast and chain use quasi-static geometry. No dynamic capture, swing, unequal partner loading or sequential load-sharing proof.',
        'Disabled support requires positive lock engagement, rail attachment, single-link seating and adequate strength. Hardware selection, proof loading and full passive hold are open.',
        'Installed field screening uses fixed leaf boxes at the nominal terminal poses. Approach transients, yaw drift and separate flexible spans need further testing.'])


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('r7_survey',type=Path)
    parser.add_argument('installed_field',type=Path)
    args=parser.parse_args()
    result=study(args.r7_survey,args.installed_field,load(ROOT/'cad/student/r8-reopened.json'))
    selected=load(args.r7_survey)['rows']
    trace=trajectory(result,bounds(selected,'bboxMm'))
    result['soloChainPath']=screen_chain(selected,load(ROOT/'cad/student/r8-reopened.json')['climberBodies'],result,trace)
    (HERE/'solo-trajectory.json').write_text(json.dumps(trace,indent=2,allow_nan=False)+'\n',encoding='utf8',newline='\n')
    (HERE/'static-screen.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf8',newline='\n')
