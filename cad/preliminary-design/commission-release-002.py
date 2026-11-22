"""Commission a representative, source-bound mechanical feed, discharge dwell and service operation on review 002."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]


def read(path):
    return json.loads(path.read_bytes())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def array(pose):
    return np.vstack((np.array(pose).reshape(3, 4), [0, 0, 0, 1]))


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("plugin", type=Path)
    args.add_argument("step", type=Path)
    args.add_argument("output", type=Path)
    args.add_argument("power", nargs="?", type=Path,
                      default=HERE / "motion-evidence-002/gross-power.json",
                      help="Mounted slide power and gross inspection-mass ledger")
    opt = args.parse_args()
    sys.path.insert(0, str(opt.plugin.resolve() / "scripts"))
    from mechanical_motion import load_solids, transforms
    from OCP.gp import gp_Trsf

    root = opt.output.resolve()
    root.mkdir(exist_ok=True, parents=True)
    release = PROJECT / "docs/preliminary-design/releases/002"
    published = read(release / "release.json")
    assert sha(opt.step) == published["geometry"] and sha(release / "scene.json.gz") == published["sceneSha256"]
    scene = json.loads(gzip.decompress((release / "scene.json.gz").read_bytes()))
    guided = scene["concepts"]["guided"]
    replay = next(c for c in guided["cases"] if c["key"] == "continuous-cycle-96")
    issue = "2024-002-cassette"
    def pin(file):
        return dict(file=file.relative_to(root).as_posix(), sha256=sha(file))
    def save(name, value):
        path = root / name
        path.write_bytes((json.dumps(value, indent=2, allow_nan=False) + "\n").encode())
        return pin(path)
    def copy(source, name):
        dest = root / name
        if not dest.exists() or sha(dest) != sha(source):
            shutil.copyfile(source, dest)
        return pin(dest)

    step = copy(opt.step, "robot.step")
    narrative = copy(PROJECT / "kickoff/strategic-narrative.md", "strategic-narrative.md")
    roster = load_solids(opt.step)[0]
    ids = {n.replace("\u00c2\u00b7", "\u00b7"): n for n in roster}
    field = scene["concepts"]["field"]
    solids = {n: s for n, s in scene["bodies"].items() if n not in field["remove"]}
    solids.update(field["add"])
    solids.update(guided["add"])
    limits = read(HERE / "compression-limits-002.json")
    rules = [dict(payload="Chambered NOTE",contact=ids[n],maxDepthMm=v,basis=limits["basis"]) for n,v in limits["limitsMm"].items()]
    pos = replay["positions"][0]
    route = [dict(id="start",x=pos[0]/1000,y=pos[1]/1000,headingDeg=-115,action="none")]
    pm = read(PROJECT / "kickoff/physical-match.json")
    size = pm["field"]
    strategy = dict(activeNarrativeId="score-climb", narratives=[dict(id="score-climb",title="Recover, score, climb",
        status="exploring",narrative="Acquire finite floor NOTES, deliver to the speaker, then park or climb.")],
        requirements=[dict(id="delivery",narrativeIds=["score-climb"],outcome="Move the retained NOTE out of the cassette",
        screen="Registered free-piece trajectory, launch exit and paired field detections.",status="candidate")],decisions=[])
    form = dict(schema="agent-frc/design-artifact@2",id="crescendo-cassette",revision=2,parentRevision=1,units="m",
        workspace=dict(lengthM=size["length"]*.0254,widthM=size["width"]*.0254,pieceCenterM=[pos[0]/1000,pos[1]/1000,.0254],
            pieceSizeM=[.3556,.3556,.0508],targetSizeM=[.45,1.05,.32],basis="Kickoff field dimensions; reference target box only. Exact field mesh and target are used in the scoring simulation."),
        rules=[],frames=dict(world=dict(axes="x downfield, y across field, z up",basis="2024 blue field frame"),
            robot=dict(x=pos[0]/1000,y=pos[1]/1000,z=0,headingDeg=-115,basis="First saved robot pose in continuous-cycle-96"),
            target=dict(x=0,y=5.55,z=2.0,headingDeg=0,basis="Approximate speaker reference. Official opening geometry governs the field trial.")),
        strategy=strategy,design=dict(parameters=dict(frameLengthM=.680,frameWidthM=.660,trackM=.510,bumperBottomM=.052,
            bumperHeightM=.124,bumperDepthM=.082,height=1.040315,reach=0,toolHeight=.821,opening=.380,phase="start"),
            requirementLinks={"body-envelope":["delivery"],"tool":["delivery"]}),operations=[dict(id="floor-opening",type="intake-opening",enabled=False,targetIds=["frame-plate","body-envelope"],requirementIds=["delivery"],unresolvedChecks=["Verify floor clearance and roller pickup with the solid front perimeter."])],
        rehearsal=dict(route=route,driveSpeedMps=1,turnSpeedDegps=60,actionDurations=dict(pickup=2.84,deliver=.81),
            opponent=dict(enabled=False,footprintM=.84,periodS=1,x=0,yMin=0,yMax=1),basis="estimated"),
        sources=[],disciplinePlans=dict(programming=dict(status="pseudocode",states=[],pseudocode="Confirm retention, aim and feed. Observe exit, clear the cassette and reset."),
        fabrication=dict(materials=[],processes=[],toolCapabilities=[],openQuestions=["Measure NOTE load versus indentation and tangential force under slip."])),
        constraints=[],checks=[],provenance=dict(basis="Published revision 002. The editable envelope is secondary to the registered solid model and published trajectories.",
        limitations=["Reference target and route speeds in the envelope editor are approximate. Use the retained field trajectory for scoring."]))
    design = save("design.json", form)
    datum = dict(name="Parked robot frame, mm",robotToStepMm=np.eye(4).flatten().tolist())
    ingredient = {}
    components = {}
    groups = {}
    def group_for(n,b):
        if n == "Chambered NOTE": return "NOTE", "payload"
        if n == "Battery service pin": return "Removable service pin","mechanism"
        if n == "Blue bumper set": return "Bumpers","bumper"
        if n == "Battery and terminal case": return "Battery","battery"
        if "swerve" in b["group"].lower(): return b["group"],"drive"
        if b["group"] == "Chassis": return "Chassis","structure"
        return b["group"],"mechanism" if b["category"] in ("mechanism","structure") else "other"
    for n,raw in ids.items():
        b=solids[n]
        group,role=group_for(n,b)
        # Functional assemblies may include their own electrical and structural articles.
        if group in groups:role=groups[group]["role"]
        groups.setdefault(group,dict(role=role,members=[]))["members"].append(raw)
        ingredient[n]=group
        components[raw]=dict(name=n,category=b["category"],material=b["material"],subsystem=b["group"],
            purpose=b["purpose"],rationale=b["rationale"],volumeType="envelope" if b["category"]=="reserved-space" else "material")
    masses=[];assigned=set()
    for row in guided["mass"]:
        members=[ids[n] for n in row["members"] if n in ids and n not in assigned]
        assigned.update(n for n in row["members"] if n in ids)
        low,high=row["kg"]
        if low<0 and high==0:
            # Earlier row owns the full belt package. Apply its net machining credit once at the low end.
            parent=next(r for r in masses if r["id"]=="Split transmission, net added stock and electrical allowance")
            parent["kg"][0]+=low
            parent["basis"]+=" Includes the source ledger's lower-bound backing material credit."
            continue
        masses.append(dict(id=row["id"],members=members,kg=[low,high],basis="Published revision 002 subtotal, preserving additions and incremental stock allowances."))
    for row in guided["details"]:
        if row["name"]=="one NOTE": continue
        members=[ids[n] for n in row["members"] if n in ids and n not in assigned]
        assigned.update(n for n in row["members"] if n in ids)
        if members:masses.append(dict(id=row["name"],members=members,kg=[0,0],
            basis="Excluded from the inspection scale; included at operating weight in the payload ledger."))
    gross=read(opt.power)
    reserve=scene["slideService"]["mass"]["kg"]
    masses.append(dict(id="Slide motor service package, gross allowance",members=[],kg=reserve,basis=scene["slideService"]["budgetBasis"]))
    inspection=[0., 0.]
    for row in masses:
        inspection=[a+b for a,b in zip(inspection,row["kg"])]
    assert abs(inspection[1]-gross["inspectionHighKg"])<1e-7,inspection
    operating=[sum(r["kg"][i] for r in guided["details"]) for i in (0,1)]
    assert set(ids)-assigned=={"Chambered NOTE"},set(ids)-assigned
    running=[a+b for a,b in zip(inspection,operating)]
    def rotate(point,angle):
        return dict(kind="rotation",axis=[0,1,0],originMm=point,travelDeg=angle)
    def xyz(v): return dict(kind="translation",travelMm=list(map(float,v)))
    def relative(i,group):
        return np.linalg.solve(array(replay["series"]["frame"][i]),array(replay["series"][group][i]))
    def rotation(m):
        t=gp_Trsf();t.SetValues(*m[:3].flatten());q=t.GetRotation()
        vec=np.array([q.X(),q.Y(),q.Z()]);length=np.linalg.norm(vec)
        return (vec/length if length>1e-12 else np.array([0.,1.,0.])),2*np.degrees(np.arctan2(length,q.W()))
    center=np.array([20.39580315236,0.,648.20943713004,1.])
    def change(start,end):
        axis,angle=rotation(end@np.linalg.inv(start))
        ops=[]
        if abs(angle)>1e-7:ops.append(dict(kind="rotation",axis=axis.tolist(),originMm=(start@center)[:3].tolist(),travelDeg=float(angle)))
        shift=((end-start)@center)[:3]
        if np.linalg.norm(shift)>1e-6:ops.append(xyz(shift))
        return ops
    first,last=284,291
    # The robot holds the fixed launch angle for this full feed, departure and empty dwell.
    arm_y=np.degrees(np.arctan2(relative(first,"arm")[0,2],relative(first,"arm")[0,0]))
    flap_y=np.degrees(np.arctan2(relative(first,"flap")[0,2],relative(first,"flap")[0,0]))-arm_y
    arm_ops=[rotate([185.,0.,501.],float(round(arm_y,6)))]
    flap_ops=[rotate([-156.0498158823,0.,820.69872237251],float(flap_y)),*arm_ops]
    stock_ops={}
    for n,raw in ids.items():
        rig=replay["rig"][n]
        if rig in ("flap","arm"):stock_ops[raw]=flap_ops if rig=="flap" else arm_ops
        if rig!="piece":
            for i in range(first,last+1):
                # Captured matrices are rounded display numbers. Compare the reconstructed poses on actual corners.
                tf=gp_Trsf()
                for operation in transforms(stock_ops.get(raw,[]),1):tf.PreMultiply(operation)
                m=np.eye(4);m[:3]=[[tf.Value(r,c) for c in range(1,5)] for r in range(1,4)]
                vertices=np.array(solids[n]["v"]);delta=m-relative(i,rig)
                err=np.max(np.linalg.norm(vertices@delta[:3,:3].T+delta[:3,3],axis=1))
                assert err<.020,(n,i,err)
    groups[ingredient["Feed tread · left"]]["role"]="mechanism"
    source=save("replay-source.json",dict(sceneSha256=published["sceneSha256"],case=replay["key"],frames=[first,last],
        sourceTimes=replay["time"][first:last+1],scope="Finishing feed, free discharge and empty launch dwell on the original blue wing middle delivery."))
    cases=[];stages={};cycle=[]
    obj="Discharge the retained NOTE from the cassette"
    kin=change(np.eye(4),relative(first,"piece"))
    for i in range(first,last):
        key=f"feed-{i}" if i<290 else "empty-launch-dwell"
        stage="operating" if i<290 else "park"
        moves={"Chambered NOTE":change(relative(i,"piece"),relative(i+1,"piece"))}
        assert moves["Chambered NOTE"]
        # Accumulate the ordered motion so the end and following start share the identical kinematic construction.
        initial=dict(stock_ops);initial["Chambered NOTE"]=kin[:]
        c=dict(id=key,stage=stage,basis=f"{replay['key']} frames {i}–{i+1}, {replay['time'][i]:.4f}–{replay['time'][i+1]:.4f} s. Independent free NOTE, launch station fixed.",
            initialPose=initial,moves=moves,minGapMm=0,maxDepth=14 if i==289 else 12,payloadCompression=rules)
        cases.append(c);cycle.append(key)
        stages[key]=dict(purpose=stage,allowanceBasis="Declared, surface-specific NOTE indentation. All other contacts retain rigid checks.",movingGroups=["NOTE"])
        if stage=="operating" and i<289:
            drive=ingredient["Feed tread · left"]
            trial=save(key+"-transport.json",dict(source=source,frame=i,seconds=replay["time"][i:i+2],
                transport=dict(case=key,payload="NOTE",mechanisms=[drive],moves=moves,initialPose={"Chambered NOTE":kin[:]})))
            stages[key]["transportedBy"]={"NOTE":dict(mechanisms=[drive],drive={drive:"Driven tread at the launch station during feed."},
                basis="Published free-body transport, same centers and section orientation.",evidence=trial)}
        kin=kin+moves["Chambered NOTE"]
    service="withdraw-service-pin"
    cases.append(dict(id=service,stage="service",basis="Withdraw the lateral battery-service index pin from its guide, in the source parked configuration.",
        moves={ids["Battery service pin"]:[xyz([0,225,0])]},extract=[ids["Battery service pin"]],minGapMm=0,maxDepth=10,
        checkLinearEnvelope=True,payloadCompression=rules))
    stages[service]=dict(purpose="service",allowanceBasis="Full straight withdrawal past the installed robot envelope.",movingGroups=["Removable service pin"])
    motion=save("motion.json",dict(schema="agent-frc/mechanical-motion@1",layoutDesignSha256=design["sha256"],layoutDatum=datum,step=step,cases=cases))
    # Use the grounded source budgets, including the gross service package and a weak pack.
    duty={}; electrical={}
    shot=guided["power"][0]
    for stage in cycle:
        for condition in ("nominal","weak"):
            key=stage+"-"+condition
            loads={line["name"]:line["amps"] for line in shot["lines"]}
            # The weak-pack current envelope also bounds the empty dwell in this commissioned slice.
            nominal=next(r for r in scene["power"] if r["batteryCondition"]=="nominal")
            pack=shot if condition=="weak" else nominal
            duty[key]=dict(stage=stage,duty="Feed and launch currents with concurrent accessories",batteryCondition=condition,
                requiredMotionMinV=shot["requiredMinV"],sources=list(loads))
            electrical[key]=dict(loadedKg=running,batterySideA=loads,openCircuitV=[pack["openCircuitV"]]*2,
                resistanceOhm=[pack["resistance"]]*2,basis="Same source-bounded battery-side load envelope for the feed and immediate post-release dwell.")
    limits_pin=save("limits.json",dict(basis="Kickoff inspection allocation and selected 8 V minimum loaded bus.",inspectionLimitKg=scene["limits"]["inspectionKg"],
        operatingMinV=8,massLines=[r["id"] for r in masses],electricalCases=duty))
    budget=save("budgets.json",dict(issue=issue,step=step,motion=motion,design=design,masses=masses,runningPayloadKg=operating,requirements=limits_pin,electricalCases=electrical))
    limit_source=save("issued-budget.json",gross)
    hold=dict(id="inspection-mass",unit="kg",scenario="Upper inspection estimate including slide service stock")
    accepted=save("accepted.json",dict(**hold,basis="Kickoff inspection limit applied to the current gross upper allocation.",phase="preliminary-design",criterion=dict(range=[0,scene["limits"]["inspectionKg"]]),
      interfaces=["Chassis"],source=limits_pin))
    current=save("observed.json",dict(**hold,valueField="inspectionHighKg",value=gross["inspectionHighKg"],source=limit_source,
        issue=issue,designSha256=design["sha256"],stepSha256=step["sha256"]))
    # Pick a physical interface to an adjacent group for each mechanism. Store the
    # point on the host in the source STEP frame. The removable pin has running clearance.
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    interfaces = {
        "Pivot support": ("Pivot A-frame · left", "Chassis sill · left"),
        "Pivoting cassette": ("Cassette cheek · left", "Pivot torque shaft"),
        "Piece support": ("Runner crossbar 125", "Cassette side beam · left"),
        "Guided chain hoist": ("Pivot torque shaft", "Pivot bearing · left"),
        "Cassette drive": ("Nose roller shaft", "Entry bearing bracket · left"),
        "Electrical bay": ("Electronics deck", "Bay crossmember · rear"),
        "Removable service pin": ("Battery service pin", "Service pin guide"),
        "Passive cover": ("Lid post 95 · left", "Cassette side beam · left"),
        "Shared cassette lift": ("Climb yoke post · left", "Cassette side beam · left"),
        "Pickup lip": ("Cassette side beam extension · left", "Cassette side beam · left"),
        "Climb catch": ("Catch sector screw 1", "Catch sector"),
        "Carpet followers": ("Follower shoulder axle · left", "Cassette side beam extension · left"),
        "Metering drive": ("Feed reducer mount", "Runner crossbar 125"),
        "Launch drive": ("Launch Belt motor shelf · left", "Cassette side beam · left"),
        "Chain capture": ("Keeper drive mast", "Shared lift crosspiece"),
    }
    mounts = []
    for group, (member, host) in interfaces.items():
        assert ingredient[member] == group and ingredient[host] != group
        contact = BRepExtrema_DistShapeShape(roster[ids[member]], roster[ids[host]])
        expected_gap = .175 if member == "Battery service pin" else 0.
        assert contact.IsDone() and abs(contact.Value() - expected_gap) < 1e-5, (member, host)
        p = contact.PointOnShape2(1)
        mounts.append(dict(group=group, on=ids[host], pointMm=[p.X(), p.Y(), p.Z()],
            basis=f"Parked STEP surface datum on {host} opposite {member}. Measured separation {contact.Value():.3f} mm."))
    plan=dict(schema="agent-frc/preliminary-layout@1",issue=issue,datum=datum,step=step,design=design,motion=motion,groups=groups,
        mounts=mounts,stages=stages,cycle=cycle,budgets=budget,missingInputs=[],premises=[dict(accepted=accepted,current=current)],
        view="package-motion.html",presentation=dict(architecture="Single pivot cassette and translating support with guided climb",
        components=components),narrativeReview=dict(source=narrative,objectives=[dict(objective=obj,disposition="represented",
        basis="First measured fine-clock discharge and free flight are represented in the commissioned steps."),
        dict(objective="Recover six floor pieces, score, then climb",disposition="out-of-scope",basis="The season player retains the complete scoring and climb trials, including the missed repeatability target on perturbed starts. This mechanical check covers the first fine-clock discharge and a separate service operation.")]),
        control=dict(case=cycle[0],moving="Chambered NOTE",displaced=ids["Feed tread · left"],offsetMm=[0,-22,0],
                     basis="Push the feed tread beyond its registered radial NOTE allowance."))
    save("layout.json",plan)
    print("Prepared",len(roster),"placed articles,",len(cases),"cases at",root)


if __name__ == "__main__":
    main()
