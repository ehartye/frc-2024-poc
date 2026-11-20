"""Parked-envelope and floor-custody rehearsal. Inches on the field, seconds on the match clock."""
from bisect import bisect_right
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
import hashlib
import json
import math

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
def read(p):
    return json.loads(p.read_bytes())
def save(p, value):
    p.write_bytes((json.dumps(value, separators=(",",":"), ensure_ascii=False)+"\n").encode())
def ref(p):
    return dict(path=p.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),pointer="")

FIELD = read(ROOT/"kickoff/crescendo-field.json")
PEERS = read(HERE/"peer-tracks.json")
BOX_MM = read(HERE/"footprint.json")["boundsMm"]
B3 = tuple(BOX_MM[i]/25.4 for i in (0,1,3,4))
PEER = (-17,-17,17,17)
DT = PEERS["dt"]
START, END = (p["seconds"] for p in FIELD["rehearsal"]["periods"])
FLOOR = START + END
RADIUS = 7.0
PORT = B3[2] + RADIUS + 1.0   # Proposed ground-transfer mouth, one inch between ring and bumper.
PADS = [(81.,99.), (137.,108.)]
SOURCES = {s["id"]:s["displayAt"] for s in FIELD["rehearsal"]["stations"] if s["id"] in ("center-1","center-2")}
CARRY = read(ROOT/"sim/pressure/evidence/maxswerve-development-carry.json")["runs"]
SHORT = read(ROOT/"sim/pressure/evidence/retained-pickup-20261118/short-lift-verified.json")
HEIGHT = min(r["minHoldClearanceM"] for r in SHORT["runs"])
assert SHORT["retainedAll"] and SHORT["spatialBoundPassed"] and SHORT["temporalBoundPassed"]
assert min(r["minimumCarrySideLoadN"] for r in CARRY.values()) > 0
STOCK = [PEERS["stock"][s][0] for s in ("center-2","center-1")]
assert STOCK == ["center-2/1","center-1/1"]
def distance(a,b): return math.dist(a,b)
def near(a,b,eps=1e-7): return distance(a,b)<eps
def hull(points):
    def half(arr):
        chain=[]
        for p in arr:
            while len(chain)>1:
                a,b=chain[-2:]; cross=(b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
                if cross>0: break
                chain.pop()
            chain.append(p)
        return chain[:-1]
    pts=sorted(set(map(tuple,points)))
    return half(pts)+half(pts[::-1])
STAGES = [hull([p for ob in FIELD["rehearsal"]["obstacles"] if ob["name"].startswith(side) for p in ob["poly"]]) for side in ("Blue","Red")]
def extent(p,q,body):
    return (min(p[0],q[0])+body[0],min(p[1],q[1])+body[1],
            max(p[0],q[0])+body[2],max(p[1],q[1])+body[3])
def gap(a,b):
    return max(b[0]-a[2],a[0]-b[2],b[1]-a[3],a[1]-b[3])
def polygon_gap(r,poly):
    # Separating-axis gap between convex protected stage and swept axis-aligned envelope.
    rect=[(r[0],r[1]),(r[2],r[1]),(r[2],r[3]),(r[0],r[3])]
    margins=[]
    for a,b in zip(poly,poly[1:]+poly[:1]):
        dx,dy=b[0]-a[0],b[1]-a[1]; length=math.hypot(dx,dy)
        axis=(-dy/length,dx/length)
        u=[x*axis[0]+y*axis[1] for x,y in rect]; v=[x*axis[0]+y*axis[1] for x,y in poly]
        margins.append(max(min(v)-max(u),min(u)-max(v)))
    xs,ys=zip(*poly)
    return max(*margins,gap(r,(min(xs),min(ys),max(xs),max(ys))))
def pose_at(rows,tick):
    i=bisect_right(rows,[tick,math.inf,math.inf])-1
    a=rows[max(0,i)]
    if i==len(rows)-1:return tuple(a[1:])
    b=rows[i+1]; u=(tick-a[0])/(b[0]-a[0])
    return tuple(a[j]+u*(b[j]-a[j]) for j in (1,2))

@dataclass
class Bot:
    name:str
    p:tuple
    path:list=field(default_factory=list)
    action:str="idle"
    time:float=0.
    cargo:str|None=None
    origin:tuple|None=None
    progress:int=0
    ticks:int=0
    travel:float=0.
    waits:Counter=field(default_factory=Counter)

    def route(self,points,action):
        self.path=list(points)
        self.action=action
        self.origin=None
        self.time=0

    def propose(self):
        while self.path and near(self.p,self.path[0]):
            self.path.pop(0)
            self.origin=None
        if not self.path:return self.p
        if self.origin is None:
            self.origin=self.p
            length=distance(self.p,self.path[0])
            # Cubic rest-to-rest segment. Peak speed and acceleration stay within the trial assumptions.
            duration=max(1.5*length/42, math.sqrt(6*length/42))
            self.ticks=max(1,math.ceil(duration/DT)); self.progress=0
        u=(self.progress+1)/self.ticks
        blend=u*u*(3-2*u)
        return tuple(a+(b-a)*blend for a,b in zip(self.origin,self.path[0]))

    def move(self,p):
        self.travel+=distance(self.p,p);self.p=p;self.progress+=1

def partner_track(tick):
    return {name:pose_at(rows,tick) for name,rows in PEERS["traces"].items()}

def dock(site):return (site[0]-PORT,site[1])
def rest_duration(h,e=.2):
    # Ballistic vertical settle under gravity with no horizontal launch; stop below 0.1 mm rebound.
    duration=math.sqrt(2*h/9.80665)
    rebound=h*e*e
    while rebound>.0001:
        duration+=2*math.sqrt(2*rebound/9.80665);rebound*=e*e
    return duration

def simulate(case,revision,freeze_sha,demand,control=None):
    control=control or {}
    delay=case["parameters"]["delay"];offset=case["parameters"]["offset"]
    initial_stock=STOCK[:control.get("available",2)]
    b=Bot("B3",tuple(PEERS["initial"]["B3"])); partner=Bot("B2",tuple(PEERS["initial"]["B2"]))
    pieces={item:dict(state="source",source=item.split("/")[0],xy=list(SOURCES[item.split("/")[0]])) for item in initial_stock}
    if STOCK[1] in pieces:pieces[STOCK[1]]["xy"][1]+=offset
    initial_positions={i:list(v["xy"]) for i,v in pieces.items()}
    serial=revision=="serial-confirm"
    events=[];history=[];samples=[];deposits=[];received=[];finished=[];clock=[]
    previous_velocity=(0.,0.); peak_accel=0.; current_accel=0.; contacts=set(); acquisition_seen=set()
    min_clear=math.inf; min_hold=math.inf; failure=None; piece=None; pad=None; retry=False
    def log(t,kind,actor,item=None,**data):
        history.append(dict(t=round(t,3),kind=kind,actor=actor,piece=item,**data))
    def contract(t,kind,item,station,witness,**data):
        events.append(dict(timeS=round(t,3),kind=kind,piece=item,focalRobot="B3",station=station,
                           criterionId=demand["criterionId"],witness=witness,**data))
    def ext(bot):
        body=B3 if bot is b else PEER
        if bot.cargo:return (body[0],body[1],PORT+RADIUS,body[3])
        return body
    def departure(target):
        # Reach southern through-lane before crossing the held-out defensive route.
        return [(b.p[0],31.),(target[0],31.),target]
    def start_source(t):
        nonlocal piece,retry
        left=[s for s in initial_stock if pieces[s]["state"]=="source"]
        if not left:
            b.route([(b.p[0],54.),(94.,54.)],"reset");return
        piece=left[0];retry=False
        aim=dock(SOURCES[pieces[piece]["source"]])
        b.route(departure(aim),"sense");log(t,"dispatch","B3",piece)

    def blockers(bot,dest,other,otherdest,peer0,peer1):
        sweep=extent(bot.p,dest,ext(bot))
        margin=min(sweep[0],sweep[1],FIELD["field"]["length"]-sweep[2],FIELD["field"]["width"]-sweep[3])
        obstacles=[("wall",margin)]
        obstacles.extend((f"stage-{i}",polygon_gap(sweep,p)) for i,p in enumerate(STAGES))
        for k,p in peer0.items():
            obstacles.append((k,gap(sweep,extent(p,peer1[k],PEER))))
        obstacles.append((other.name,gap(sweep,extent(other.p,otherdest,ext(other)))))
        # Settled floor rings occupy their own locations through receipt and reset.
        for item,v in pieces.items():
            if v["state"] not in ("pad","stored"):continue
            ring=extent(v["xy"],v["xy"],(-RADIUS,-RADIUS,RADIUS,RADIUS))
            obstacles.append((item,gap(sweep,ring)))
        return min(obstacles,key=lambda o:o[1])
    def lane_available(tick):
        # Shared published peer itinerary. Reserve the complete stationary pickup
        # and retreat to the southern through-lane before leaving that lane.
        dest=b.path[0]
        length=distance(b.p,dest)
        travel=max(1,math.ceil(max(1.5*length/42,math.sqrt(6*length/42))/DT))
        dwell=math.ceil(CARRY["D1"]["seatedS"]/DT)+6
        # Retain a conservative occupied footprint at the most exposed end for
        # the complete round trip. No acceptance uses a future custody outcome.
        zone=extent(b.p,dest,(B3[0],B3[1],PORT+RADIUS,B3[3]))
        for future in range(tick,min(round(END/DT),tick+2*travel+dwell)+1):
            for name,rows in PEERS["traces"].items():
                a=pose_at(rows,future);c=pose_at(rows,min(future+1,round(END/DT)))
                if gap(zone,extent(a,c,PEER))<=.1:
                    return False
        return True
    partner_resets=[]
    def observe(t):
        nonlocal piece,pad,retry,failure,min_hold
        if b.cargo:
            v=pieces[b.cargo];v["xy"]=[b.p[0]+PORT,b.p[1]]
            retained=control.get("effectiveSideLoadN",min(CARRY[k]["minimumCarrySideLoadN"] for k in ("D1","D2")))
            margin=.25*retained-.2413*current_accel*.0254
            min_hold=min(min_hold,margin)
            if margin<=0:
                contract(t,"miss",b.cargo,"route","Carry force budget exhausted",release=True)
                log(t,"lost","B3",b.cargo);v["state"]="lost";b.cargo=None;failure="insufficient retention";return
        if partner.cargo:pieces[partner.cargo]["xy"]=[partner.p[0]+PORT,partner.p[1]]
        if b.action=="start":start_source(t)
        if not b.path:
            b.time+=DT
            if b.action=="sense":
                run=CARRY["D1" if piece==STOCK[0] else "D2"]
                if b.time>=run["seatedS"]:
                    actual=pieces[piece]["xy"];target=(b.p[0]+PORT,b.p[1])
                    if pieces[piece].get("absent"):
                        log(t,"empty after traffic sweep","B3",piece)
                        pieces[piece]["state"]="unlocated";start_source(t);return
                    if distance(actual,target)>1.5:
                        log(t,"missed contact","B3",piece,residualIn=round(distance(actual,target),3))
                        if retry:failure="second contact missed";return
                        retry=True
                        # Local lateral search only inside the stipulated sensor capture band.
                        if distance(actual,target)>4:
                            log(t,"outside search band","B3",piece)
                            pieces[piece]["state"]="unlocated";start_source(t);return
                        b.route([dock(actual)],"sense")
                    else:
                        b.cargo=piece;pieces[piece]["state"]="B3";acquisition_seen.add(piece)
                        contract(t,"acquire",piece,pieces[piece]["source"],"Stationary mouth within 1.5 in for archived seating duration")
                        log(t,"acquired","B3",piece,archive=run["attempt"],freshLocalDetection=True,residualIn=round(distance(actual,target),3))
                        pad=0 if serial else len(deposits)%2
                        home=dock(PADS[pad])
                        b.route([(b.p[0],31.),(home[0],31.),home],"deposit")
            elif b.action=="deposit":
                v=pieces[piece]
                if b.cargo and b.time>=.35:
                    occupied=any(other.get("pad")==pad and other["state"]=="pad" for other in pieces.values())
                    if not occupied:
                        # A stationary release has no horizontal ejection velocity.
                        landing=[b.p[0]+PORT,b.p[1]+control.get("padOffsetIn",0)]
                        v.update(state="settling",xy=landing,releasedAt=t,pad=pad)
                        b.cargo=None
                        log(t,"ground release","B3",piece,xy=landing,heightMm=round(1000*HEIGHT,3))
                if v["state"]=="settling" and t-v["releasedAt"]>=rest_duration(HEIGHT):
                    valid=distance(v["xy"],PADS[pad])+RADIUS<=10.
                    v["state"]="pad" if valid else "lost"
                    if not valid:
                        contract(t,"miss",piece,"blue-exchange","Settled ring crosses pad edge",release=True)
                        log(t,"outside pad","B3",piece);failure="invalid deposit";return
                    contract(t,"accept",piece,"blue-exchange","At rest on floor; complete ring inside empty pad")
                    deposits.append(piece);log(t,"floor deposit","B3",piece,pad=pad,xy=v["xy"])
                    b.route([(b.p[0],49.)],"confirm" if serial else "next")
            elif b.action in ("confirm","next"):
                if b.action=="next" or deposits[-1] in received:start_source(t)
            elif b.action=="reset":
                b.action="parked";log(t,"reset","B3")
        if not partner.path:
            partner.time+=DT
            if partner.action=="standby" and finished and finished[-1] not in partner_resets:
                partner_resets.append(finished[-1]);log(t,"reset","B2",finished[-1])
            if partner.action=="receive" and t-START<delay:
                partner.time=0.
            if partner.action=="standby" and (t-START>=delay or control.get("preposition")):
                waiting=[i for i in deposits if pieces[i]["state"]=="pad"]
                if waiting and distance(b.p,dock(pieces[waiting[0]]["xy"]))>40:
                    item=waiting[0]; dest=dock(pieces[item]["xy"])
                    partner.route([(partner.p[0],dest[1]),dest],"receive")
            elif partner.action=="receive" and partner.time>=2.45:
                port=(partner.p[0]+PORT,partner.p[1])
                found=next((i for i in deposits if pieces[i]["state"]=="pad" and near(pieces[i]["xy"],port)),None)
                if found:
                    partner.cargo=found;pieces[found]["state"]="B2";received.append(found)
                    log(t,"received","B2",found)
                    terminal=(30.,140. if len(received)==1 else 75.)
                    if control.get("localStore") and len(received)==1:
                        downstream = (partner.p[0],133.5) if control["localStore"]=="east" else (30.,133.5)
                        partner.route([downstream],"store")
                    else:
                        partner.route([(30.,partner.p[1]),terminal],"store")
            elif partner.action=="store" and partner.cargo and partner.time>=.35+rest_duration(HEIGHT):
                item=partner.cargo;partner.cargo=None;pieces[item]["state"]="stored";finished.append(item)
                log(t,"downstream floor","B2",item,xy=list(pieces[item]["xy"]))
                partner.route([] if control.get("localStore") and len(received)==1 else [(30.,150.)],"standby")
    b.action="start"
    partner.route([(30.,298.),(30.,150.)],"standby")
    for tick in range(round(END/DT)+1):
        t=round(START+tick*DT,3)
        if tick==0:clock.append(dict(kind="phase-start",phase="TELEOP",timeS=t,witness="Field engine clock"))
        if tick%10==0:
            samples.append([t,*[round(n,2) for n in (*b.p,*partner.p)],b.cargo,partner.cargo])
        if tick==round(END/DT):break
        if failure:continue
        p0,p1=partner_track(tick),partner_track(tick+1)
        for actor,pos in p0.items():
            peerbox=extent(pos,p1[actor],PEER)
            for item,v in pieces.items():
                if v["state"] not in ("source","pad","stored"): continue
                x,y=v["xy"];delta=math.hypot(max(peerbox[0]-x,x-peerbox[2],0),
                                               max(peerbox[1]-y,y-peerbox[3],0))-RADIUS
                pair=(actor,item,v["state"])
                if delta<=0 and pair not in contacts:
                    contacts.add(pair)
                    log(t,"peer crossed piece projection",actor,item,state=v["state"])
                    if v["state"]=="source" and control.get("sweptAway"):
                        v["absent"]=True
                    if v["state"]!="source":
                        failure="unresolved peer contact with deposited piece"
        before=(b.p,partner.p)
        for bot,other,other_start in ((b,partner,partner.p),(partner,b,before[0])):
            dest=bot.propose()
            if (bot is b and not control.get("unannouncedTraffic") and b.action=="sense" and b.path
                    and piece==STOCK[0] and b.progress==0 and near(b.p,(dock(SOURCES["center-2"])[0],31.))
                    and not lane_available(tick)):
                b.waits["reserved approach"]+=DT
                dest=b.p
                b.origin=None  # no elapsed trajectory progress while the reservation is closed
            # Check both actors' swept footprints for the same interval.
            if bot is partner:
                save_pos=b.p;b.p=other_start
                who,room=blockers(bot,dest,b,save_pos,p0,p1)
                b.p=save_pos
            else:who,room=blockers(bot,dest,other,other.p,p0,p1)
            if room<=control.get("clearanceIn",.01):
                bot.waits[who]+=DT
                # A moving peer entering a stopped footprint invalidates the route.
                who,room=blockers(bot,bot.p,other,other.p,p0,p1)
                if room<=control.get("clearanceIn",0.):
                    min_clear=min(min_clear,room)
                    failure=("occupied footprint: " if room<=0 else "clearance margin: ")+who
                    log(t,"collision" if room<=0 else "margin breach",bot.name,detail=who,gapIn=round(room,3));break
            else:bot.move(dest)
            min_clear=min(min_clear,room)
        velocity=tuple((x-y)/DT for x,y in zip(b.p,before[0]))
        current_accel=distance(velocity,previous_velocity)/DT
        if b.cargo:peak_accel=max(peak_accel,current_accel)
        previous_velocity=velocity
        if not failure and tick+1<round(END/DT):observe(round(t+DT,3))
    assert len(set(deposits))==len(deposits) and set(finished)<=set(received)<=set(deposits)<=set(initial_stock)
    assert set(deposits)<=acquisition_seen
    assert not b.cargo or b.cargo!=partner.cargo
    clock=[dict(kind="match-start",timeS=0,witness="Archived field clock, AUTO begins"),*clock,
           dict(kind="phase-end",phase="TELEOP",timeS=FLOOR,witness="TELEOP cutoff"),
           dict(kind="match-end",timeS=FLOOR,witness="Regulation clock")]
    receipt=dict(schema="agent-frc/throughput-execution@1",caseId=case["id"],runId=revision+"-"+case["id"],
        revision=revision,evidence="timing-model",cohortSha256=freeze_sha,criterionId=demand["criterionId"],
        criterion=demand["criterion"],focalRobot="B3",initial=dict(receiverAvailableS=delay,secondOffsetIn=offset),
        clock=clock,windowS=[START,FLOOR],initialPieces=initial_stock,events=events)
    outcome=dict(initialPositions=initial_positions,deposits=deposits,received=received,stored=finished,reset=b.action=="parked",
        partnerReset=partner.action=="standby" and not partner.path,matchPoints={"B3":0,"B2":0},
        cutoffReason=failure or "clock",minimumGapIn=round(min_clear,3),
        peakLoadedAccelInS2=round(peak_accel,3),pickupEvidenceClass="local pose and dwell, archived simulation calibration",
        carryMarginN=round(min_hold,4) if math.isfinite(min_hold) else None,
        travelIn={r.name:round(r.travel,2) for r in (b,partner)},
        waitsS={r.name:{k:round(v,2) for k,v in r.waits.items()} for r in (b,partner)},
        finalCustody={i:v["state"] for i,v in pieces.items()},history=history,samples=samples)
    return receipt,outcome
