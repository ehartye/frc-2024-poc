"""Execute the two ground custody roles on the same clock and four archived peer tracks."""
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
PARENT = HERE.parent / "ground-relay"
sys.path.insert(0, str(PARENT))
import relay
from contact import nominal_permitted

DT = relay.DT
START = 15.
HORN = 150.
SEAT = 241.5 / 25.4
LOADS = relay.read(HERE / "loads.json")[-1]
CONTACT = relay.read(HERE / "contact.json")
PADS = relay.PADS
PEER_BOXES = [
    {k:relay.extent(relay.pose_at(rows,i),relay.pose_at(rows,i+1),relay.PEER)
     for k,rows in relay.PEERS["traces"].items()}
    for i in range(2700)
]


class Stop(Exception):
    """An observed gate rejected further credit."""


@dataclass
class Task:
    bot: relay.Bot
    program: object = None
    command: object = None
    ready: float = START
    done: bool = False

    def advance(self, t):
        while not self.done:
            if self.command:
                kind, value = self.command
                if kind == "move" and self.bot.path:
                    return
                if kind == "hold" and t < self.ready-1e-8:
                    return
                if kind == "wait" and not value():
                    return
            self.command = next(self.program, None)
            if self.command is None:
                self.done = True
                return
            kind, value = self.command
            if kind == "move":
                self.bot.route(value, "move")
            elif kind == "hold":
                self.ready = t+value


def run(case, control="revised"):
    if not nominal_permitted(CONTACT):
        raise ValueError("Nominal registered contact path is unavailable")
    t = START
    b3, b2 = (relay.Bot(name, tuple(relay.PEERS["initial"][name])) for name in ("B3", "B2"))
    agents = [Task(b3), Task(b2)]
    pieces = {item: {"xy": list(relay.SOURCES[item.split("/")[0]]), "state": "source"} for item in relay.STOCK}
    pieces[relay.STOCK[1]]["xy"][1] += case["parameters"]["offset"]
    if control == "empty":
        pieces.clear()
    history, deposits, received, stores = [], [], [], []
    phase = "travel"
    target = None
    reason = "horn"
    minimum_gap = math.inf
    minimum_hold = math.inf
    maximum_accel = 0
    previous_velocity = (0., 0.)
    receiver_ready = True
    crossed = set()

    def log(actor, action, item=None, **values):
        history.append({"timeS": round(t,2), "actor": actor, "action": action, "piece": item, **values})

    def gate(ok, message):
        if not ok:
            raise Stop(message)

    def wait(seconds):
        return ("hold", seconds)

    def move(*points):
        return ("move", list(points))

    def dock(xy, port):
        return (xy[0]-port, xy[1])

    reservation = []
    def lane_free(outer, piece_xy):
        if control != "rectangularReservation":
            if not reservation:
                probe = relay.Bot("reservation", b3.p)
                def leg(point):
                    probe.route([point], "forecast")
                    while probe.path:
                        dest = probe.propose()
                        reservation.append(relay.extent(probe.p, dest, relay.B3))
                        probe.move(dest)
                leg(outer)
                leg(dock(piece_xy, SEAT))
                dwell = math.ceil((relay.CARRY["D1"]["seatedS"]+.8+.15)/DT)
                reservation.extend([relay.extent(probe.p,probe.p,relay.B3)]*dwell)
                leg(outer)
                leg((outer[0],31.))
            start = round((t-START)/DT)
            for i,area in enumerate(reservation):
                k = start+i
                if k >= 2700:
                    return False
                if any(relay.gap(area,peer)<=.2 for peer in PEER_BOXES[k].values()):
                    return False
            return True
        # Reserve the entire exposed source path, including insertion, lift and
        # return to the southern lane. The same four peer paths remain on their clock.
        envelope = relay.extent(b3.p, dock(piece_xy, SEAT), relay.B3)
        length = abs(b3.p[1]-outer[1])
        crossings = 2*max(1.5*length/42, math.sqrt(6*length/42))
        horizon = crossings+2*max(1.5*(relay.PORT-SEAT)/42, math.sqrt(6*(relay.PORT-SEAT)/42))+6
        lo = round((t-START)/DT)
        for k in range(lo, min(2699,lo+math.ceil(horizon/DT))+1):
            for other in PEER_BOXES[k].values():
                if relay.gap(envelope,other) <= .1:
                    return False
        return True

    def carrier():
        nonlocal phase, target
        for index, item in enumerate(pieces):
            target = item
            ring = pieces[item]
            nominal = relay.SOURCES[item.split("/")[0]]
            outer = dock(nominal,relay.PORT)
            yield move((b3.p[0],31.),(outer[0],31.))
            if index == 0 and control != "unreserved":
                yield ("wait",lambda:lane_free(outer,ring["xy"]))
            yield move(outer)
            if ring.get("absent"):
                ring["state"] = "unlocated"
                log("B3","source missing after traffic",item)
                continue
            error = ring["xy"][1]-b3.p[1]
            gate(abs(error)<=4, "outside local search")
            if error:
                log("B3","lateral correction",item,initialOffsetIn=round(error,3))
                yield wait(.35)
            y_error = 3/25.4 if control=="offset3mm" else 0.
            source = list(ring["xy"])
            outer = (source[0]-relay.PORT,source[1]+y_error)
            yield move(outer)
            if control=="externalDock":
                gate("534" in CONTACT["externalDock"],"no registered tread contact at external dock")
            phase = "entry"
            if control=="offset3mm":
                gate(CONTACT["offset3mm"]["535"][1]<0 and "534" in CONTACT["offset3mm"]
                     and CONTACT["offset3mm"]["534"][1]<0, "offset entry contacts one tread and a pulley")
            gate(LOADS["withdrawalN"]["free"] < LOADS["floorLimitN"],"ring moves before seating")
            yield move(dock(source,SEAT))
            yield wait(relay.CARRY["D1" if index==0 else "D2"]["seatedS"])
            gate(all(CONTACT["floorSeat"][x][1]<0 for x in ("534","535")),"missing bilateral seat")
            gate(relay.near((b3.p[0]+SEAT,b3.p[1]),ring["xy"]),"incorrect source identity or pose")
            gate(LOADS["tractionN"]>LOADS["weightN"],"insufficient lift traction")
            if control=="noGrip":
                raise Stop("zero holding traction")
            b3.cargo=item
            ring["state"]="B3"
            log("B3","bilateral floor seat",item,treads=[534,535],bodyXmm=241.5)
            lift_peak = 6*.008/.8**2
            gate(LOADS["tractionN"] > LOADS["weightN"]+.2413*lift_peak,
                 "insufficient force during short lift")
            yield wait(.8)
            phase="transport"
            log("B3","seated lift complete",item,riseMm=8)
            pad=PADS[index]
            approach=dock(pad,relay.PORT)
            yield move(outer,(outer[0],31.),(approach[0],31.),approach,dock(pad,SEAT))
            yield wait(.8)
            gate(relay.near((b3.p[0]+SEAT,b3.p[1]),pad),"pad position mismatch")
            gate(all(pieces[i]["state"]!="pad" or pieces[i]["xy"]!=list(pad) for i in deposits),
                 "exchange pad occupied")
            if control=="raisedWithdrawal":
                gate(CONTACT["raisedStroke"]["R9 bumper cover envelope / R7 bumper"][0]>0,
                     "raised withdrawal intersects front cover")
            phase="peel"
            log("B3","floor support at pad",item)
            drag=LOADS["withdrawalN"]["braked" if control=="braked" else "free"]
            gate(drag < LOADS["floorLimitN"], "withdrawal drag exceeds floor restraint")
            b3.cargo=None
            ring.update(state="withdrawing",xy=list(pad))
            yield move(approach)
            yield wait(.35)
            gate(relay.near(ring["xy"],pad),"ring left exchange pad")
            ring["state"]="pad"
            deposits.append(item)
            log("B3","clear floor deposit",item,pad=index,dragN=drag)
            phase="travel"
            yield move((b3.p[0],49.))
        yield move((b3.p[0],54.),(94.,54.))
        log("B3","reset")

    def receiver():
        nonlocal receiver_ready
        yield move((30.,298.),(30.,150.))
        while True:
            def available():
                return ((t-START>=case["parameters"]["delay"] or control=="directDispatch") and any(
                    pieces[i]["state"]=="pad" and relay.distance(b3.p,dock(pieces[i]["xy"],relay.PORT))>40
                    for i in deposits))
            yield ("wait",available)
            item=next(i for i in deposits if pieces[i]["state"]=="pad" and
                      relay.distance(b3.p,dock(pieces[i]["xy"],relay.PORT))>40)
            receiver_ready=False
            dest=dock(pieces[item]["xy"],relay.PORT)
            yield move((b2.p[0],dest[1]),dest)
            yield ("wait",lambda:t-START>=case["parameters"]["delay"])
            yield wait(2.45)
            gate(pieces[item]["state"]=="pad","receiver cannot find deposited identity")
            b2.cargo=item
            pieces[item]["state"]="B2"
            received.append(item)
            log("B2","fixture receiver pickup",item)
            terminal=(30.,140. if len(received)==1 else 75.)
            local_store = control=="directDispatch" and len(received)==1
            if local_store:
                yield move((30.,133.5))
            else:
                yield move((30.,b2.p[1]),terminal)
            yield wait(.45)
            pieces[item].update(state="stored",xy=[b2.p[0]+relay.PORT,b2.p[1]])
            stores.append(item)
            b2.cargo=None
            log("B2","downstream store",item)
            if not local_store:
                yield move((30.,150.))
            receiver_ready=True
            log("B2","reset",item)

    agents[0].program,agents[1].program=carrier(),receiver()

    def bounds(bot):
        body=relay.B3 if bot is b3 else relay.PEER
        port=SEAT if bot is b3 else relay.PORT
        return (body[0],body[1],max(body[2],port+relay.RADIUS) if bot.cargo else body[2],body[3])

    def floor_pass(item, bot, end):
        if bot is not b3 or target!=item or phase not in ("entry","peel"):
            return False
        xy=pieces[item]["xy"]
        return all(abs(p[1]-xy[1])<.00001 and SEAT-.00001<=xy[0]-p[0]<=relay.PORT+.00001
                   for p in (bot.p,end))

    try:
        for tick in range(2700):
            t=round(START+tick*DT, 3)
            for task in agents:
                task.advance(t)
            ends=[bot.propose() for bot in (b3,b2)]
            positions=[b3.p,b2.p]
            boxes=[relay.extent(bot.p,end,bounds(bot)) for bot,end in zip((b3,b2),ends)]
            peers=PEER_BOXES[tick]
            for item, ring in pieces.items():
                if ring["state"] not in ("source","pad","stored","withdrawing"):
                    continue
                x,y=ring["xy"]
                for name,box in peers.items():
                    gap=math.hypot(max(box[0]-x,x-box[2],0),max(box[1]-y,y-box[3],0))-relay.RADIUS
                    key=(name,item,ring["state"])
                    if gap<=0 and key not in crossed:
                        crossed.add(key)
                        log(name,"crossed floor projection",item)
                        gate(ring["state"]=="source","peer crosses deposited NOTE")
                        if control=="sweptAway":
                            ring["absent"]=True
            def closest(i,bot,end,box):
                length,width=relay.FIELD["field"]["length"],relay.FIELD["field"]["width"]
                gaps=[("field",min(box[0],box[1],length-box[2],width-box[3])),
                      ("other carrier",relay.gap(box,boxes[1-i]))]
                gaps.extend((name,relay.gap(box,other)) for name,other in peers.items())
                gaps.extend(("stage",relay.polygon_gap(box,p)) for p in relay.STAGES)
                for item,ring in pieces.items():
                    if ring["state"] in ("source","pad","stored","withdrawing") and not floor_pass(item,bot,end):
                        floor=relay.extent(ring["xy"],ring["xy"],(-7,-7,7,7))
                        gaps.append((item,relay.gap(box,floor)))
                return min(gaps,key=lambda v:v[1])
            paused = set()
            for i,bot in enumerate((b3,b2)):
                who,room=closest(i,bot,ends[i],boxes[i])
                if room<=.01:
                    ends[i]=bot.p
                    boxes[i]=relay.extent(bot.p,bot.p,bounds(bot))
                    paused.add(i)
            for i,bot in enumerate((b3,b2)):
                who,room=closest(i,bot,ends[i],boxes[i])
                minimum_gap=min(minimum_gap,room)
                gate(room>.01,bot.name+" swept envelope meets "+who)
            for i,(bot,end) in enumerate(zip((b3,b2),ends)):
                if i not in paused:
                    bot.move(end)
                if bot.cargo:
                    pieces[bot.cargo]["xy"]=[bot.p[0]+(SEAT if bot is b3 else relay.PORT),bot.p[1]]
            velocity=tuple((p-q)/DT for p,q in zip(b3.p,positions[0]))
            ax,ay=((p-q)/DT*.0254 for p,q in zip(velocity,previous_velocity))
            previous_velocity=velocity
            if b3.cargo:
                maximum_accel=max(maximum_accel,math.hypot(ax,ay))
                margin=min(LOADS["tractionN"]-math.hypot(LOADS["weightN"],.2413*ax),
                           LOADS["lateralAtPoint1MmN"]-.2413*abs(ay))
                minimum_hold=min(margin,minimum_hold)
                gate(margin>0,"insufficient carry force")
    except Stop as e:
        reason=str(e)
        log("system","stopped",target,reason=reason)
    return {"case":case["id"],"control":control,"deposits":deposits,"received":received,"stored":stores,
            "carrierReset":agents[0].done,"receiverReady":receiver_ready,"points":{"B3":0,"B2":0},
            "stop":reason,"minimumPlanGapIn":round(minimum_gap,3) if math.isfinite(minimum_gap) else None,
            "minimumCarryMarginN":round(minimum_hold,4) if math.isfinite(minimum_hold) else None,
            "peakLoadedAccelerationMps2":round(maximum_accel,4),
            "custody":{i:v["state"] for i,v in pieces.items()},"history":history}


def generate():
    cases=relay.read(PARENT/"buffer-freeze.json")["cases"]
    results={case["id"]:run(case) for case in cases}
    results["diagnostic/directDispatch"]=run(cases[-1],"directDispatch")
    for control in ("externalDock","offset3mm","raisedWithdrawal","braked","empty","noGrip",
                    "sweptAway","unreserved","rectangularReservation"):
        results["control/"+control]=run(cases[0],control)
    (HERE/"results.json").write_bytes((json.dumps(results,indent=2)+"\n").encode())


if __name__=="__main__":
    generate()
