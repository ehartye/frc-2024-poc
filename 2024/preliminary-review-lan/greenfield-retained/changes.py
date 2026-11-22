"""Preliminary article geometry in cassette and robot coordinates, millimetres."""
intent = {}
pin = 'Spring-return capture pin'
def specify(name, purpose, reason):
    intent[name] = dict(purpose=purpose,rationale=reason)
def build():
    def item(name,shape,purpose,why,group='Capture latch',material='6061 aluminum',category='mechanism'):
        local(name,shape,group,category,material)
        specify(name,purpose,why)
    # Two rigid bridges tie both cassette side rails to the receiver and the guide.
    for y,side in [(-226,'right'),(218,'left')]:
        item('Latch upright · '+side, box(125,y,19,10,8,61),'Connect roof to cassette side beam.','Transfers rim contact and latch reactions to the existing moving beam.')
    # Bottom crossbar already spans the two side beams. Ream with clearance around the pin.
    lower='Runner crossbar 125'
    lower_bore=cyl((130,55,13),Z,3.6,12)
    G[lower]=cut(G[lower],place(lower_bore,mat(park,*pivot)))
    bore=cyl((130,55,74),Z,3.6,35)
    item('Capture bridge',cut(box(121,-226,80,18,452,6),bore),'Keep the inverted ring on the cassette.','The span closes the load path across both uprights and guides the capture pin.')
    item('Lower pin socket',ann((130,55,19),Z,6,3.6,6),'Receive the spring-return pin through the NOTE opening.','The lower socket engages before lift and takes transverse pin load.')
    item('Latch solenoid and guide',ann((130,55,86),Z,11,3.65,82),'Retract the pin for pickup and discharge.','Spring extension captures the ring on power loss after confirmed seating. Housing includes spring and position-sensing allowance.','Capture latch','Solenoid, guide and spring envelope','electrical')
    item(pin,cyl((130,55,14),Z,3.5,84),'Pass through the central opening and seat in the lower socket.','A closed mechanical loop retains the NOTE without belt friction. Retract 66 mm to release.','Capture latch','Steel')
    # Full endless belt with a split-diameter nose roller, all driven by the existing nose shaft.
    nose='Nose pickup roller'
    old=G.pop(nose); moving.remove(nose); meta.pop(nose)
    drumshape=fuse(cyl((625,-217,13),Y,12,194),fuse(cyl((625,-23,13),Y,10,46),cyl((625,23,13),Y,12,194)))
    local(nose,cut(drumshape,cyl((625,-217,13),Y,6,434)),'Cassette drive','mechanism','Elastomer drum with relieved belt land')
    def stadium(r):
        web=box(91,-22.5,13-r,534,45,2*r)
        return fuse(fuse(web,cyl((91,-22.5,13),Y,r,45)),cyl((625,-22.5,13),Y,r,45))
    item('Center intake belt',cut(stadium(12),stadium(10)),'Drive the lower NOTE contact from pickup nose to chamber.','A continuous upper run replaces the gap between the powered nose and side belts.','Cassette drive','Elastomer belt')
    item('Center belt idler',ann((91,-22.5,13),Y,10,4,45),'Return and tension the center belt.','Bearing-mounted drum keeps the chamber end of the belt at runner height.','Cassette drive','Polymer and bearing envelope')
    item('Center idler axle',cyl((91,-32,13),Y,4,64),'React the idler load into its forks.','A through axle supports the return drum at both ends.','Cassette drive','Steel')
    for y,side in [(-31,'right'),(26,'left')]:
        # Each fork reaches the lower rear crossbar, outside the moving belt.
        sh=fuse(box(84,y,7,41,5,12),box(84,y,3,14,5,12))
        item('Center idler fork · '+side,cut(sh,cyl((91,y,13),Y,4,5)),'Mount the center idler axle to the runner crossbar.','Bearing support sits outside the belt width.','Cassette drive')
    # Armed service at 80 degrees; screw pin through aligned lug on rotating cheek and slide.
    # Endgame gross package is rear-mounted clear of battery extraction. Dual mast ties to sills.
    def end(name,shape,purpose,why,material='6061 aluminum',category='mechanism'):
        add(name,place(shape,mat(0,-17,0,0)),'Solo climber',category,material);specify(name,purpose,why)
    end('Climber bridge',box(-315,-248,146,35,496,20),'Carry the solo lift load to both longitudinal sills.','Rear placement keeps the center of the robot available for pickup and electrical service.')
    for y,side in [(181,'left'),(-201,'right')]:
        outer=box(-312,y,166,28,20,390); void=box(-309,y+3,165,22,14,392)
        end('Climber mast · '+side,cut(outer,void),'Guide the telescoping chain crosshead.','Two spaced guides carry off-center chain reactions into the bridge.')
        end('Climber slider · '+side,box(-308,y+4,286,20,12,275),'Carry the chain saddle between the guide rails.','At full reach each slider retains 65 mm of guide overlap.')
    end('Climber crosshead',box(-311,-197,561,26,394,15),'Join the telescoping sliders and support the chain saddle.','A common crosshead lifts both sides from one chain.')
    hook=prism([(-309,-8,576),(-296,-8,576),(-296,-8,625),(-261,-8,625),(-261,-8,607),(-249,-8,607),(-249,-8,638),(-309,-8,638)],(0,16,0))
    end('Chain saddle',hook,'Catch the field chain and receive the upward chain reaction.','A deep returning lip gives the chain a positive seat while the winch retracts.','Steel')
    end('Climb winch drive',cyl((-298,-94,205),Y,30,113),'Reel the crosshead toward the frame under load.','The reducer, motor and drum share a dedicated mass and bus-current allowance.','Geared brushless winch envelope','electrical')
    end('Winch foot',box(-312,-94,166,28,113,9),'Mount the winch case on the bridge.','The bridge routes cable tension into the supported frame.')
    end('Climb cable',cyl((-296,10,235),Z,1.6,326),'Connect winch drum to sliding crosshead.','Cable length changes as the carriage moves. End terminations are preliminary.','Steel wire rope')
    end('Winch hold pawl',box(-303,19,182,10,9,24),'Hold the drum after motor power is removed.','A spring-applied ratchet is required for the solo branch. Tooth form and rating remain to be selected.','Steel')
build()

