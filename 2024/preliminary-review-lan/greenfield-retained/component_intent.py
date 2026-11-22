"""Component explanations for this concept. Handed duplicates share design decisions."""
import re
lines="""Chassis perimeter|Tie the outer chassis together and back the bumper.|The outer contour establishes the forward extension datum used to schedule the moving pivot.
Blue bumper set|Cushion perimeter contact.|Its full depth and height constrain the approach to a floor NOTE.
Bay crossmember|Tie the two sills together and support the electrical deck.|Moving the crossmembers inboard freed space for the rear drive motors.
Chassis sill|Carry the pivot supports into the chassis.|Two spaced longitudinal members give each support a direct footprint.
Pivot A-frame|Transfer cassette bearing loads into the sill.|The outboard legs straddle the centered NOTE path and spread the pivot reaction between two feet.
Slide base|Seat the linear guide on top of the pivot support.|The elevated rail lets the cassette translate above the bumper during rotation.
Slide guide|Constrain the translating bearing block to fore-aft travel.|Retraction through mid-swing is needed to stay within forward reach.
Translating bearing block|Move the pivot bearing with the linear carriage.|Both sides of the shaft must translate together to retain its axis.
Pivot bearing|Support the rotating transverse shaft.|The shaft needs a radial reaction on each side outside the piece channel.
Cassette cheek|Join the rotating cassette frame to the pivot shaft.|Its shaft bore locates the channel relative to the supported rotation axis.
Cassette side beam|Support the running gear along the intake channel.|Keeping structure outboard leaves both runner surfaces clear for the NOTE.
Entry bearing bracket|Seat the nose roller axle at the cassette mouth.|Its actual tip controls extension and carpet clearance throughout the swing.
Belt motor shelf|Carry the belt motor and pass its output shaft.|The drive-side mount must share the pulley axis without obstructing the runner crossbar.
Belt motor|Drive one traction belt in either direction.|Independent side drives allocate intake and launch torque on each side of the NOTE.
Belt drive shaft|Transmit motor torque to the drive pulley.|Its actual diameter needs a through-hole in the motor shelf.
Nose pulley shelf|Support the far idler axle.|The pulley center distance establishes the long straight contact run.
Idler axle|Locate the undriven pulley.|The belt needs a return end that holds position under tension.
Opposed traction belt|Advance or release the NOTE along the channel.|A reversible side contact lets one channel serve intake and launch. Normal force and grip are open.
Drive pulley|Transmit shaft torque into the traction belt.|The drive end converts motor rotation into linear belt travel.
Idler pulley|Turn the belt back along its return run.|The far pulley defines the entry end of the powered side contact.
NOTE runner|Support the underside of the NOTE during transfer.|Two strips leave central volume available and keep sliding surfaces away from the side belts.
tire|Apply traction to the floor at this chassis corner.|Four separated contacts support the mobile base and its loaded cassette.
wheel axle|Locate the wheel and transmit wheel torque.|A real axle reserves the space between the wheel and both supports.
module deck|Attach this corner drive package to the frame.|Its top face carries both motors and constrains electronics packaging.
axle support|React wheel loads into the module deck.|A support on each side bounds the axle span and exposes wheel clearance.
Swerve drive motor|Supply tractive torque at this corner.|Independent drive authority is part of the selected four-module base.
Swerve steer motor|Set this corner module angle.|Independent azimuth lets the robot approach and leave the floor piece on different headings.
Pivot torque shaft|Transmit pivot torque across the cassette.|One transverse axis aligns the two outboard bearings and the rotating cheeks.
Pivot reducer|Convert motor speed into cassette pivot torque.|A right-angle package fits beside the moving carriage while preserving forward clearance.
Pivot motor|Rotate the loaded cassette between intake and aiming poses.|The lift requires active torque against gravity, with holding capacity still to be verified.
Slide screw|Translate the pivot carriage fore and aft.|The scheduled shift removes the long fixed-pivot overreach.
Slide drive motor|Turn the carriage screw.|The slide needs independently controlled movement as the pivot rotates.
Slide bearing mount|Locate the slide screw on the frame.|The screw needs aligned end bearings across the translating stroke.
Travel nut and clevis|Convert screw rotation into carriage travel.|The offset connection couples the translating truck to the outboard screw.
Runner crossbar|Join the two cassette beams and support the runner strips.|Spaced crossbars support the long sliding bed while clearing both belt drives.
Rear piece stop|Limit movement toward the pivot end of the channel.|A repeatable seat establishes the starting pose; this stop cannot restrain motion out of the open face.
Nose pickup roller|Make the first powered contact with a floor NOTE.|The side belts alone cannot reach the ring at the initial approach.
Nose roller shaft|Locate and drive the full-width nose roller.|The shaft spans both entry brackets while leaving the pickup surface continuous.
Nose motor|Drive the nose roller during capture.|The entry contact needs torque before the ring reaches the opposed side belts.
Electronics deck|Mount the control and power equipment between the sills.|A shared service plane separates the electronics from the rotating cassette.
Battery and terminal case|Supply power for the match.|The installed envelope must accommodate the complete pack and a service withdrawal.
PDH|Distribute power to robot branches.|A placed distribution hub anchors the downstream power routing.
roboRIO flat|Run the robot control loops.|The control computer needs a mounted site and an allocation in each electrical duty.
radio|Provide the field communications link.|The real package reserves deck area and forms part of the continuous electrical load.
main breaker|Disconnect and protect the main battery circuit.|Its installed location sets the route between the battery and distribution hub.
Pack lead|Reserve the cable route from the battery to the breaker.|The bend and terminals must fit beside the controls and disconnect for service.
Distribution lead|Reserve the main feed from the breaker to the distribution hub.|A routed tube exposes bend and neighbor constraints that a schematic omits.
Chambered NOTE|Represent the finite foam article through pickup, carry and discharge.|Its ring geometry exposes lost powered contact and open escape paths in the cassette."""
entries={k:(purpose,rationale) for k,purpose,rationale in (line.split('|') for line in lines.splitlines())}
def explain(name):
    key=name.split(' · ')[0]
    if key=='Belt pulley':key='Drive pulley' if name.endswith(' drive') else 'Idler pulley'
    key=re.sub(r'^(Front|Rear) (left|right) ','',key)
    key=re.sub(r' -?\d+$','',key)
    purpose,rationale=entries[key]
    return dict(purpose=purpose,rationale=rationale)

