# Powered opening: mechanical contact port and operating gates

The cassette and servo cam have independent positions and momenta. This port never assigns cassette travel from a PWM target or cam angle. With inward travel `u` and cam angle `theta`, the unilateral gap is `g = e*(1-cos(theta))-u`. Positive gap is separated; negative gap is the explicit compliant-contact hypothesis.

The frictionless normal port returns reciprocal generalized forces `Q_u=-lambda` and `Q_theta=lambda*e*sin(theta)`, plus separate local application points and equal/opposite forces. It accounts for elastic potential and closing-only damping. Mechanical power plus potential-rate plus dissipative heat is zero up to floating-point error. Frame motion cancels between the internal force pair; callers must not add a second chassis reaction on top of those mapped forces.

At exactly zero gap this implementation has zero normal force, even when approaching. Damping starts only with positive penetration. That onset convention is deliberate and tested, not an impulse or exact nonpenetration solver. Stiffness/damping are uncalibrated parameters. Tangential cam friction, contact stress, wear and high-speed impacts are not modeled. The CAD study bounds theta to0..pi and cassette travel to0..1.5mm; leaving the domain rejects the query without resetting physical state or inventing a mechanical stop.

## Source operating envelope is not a motor model
Pinned SRS V1 documentation provides6V nominal,4.8..7.4V rating,13.5kg-cm stall torque,2A stated maximum stall current at6V, and0.13s/60degree speed. These do not determine an internal motor constant, torque-speed/current law, control bandwidth, reflected inertia, continuous holding temperature or PWM-loss behavior. Do not derive a supposedly calibrated servo plant from those few numbers or turn2A into an active current clamp.

The default270degree PWM mapping is available only after confirming that mode and supplying physical spline/cam clocking. The returned angle is a request, not measured position. Custom-angle and continuous modes have different semantics. Out-of-study targets remain flagged, not silently converted into physical state.

REV-11-1144 provides the historical6V power path; no direct12V servo supply. Its shared15A shutdown is not a3A per-port limiter. Regulator efficiency/transients and both actuator loads must be represented in the shared battery accounting before adoption. This standalone port does not synthesize electrical consumption or regulator response.

## Integration contract
The numerical owner confirmed the gap sign and reciprocal u-work convention. Inputs are independent `u, udot, theta, thetadot`; outputs include potential, heat, work and force/Jacobian data. Before moving-cassette integration, declare cassette mass, output/gear inertia, a bounded actuator/controller hypothesis, spring potential, stops and exact source/geometry identity. Preserve all state and accumulated energy at phase changes and power loss. The cam contact remains mechanically active with motor power off; neither motor-off nor zero dead-center torque is a positive latch. In fact, the open dead center lowers spring energy when perturbed toward inward travel.

No full simulator adoption or capture/holding acceptance is claimed. Source facts, geometric laws and uncalibrated hypotheses are deliberately separate. The eight tests cover finite-difference gradients,60 rigid-frame cases, damping/work identity, separated/onset states, dead-center stability, input rejection and command-versus-position semantics.

Locking-source search found no V1-specific tightening torque or threadlocker instruction in the retrieved manufacturer material. Do not import MAXSwerve screw torque or advice for a different servo revision. Final fastener locking and inspection remain an explicit manufacturing/maintenance gate, not an assumed CAD property.
