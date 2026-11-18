"""Bounded pre-sliding shear on the two halves of a planar native belt contact."""
import numpy as np

from native_pressure_lift import LiftDomainError, NativePressureLift


class ShearLift(NativePressureLift):
    """Axial tread shear shares each pressure sample's circular Coulomb budget.

    Each fore/aft region has one elastic coordinate. Its rate is the
    normal-load-weighted axial slip, conjugate to its resultant force.
    This reduced constitutive hypothesis stops if an elastic region yields.
    """

    def __init__(self, *args, shear_stiffness):
        if not np.isfinite(shear_stiffness) or shear_stiffness <= 0:
            raise ValueError("positive shear stiffness required")
        self.stiffness = shear_stiffness
        super().__init__(*args)
        self.s["axialShearM"] = [0.0, 0.0]

    def contact(self, s, cache, rotors=None):
        con = super().contact(s, cache, rotors)
        p, original = con["pressure"], con["q"]
        d, note = s["chassis"], s["note"]
        axis = np.array([-np.sin(s["theta"]), 0.0, np.cos(s["theta"])])
        sidewall = np.abs(p["normalWorld"] @ axis) < 1e-8
        note_velocity = np.array([*note["v"], s["noteVz"]])
        cradle_velocity = np.array([d["vx"], d["vy"], s["vPivot"]])
        note_spin = np.array([0.0, -s["notePitchRate"], note["spin"]])
        cradle_spin = np.array([0.0, -s["thetaRate"], d["yaw_rate"]])
        body = note_velocity + np.cross(note_spin, p["noteLeverWorldM"])
        base = cradle_velocity + np.cross(cradle_spin, p["cradleLeverWorldM"])
        omega = np.asarray(note["w"] if rotors is None else rotors)
        tangent, normal = p["tangentWorld"], p["normalWorld"]
        relative = base - body + tangent * (self.nc["radiusM"] * omega[p["motor"]])[:, None]
        slip = relative - normal * np.sum(normal * relative, axis=1)[:, None]
        speed = np.linalg.norm(slip, axis=1)
        direction = np.divide(slip, speed[:, None], out=np.zeros_like(slip), where=speed[:, None] > 0)
        cap = self.nc["beltMu"] * p["normalN"]
        strain = np.asarray(s.get("axialShearM", [0., 0.]))
        elastic = np.zeros_like(slip)
        velocity = np.zeros(2)
        front = p["noteLeverWorldM"][:, 0] >= 0
        for group, region in enumerate((sidewall & ~front, sidewall & front)):
            loads = p["normalN"][region]
            total = loads.sum()
            force = self.stiffness * strain[group]
            if abs(force) > self.nc["beltMu"] * total + 1e-12:
                raise LiftDomainError("AXIAL_TREAD_YIELD")
            if total:
                weights = loads / total
                velocity[group] = weights @ (slip[region] @ axis)
                elastic[region] = (force * weights)[:, None] * axis

        # Intersect the kinetic traction ray with the remaining friction disk.
        along = np.sum(elastic * direction, axis=1)
        budget = np.sqrt(np.maximum(0., cap**2 - np.sum(elastic**2, axis=1) + along**2)) - along
        kinetic = direction * (budget * np.tanh(speed / self.nc["slipRegularizationMps"]))[:, None]
        friction = elastic + kinetic
        if np.any(np.linalg.norm(friction, axis=1) > cap + 1e-12):
            raise LiftDomainError("FRICTION_CONE_VIOLATION")
        forces = normal * p["normalN"][:, None] + friction
        resultant = forces.sum(axis=0)
        moment = np.cross(p["noteLeverWorldM"], forces).sum(axis=0)
        reaction_moment = np.cross(p["cradleLeverWorldM"], -forces).sum(axis=0)
        axis_traction = np.bincount(p["motor"], weights=np.sum(friction * tangent, axis=1), minlength=2)
        torque = -self.nc["radiusM"] * axis_traction
        heat = float(np.sum(kinetic * slip))
        elastic_power = self.stiffness * strain @ velocity
        normal_power = np.sum(normal * p["normalN"][:, None] * (body - base))
        work_error = (resultant @ (note_velocity - cradle_velocity) + moment @ note_spin
                      + reaction_moment @ cradle_spin + torque @ omega - normal_power + heat + elastic_power)
        if abs(work_error) > 1e-8 or heat < -1e-12:
            raise LiftDomainError("SHEAR_WORK_BUDGET")
        out_of_plane = [resultant[1], moment[0], moment[2], reaction_moment[0], reaction_moment[2]]
        if max(map(abs, out_of_plane)) > 1e-8:
            raise LiftDomainError("UNSUPPORTED_NONPLANAR_WRENCH")
        con["force"] += resultant - original["noteForceWorldN"]
        con["moment"] += moment - original["noteMomentWorldNm"]
        con["q"] = dict(noteForceWorldN=resultant.tolist(), noteMomentWorldNm=moment.tolist(),
                        cradleReactionWorldN=(-resultant).tolist(), cradleMomentWorldNm=reaction_moment.tolist(),
                        rotorContactTorqueNm=torque.tolist(), sideBeltAxisTractionN=axis_traction.tolist(),
                        slipHeatW=heat, workResidualW=float(work_error))
        con["shearRate"] = velocity
        return con

    def shifted(self, s, r, dt):
        state = super().shifted(s, r, dt)
        state["axialShearM"] = (np.asarray(s["axialShearM"]) + dt * r["contact"]["shearRate"]).tolist()
        return state

    def mechanical(self, s, con):
        shear = np.asarray(s.get("axialShearM", [0., 0.]))
        return super().mechanical(s, con) + 0.5 * self.stiffness * (shear @ shear)
