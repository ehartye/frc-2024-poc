"""Install an R7 mass-budget case in the planar swerve model at its CAD floor pose."""
import numpy as np

from run_axial_shear_lift import ROOT, digest, load


BUDGET = ROOT / "cad/drivetrain/r7-floor-mass.json"
LAYOUT = ROOT / "cad/drivetrain/maxswerve-layout.json"


def binding(case):
    if case is None:
        return {}
    data = load(BUDGET)
    if case not in data["cases"]:
        raise ValueError("Unknown R7 mass case")
    return dict(r7MassCase=case, massBudgetSha256=digest(BUDGET))


def verify_binding(receipt):
    expected = binding(receipt.get("r7MassCase"))
    archive = load(BUDGET)["archive"] if expected else load(LAYOUT)["privateFusionArchiveSha256"]
    actual = {k: receipt[k] for k in ("r7MassCase", "massBudgetSha256") if k in receipt}
    if actual != expected or receipt["nativeSha256"] != archive:
        raise ValueError("Mass case and native archive differ from registered dynamics")
    return expected


def shift(r):
    return np.eye(3)*(r@r)-np.outer(r, r)


def install(chassis, case):
    groups = load(BUDGET)["cases"][case]
    # Carriage and cradle share the same zero-position pivot. All budget centers
    # and inertia tensors are expressed in the CAD floor frame at that pose.
    carriage = chassis.find("body[@name='carriage']")
    cradle = carriage.find("body[@name='cradle']")
    pivot = np.fromstring(carriage.get("pos"), sep=" ")
    for node, group, origin in ((chassis, "fixed", np.zeros(3)), (carriage, "carriage", pivot),
                                (cradle, "cradle", pivot)):
        row = groups[group]
        mass = row["massKg"]
        center = np.array(row["centerM"])
        first = mass*center
        tensor = np.diag(row["diagonalAtCenterKgM2"])+mass*shift(center)
        upper = np.triu_indices(3, 1)
        tensor[upper] += row["productsAtCenterKgM2"]
        tensor[upper[::-1]] += row["productsAtCenterKgM2"]
        if group == "fixed":
            # Remove explicitly articulated rotor, fork and tread bodies from
            # the fixed parcel. Both first and second moments must be conserved.
            for child in chassis.findall("body"):
                if not child.get("name").startswith(("launcher_", "fork_")):
                    continue
                position = np.fromstring(child.get("pos"), sep=" ")
                for item in child.iter("inertial"):
                    m = float(item.get("mass"))
                    local = np.fromstring(item.get("diaginertia"), sep=" ")
                    # Fork axes are at quarter turns relative to robot axes.
                    quat = np.fromstring(child.get("quat", "1 0 0 0"), sep=" ")
                    theta = 2*np.arctan2(quat[3], quat[0])
                    c,s = np.cos(theta), np.sin(theta)
                    rot = np.array([[c,-s,0],[s,c,0],[0,0,1]])
                    tensor -= rot@np.diag(local)@rot.T+m*shift(position)
                    first -= m*position
                    mass -= m
        center = first/mass
        tensor -= mass*shift(center)
        principal = np.linalg.eigvalsh(tensor)
        if mass <= 0 or principal[0] <= 0 or principal[2] > principal[:2].sum():
            raise ValueError("Unphysical R7 inertia after articulated mass partition")
        inertial = node.find("inertial")
        inertial.attrib.pop("diaginertia")
        inertial.set("mass", str(mass))
        inertial.set("pos", " ".join(map(str, center-origin)))
        inertial.set("fullinertia", " ".join(map(str, [*tensor.diagonal(), *tensor[upper]])))
