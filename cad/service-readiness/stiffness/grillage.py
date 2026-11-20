"""Linear vertical bending/torsion grillage. Units: mm, N, radians (small slopes)."""
from dataclasses import dataclass
import math
import numpy as np

E = 68300.0
G = E / (2 * (1 + 0.33))
SCALE = 100.0  # numerical coordinates store 100 mm times each rotation


@dataclass(frozen=True)
class Section:
    inertia: float
    torsion: float

    @classmethod
    def tube(cls, width, height, wall):
        inertia = (width * height**3 - (width - 2*wall) * (height - 2*wall)**3) / 12
        median_area = (width - wall) * (height - wall)
        torsion = 4 * median_area**2 * wall / (2 * (width + height - 2*wall))
        return cls(inertia, torsion)

    @classmethod
    def bar(cls, width, height):
        thin, wide = sorted((width, height))
        torsion = wide * thin**3 * (1/3 - .21*thin/wide*(1 - (thin/wide)**4/12))
        return cls(width * height**3 / 12, torsion)

    def drilled(self, height, wall, cut_width):
        # Remove a rectangular through-slot from each flange; credit no sleeve.
        removed = cut_width * (height**3 - (height - 2*wall)**3) / 12
        return Section(self.inertia - removed, self.torsion * .5)


class Grillage:
    def __init__(self):
        self.xy = {}
        self.beams = []
        self.links = []

    def node(self, family, x, y):
        key = (family, round(x, 6), round(y, 6))
        self.xy[key] = np.array([x, y])
        return key

    def beam(self, a, b, section):
        self.beams.append((a, b, section))

    def join(self, a, b, points=(), stiffness=None):
        self.links.append((a, b, points, stiffness))

    def solve(self, loads, supports):
        keys = list(self.xy)
        count = 3 * len(keys)
        matrix = np.zeros((count, count))
        constraints = []
        force = np.zeros(count)
        def basis(node, at=None):
            rows = np.zeros((3, count))
            idx = 3 * keys.index(node)
            rows[:, idx:idx+3] = np.diag([1., 1/SCALE, 1/SCALE])
            if at is not None:
                dx, dy = np.asarray(at) - self.xy[node]
                rows[0] += dx * rows[1] + dy * rows[2]
            return rows
        for a, b, section in self.beams:
            delta = self.xy[b] - self.xy[a]
            length = np.linalg.norm(delta)
            u, v = delta / length
            left, right = basis(a), basis(b)
            # Cubic Euler-Bernoulli bending along member, Saint-Venant torsion.
            gradients = [u*end[1] + v*end[2] for end in (left, right)]
            chord = (right[0] - left[0]) / length
            curvature = np.stack([slope - chord for slope in gradients])
            matrix += E * section.inertia / length * curvature.T @ np.array([[4,2],[2,4]]) @ curvature
            twist = -v*(right[1]-left[1]) + u*(right[2]-left[2])
            matrix += G * section.torsion / length * np.outer(twist, twist)
        for a, b, points, stiffness in self.links:
            if stiffness is None:
                constraints.extend(basis(a, self.xy[b]) - basis(b))
                continue
            vertical, rotary = stiffness
            for point in points:
                dz = basis(a, point)[0] - basis(b, point)[0]
                matrix += vertical * np.outer(dz, dz)
            slopes = basis(a)[1:] - basis(b)[1:]
            matrix += rotary * slopes.T @ slopes
        support_rows = np.stack([basis(n)[0] for n in supports])
        constraints.extend(support_rows)
        for node, newtons in loads.items():
            force += newtons * basis(node)[0]
        c = np.stack(constraints)
        c /= np.linalg.norm(c, axis=1)[:, None]
        _, singular, vt = np.linalg.svd(c, full_matrices=True)
        free = vt[np.sum(singular > singular[0]*1e-11):].T
        reduced = free.T @ matrix @ free
        eig, vec = np.linalg.eigh(reduced)
        cutoff = eig[-1] * 1e-11
        if eig[0] < -cutoff:
            raise ValueError("Negative stiffness")
        dead = eig <= cutoff
        applied = vec.T @ free.T @ force
        if np.linalg.norm(applied[dead]) > 1e-7:
            return None  # the applied force excites a free mechanism
        if np.any(dead):
            q = free @ vec[:, ~dead] @ (applied[~dead] / eig[~dead])
        else:
            q = free @ np.linalg.solve(reduced, free.T @ force)
        residual = matrix @ q - force
        multipliers = np.linalg.lstsq(c.T, residual, rcond=1e-11)[0]
        assert np.linalg.norm(c @ q) < 1e-7
        assert np.linalg.norm(c.T @ multipliers - residual) < 2e-3
        assert math.isclose(q @ matrix @ q, q @ force, rel_tol=5e-6, abs_tol=1e-7)
        return ({node: float(q[3*i]) for i, node in enumerate(keys)},
                multipliers[-len(supports):].tolist())
