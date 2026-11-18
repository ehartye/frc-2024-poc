"""Rigid registration of the robot experiment at an official 2024 blue wing mark."""
import math

import numpy as np


WING_MARKS_INCHES = {"inner": 161.625, "middle": 218.625, "outer": 275.625}
SUPPLY_MARKS = {**{name: (114., y) for name, y in WING_MARKS_INCHES.items()},
                "center-low": (325.625, 161.625), "center-mid": (325.625, 227.625),
                "center-high": (325.625, 293.625)}


def piece_id(mark):
    return "blue-wing-"+mark if mark in WING_MARKS_INCHES else mark


class WingFrame:
    def __init__(self, first_mark="inner", yaw_degrees=90.):
        if first_mark not in WING_MARKS_INCHES or not math.isfinite(yaw_degrees):
            raise ValueError("Invalid initial wing registration")
        self.first_mark = first_mark
        self.yaw_degrees = yaw_degrees
        angle = math.radians(yaw_degrees)
        c, s = math.cos(angle), math.sin(angle)
        if yaw_degrees == 90.:
            c, s = 0., 1.
        self.rotation = np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
        self.origin_mark = self.mark(first_mark)

    @staticmethod
    def mark(name):
        x, y = SUPPLY_MARKS[name]
        return np.array([x*.0254,y*.0254,0.])

    def to_field(self, point):
        return self.origin_mark + self.rotation @ (np.asarray(point)-[.658,0.,0.])

    def to_local(self, point):
        return self.rotation.T @ (np.asarray(point)-self.origin_mark) + [.658,0.,0.]
