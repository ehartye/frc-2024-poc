"""Bench coordinates in millimetres. Z=0 is the bracket seating face."""

import itertools

WIDTH, DEPTH, THICKNESS = 304.0, 180.0, 2.0
CENTERS = tuple(itertools.product((-78.0, 78.0), (-41.0, 41.0)))
BRACKET_HOLES = ((-12.7, 25.4), (12.7, 25.4), (0.0, -25.4))
CLINCH_HOLES = tuple((x + u, y + v) for x, y in CENTERS for u, v in BRACKET_HOLES)
BENCH_HOLES = tuple(itertools.product((-140.0, 140.0), (-78.0, 78.0)))
BATTERY_BAYS = (
    (-292.0, -280.0, 377.115, -218.0, -200.0, 435.115),
    (-182.0, -280.0, 377.115, -108.0, -200.0, 435.115),
)
CASE_SWEEP = (-291.5, -279.0, 208.115, -108.5, -201.0, 1060.0)

# This crossover volume keeps disconnected dressing clear of the center strap.
CROSSOVER = (-218.0, -270.0, 390.115, -182.0, -246.0, 435.115)
