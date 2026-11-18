# Finite command-envelope experiment

This is a short continuation from the indexed checkpoint, not an achieved lift mission. Seven commands are declared before integration: static gravity feedforward, +0.01 lift duty, +0.01 pitch duty, +0.01 both, +0.03 both, actuator open-circuit coast, and modeled short-circuit brake. Drive and pinch demands remain the checkpoint values. No desired pose is imposed and the approximately 1.755 N per-side normal budget is unchanged.

`coupled_lift_sweep.py` uses the original reviewed phase kernel. `coupled_lift_separated_sweep.py` uses an explicit experiment fork whose only implementation difference is the guide-query import. This is intentionally not a general framework. The adapter preserves supported original results and archived Boolean disagreement. Only an original UNKNOWN whose exact torus AABB is disjoint from the enclosing guide AABB obtains zero guide force. Overlapping bounds remain UNKNOWN. Native-material UNKNOWN provenance remains attached even to a separation certificate.

Commands (fresh output paths required):

```text
python sim/integrated/coupled_lift_sweep.py artifacts/baseline.json .01 .001
python sim/integrated/coupled_lift_separated_sweep.py artifacts/separated.json .025 .001
python sim/integrated/coupled_lift_separated_sweep.py artifacts/separated-half.json .025 .0005
python -m unittest discover -s tests -p test_coupled_lift*.py -v
python -m unittest discover -s tests -p test_guide_separation.py -v
```

Reports bind source inputs before/after execution, preserve full accepted state/energy traces, distinguish inherited energy residual from new phase error, and record the last accepted state on atomic domain rejection. End-sample actuator currents are diagnostics at the accepted end state and bus, NOT the midpoint currents used for work integration. Modeled brake is not a hardware braking guarantee. Relative NOTE/cradle motion is reported directly; horizon completion is not retention, acquisition, scoring, or mechanical acceptance.

The original baseline demonstrates why a negative-pitch API boundary must not be called a collision: lift-only, coast and brake stop after one coarse step, although the separate conservative geometry certificate proves separation at those states. Higher positive duties can lift the cradle while the NOTE still falls. No command is selected as a successful strategy.

All current scripts use LF bytes for portable raw-SHA receipts. Early local CRLF baseline receipts were rerun after normalization; historical source snapshots remain historical, not relabeled as fresh executions.
