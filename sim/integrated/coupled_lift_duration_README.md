# Longer finite-command domain study

Same actual indexed checkpoint, eight-motor bus, explicit floor extension and original normal-force budget. Seven predeclared commands are attempted for100 ms at1 ms; only the three observed finite-patch failures are refined at0.5 ms. This is not whole-matrix convergence or a successful lift. No whole-robot collision, mechanism-floor or drivetrain load-transfer acceptance is supplied.

The diagnostic subclass delegates all rate equations unchanged and adds the exact rejected pure-evaluation input to an exception. A rejected voltage-scan or endpoint trial is not an accepted state or a chronologically resolved collision. All failures leave28/30 nominal witnesses inside the NOTE; they are partial registration losses, NOT full release. Guide separation is independently certified.

| Case | Step ms | Last accepted ms | Rejected trial ms | Remaining witnesses | Max new energy error microjoule |
|---|---:|---:|---:|---:|---:|
| both-plus-003 | 1 | 73.000 | 74.000 | 28 | 15.176 |
| coast | 1 | 37.000 | 37.500 | 28 | 2.777 |
| brake | 1 | 61.000 | 62.000 | 28 | 28.214 |
| both-plus-003 | 0.5 | 73.500 | 74.000 | 28 | 3.722 |
| coast | 0.5 | 37.000 | 37.250 | 28 | 0.849 |
| brake | 0.5 | 61.500 | 61.750 | 28 | 7.089 |

Lost indices14/29 are each side's +X,+Z witness corner for the positive offset. Coast/brake lose indices12/27 at +X,-Z. All signed torus implicit values, coordinates, motor identities and guide queries are retained in the report. No pressure is redistributed to the remaining points; the step stops before model support is lost.

Run with a fresh path:
```text
python sim/integrated/coupled_lift_duration.py artifacts/fresh100.json .1 .001
python sim/integrated/coupled_lift_event_refine.py artifacts/fresh-events.json .1 .0005
python -m unittest discover -s tests -p test_coupled_lift*.py -v
```

The next required extension is geometry-dependent partial-area contact/compliance with reciprocal work and explicit stored-energy handling, not removal of the validity check. Actual cassette spring force is not automatically NOTE force; cassette inertia, travel, stops/opening work and initial spring energy must be modeled before adopting a stronger grip.
