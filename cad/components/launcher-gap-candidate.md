# Wider launcher gap candidate

The full connected transfer study found actual bare-wheel contact at six intermediate NOTE positions, not merely loose shell-envelope overlap. This study preserves the 46.8 mm baseline and tests a 49.2 mm finished-tread gap without modifying the integrated assembly.

At the nominal source geometry, the original gap demands 2.0 mm per-side indentation while the flange lies only 1.27 mm below the finished tread. The wider candidate reduces nominal indentation to 0.8 mm. Tolerance-inclusive optimal native bounds give 0.4699999 mm separation between each bare wheel and the aligned rigid NOTE's entire thickness slab. That plane bound covers aligned translation, not only the sampled positions; it does not cover yaw, roll, deflection or real manufacturing tolerances.

The nine sampled positions include every one of the six baseline native contacts. All four candidate wheels have zero bare-wheel common volume at those positions; nominal tread overlap remains, peaking around 70 cubic millimeters per wheel. The baseline reproduces the prior maximum approximately 6.323 cubic millimeters of bare-wheel overlap. Tread indentation is not a measured force or friction law.

Recommendation: adopt the wider gap as the next adjustable-geometry hypothesis after checking the shifted full cartridges and mounts. Re-extract contact windows and rerun the shared six-drive model; existing speeds and preload assumptions do not transfer. Smaller squeeze may reduce available traction, and the 0.47 mm nominal flange margin must be checked against measured new/worn NOTE dimensions and assembly variation. The former overlap is not itself a safety finding; an explicit foam/flange contact model would be another defensible option, but would add uncalibrated contact surfaces.

Run `python cad/components/check_launcher_gap_candidate.py --vendor-cache <private-vendor-cache>`. The source-bound receipt is `launcher-gap-candidate.json`. Supplier geometry is private; only scalar evidence is distributed. Current assembly remains unchanged until deliberate integration.
