# Conservative guide separation

`guide_separation.query` adds a separation-only broadphase for the original, unadopted rounded wear guide. It does not alter `finite_retention_guide` or any historical receipt. Robot-frame SI inputs include the full unit torus symmetry axis. The exact axis-aligned torus half-extent is `R*sqrt(1-axis_i^2)+r`; the enclosing guide box follows its authored box/cylinder construction. Holes only remove guide material.

Strict disjointness beyond the explicit 1 nm numerical margin proves separation from this guide representation, not from the backing, chassis or other parts. Overlapping/touching bounds remain UNKNOWN and must use supported contact geometry or stop. The known 20-degree native Boolean/material-witness disagreement is not cleared. Native-material UNKNOWN provenance remains present even on separation results.

The indexed NOTE has a 7.7 mm vertical box gap at zero pitch; small negative pitch can remain separated without extrapolating a positive-pitch contact law. This addresses a possible finite-domain stop, not retention or physical calibration. The current eight-motor pilot remains unchanged until separate integration and fresh replay.

Four tests cover negative pitch, known-disagreement rejection, dense independently parameterized torus surface containment, malformed inputs and touching bounds. The query is not yet connected to the phase runner.
