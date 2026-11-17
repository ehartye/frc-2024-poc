# Pressure-only checkpoint

`checkpoint.export(plant)` and `checkpoint.restore(record)` preserve chassis, NOTE, rotor states, elapsed time, cumulative energy ledger and initial energy without resetting contact energy. Exact source/configuration/grid/material-phase identity is required. Reconstructed potential, kinetic energy and residual must match. The digest detects corruption, not maliciously forged provenance; this is a local numerical checkpoint, not an attestation of physical validity.

Three tests cover fresh-state short split versus uninterrupted byte parity, purity/corruption and malformed/legacy/changed-law rejection. This increment does not yet establish contact-phase split parity or authorize importing old point-law checkpoints. No capture claim.
