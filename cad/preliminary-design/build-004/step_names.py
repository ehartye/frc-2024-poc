"""Write the release 004 layout STEP: the release 003 solids with their product names repaired.

The 232-solid layout STEP was exported with every middle dot (U+00B7) encoded twice. The first assembly
writer passed UTF-8 bytes to the kernel as single-byte characters, and each later STEP in the lineage
kept the result. This script rewrites only those name strings, using the ISO 10303-21 control directive
for U+00B7. Every other byte of the file is unchanged, so the geometry is the reviewed geometry.
"""
import hashlib
import sys
from pathlib import Path

BACKSLASH = bytes([0x5C])
DOUBLE = bytes([0xC3, 0x82, 0xC2, 0xB7])  # U+00B7 encoded twice; read back as two characters
# ISO 10303-21 control directive for U+00B7: backslash X2 backslash 00B7 backslash X0 backslash
DIRECTIVE = BACKSLASH + b"X2" + BACKSLASH + b"00B7" + BACKSLASH + b"X0" + BACKSLASH
SOURCE_SHA256 = "a29a146d8897f64e8f056fcb46a431ba234c7aecccbc3233b6834c43c703bc88"


def require(ok, message):
    if not ok:
        raise SystemExit(message)


def repair(data: bytes) -> tuple[bytes, int]:
    """Return the repaired STEP bytes and the number of lines changed."""
    require(hashlib.sha256(data).hexdigest() == SOURCE_SHA256, "Unexpected source STEP")
    changed = 0
    out = []
    for line in data.split(b"\n"):
        if DOUBLE in line:
            # Names appear only in quoted strings of PRODUCT and related records.
            require(b"'" in line, "Double-encoded text outside a string")
            line = line.replace(DOUBLE, DIRECTIVE)
            changed += 1
        out.append(line)
    fixed = b"\n".join(out)
    require(all(b < 0x80 for b in fixed), "Non-ASCII bytes remain in the STEP")
    return fixed, changed


if __name__ == "__main__":
    source, target = map(Path, sys.argv[1:3])
    fixed, changed = repair(source.read_bytes())
    target.write_bytes(fixed)
    print(f"{target.as_posix()} sha256={hashlib.sha256(fixed).hexdigest()} changedLines={changed}")
