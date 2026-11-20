"""R13 service gauge and the rail openings at the parked R9 assembly datum (mm)."""

STATIONS = [(x, y) for y in (-329, 329) for x in (23, 53)]
BOTTOM, TOP = 69.85, 120.65
BORE, SLEEVE_OD = 8.5, 13.0
SHAFT, LENGTH = 8.0, 54.0
GRIP, GRIP_LENGTH = 12.0, 14.0
OFFSET = 0.4
PARENT = "4663d9fa55b5f3eb8e23549f3205a2d29248322543beb3e7a7d8076a23f93f6d"
MAST = {"R8 fixed climb mast", "R8 sliding saddle", "R8 transverse lock"}
ATTACHMENT = "R9 mast attachment"


def retired(component, name):
    return component in MAST or (
        component == ATTACHMENT and not name.startswith("Rail flush weld sleeve ")
    )


def station_label(x, y):
    return f"Rail flush weld sleeve {x} {y}"
