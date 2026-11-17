# Detachable slide-pad attachments

Additive candidate on the pinned upper/lower carriage assembly. `slide_pad_attachments.apply(parts, note_center_z_mm, config)` returns modified parts and joint records. Input is `upper_carriage_connections.apply(lower_support_mounts.apply(base)[0], z)[0]`. The eight pad positions and guide positions are unchanged.

## Original geometry and load path

Each 20 x 20 x 6 mm UHMW pad receives two recessed #6 screws into a 4 mm backing. A single-piece machined L bracket bridges the actual 10 mm lateral gap to the carriage. Its 3.175 mm foot is held by two #10 buttonhead screws, with nuts outside the carriage. The 24 mm bracket height provides material around the two carriage bores. This is a proposed pad -> bracket -> bolted carriage load path, not a verified strength or creep result. Original bracket material is provisionally 6061 aluminum; the CAD does not establish temper, allowable loads or bolt preload.

## Exact selected hardware and dimensional proxies

- 16 AndyMark am-1570, #6-32 x 0.625 inch flat-head socket screws: https://andymark.com/products/6-32-x-0-625-in-flat-head-socket-cap-screw . Supplier specifies alloy steel, zinc finish and 5/64 inch drive.
- 16 am-1419 #6-32 nylon locking jam nuts: https://andymark.com/products/6-32-nylock-jam-nut . Supplier thickness 0.140 inch, 5/16 inch wrench size.
- 16 am-1524 #10-32 x 0.75 inch buttonhead screws: https://andymark.com/products/10-32-x-0-75-in-hex-drive-button-head-cap-screw . Existing #10 nominal stack uses am-1042 nuts (0.249 inch thick, 3/8 inch wrench size).
- Generic standard head dimensions are from Pencom Screw Book REV004, printed pages58/59: https://www.pencomsf.com/public/product-fliers/screw_book.pdf . For #6 flathead: 82 degree cone, maximum diameter 0.307 inch, 0.078 inch socket; not a measurement or supplier certification of am-1570. Selected screw length includes head. Standard #10 buttonhead proxy is reused from existing stack.

Sources read 2026-11-16. Web-fetched specifications have no claimed local byte hash. No supplier CAD is included. Fasteners are dimensional original representations with unmodeled threads, nylon and head transitions; the bolt/nut thread zone is not simulated.

## Wear, assembly and servicing

The flat head is initially 0.5 mm below the guide-facing UHMW surface. This gives only **0.5 mm nominal wear before head exposure**, not a qualified maintenance interval. With the conservative standard maximum head diameter, the countersink leaves approximately 3 mm UHMW beneath its deepest point. The original guide/pad gap remains 0.15 mm, with no pretended preload.

Assemble the pad screws before inserting the guide. The 0.15 mm running gap does not provide driver access: do not describe the screws as in-place serviceable. Remove the carriage/bracket assembly for pad replacement. Nut-side wrench approach, removal sweep under full wiring and fastener preload remain to be qualified. Countersinking compressible UHMW introduces creep and pull-through uncertainties; do not torque it as a qualified metal joint.

## Verification

Run `python -m unittest discover -s tests -p test_slide_pad_attachments.py -v` and `python cad/assembly/slide_pad_attachments.py artifacts/pad-attachments.json`. The latter screens new material against the supplied upper/lower assembly at 15 sampled poses and binds source hashes before/after. Zero new overlaps would not clear unrelated existing assembly conflicts, prove continuous clearance or qualify loads. An over-wear control and 0.2 mm inward pad shift preserve meaningful exposure/interference failures.
