# Handoff transfer: preserve the blocked path

The frozen f8a35ed assembly is checked with the cradle held at its handoff pose while the full rigid NOTE advances along the60degree shooter direction. This is different from moving a mechanism while its held piece stays fixed relative to it.

49poses at10mm spacing expose unintended intersections. Both authored cradle side weldments intersect at110..180mm advance, peaking at222.743mm3 combined at150mm. Front pulley representations also intersect; front bearing representations intersect at140/150mm. These must not be renamed intended contact to make a simulation pass.

Full-radius launcher shell representations also intersect at140..300mm. Their bounding46.99mm radius may overbound the actual45.72mm tread seat profile, so this category requires separate private native-wheel witnesses. It is not yet proof of a real metal/NOTE clash.

No unintended hits appear in the sampled0..100mm and310..480mm ranges. Only side belts and launcher tread are tagged as intended-contact hypotheses; even these need deformation, preload and friction evidence. This is finite sampling against original representations, not continuous clearance, field/rule acceptance or a complete physical model.

`transfer_screen.py` and `transfer-evidence.json` bind actual source bytes. Input shapes were valid solids; intersections sum absolute volumes per resulting solid rather than allowing signed cancellation. No vendor CAD is included.

The powered upstream-feeder experiment is blocked by this geometry result. The launcher-only1.5m/s reversal remains a useful omitted-drive control, but replacing it with a higher prescribed speed is not an integrated mechanism solution. Revise the actual carrier/transfer geometry, repeat this screen, then couple pinch and launcher motors on shared power.

## Native wheel witnesses

Private hash-pinned AM-3608 source CAD, validated as one solid, was separately placed with the same wheel-center/tread-offset transform. All four bare wheels have zero intersection at advances140,220,300mm (12 checks). These three witnesses distinguish the conservative cylindrical shell representation from the source profile at those poses; they do not prove continuous wheel clearance or resolve cradle collisions. The source geometry is not redistributed. `native-wheel-witnesses.json` records scalar results and source/script pins.

## Material intersection update

The current checker intersects individual valid material solids and sums absolute intersection volumes. A compound-level empty Boolean is not clearance evidence. The previous archived geometry/transfer receipts retain their prior source hashes and do not validate this revised checker; regenerate against the forthcoming corrected connected assembly before running the six-drive case. Three small native regression tests cover compound aggregation, empty-material rejection and separated solids.
