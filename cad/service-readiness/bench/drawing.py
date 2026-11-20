"""Plot the first-party receiving plate from its drilling datums."""

from pathlib import Path
import pymupdf as fitz
from layout import WIDTH, DEPTH, CENTERS, CLINCH_HOLES, BENCH_HOLES


def drawing(output):
    doc = fitz.open()
    page = doc.new_page(width=792, height=612)
    ink = (0.13, 0.17, 0.21)
    muted = (0.34, 0.39, 0.43)
    teal = (0.04, 0.37, 0.37)
    light = (0.88, 0.94, 0.94)

    def txt(x, y, text, size=10, color=ink, font="helv"):
        page.insert_text((x, y), text, fontsize=size, fontname=font, color=color)

    def line(a, b, color=ink, width=0.7, dashes=None):
        page.draw_line(a, b, color=color, width=width, dashes=dashes)

    txt(32, 43, "Four-controller receiving plate", 22, font="hebo")
    txt(
        33,
        64,
        "R11  /  BENCH FIXTURE     Dimensions in millimetres. Dimensions govern over scale.",
        10,
        muted,
    )
    line((32, 78), (760, 78), teal, 1.2)
    scale = 1.45

    def pos(x, y):
        return (276 + x * scale, 254 - y * scale)

    for x, y in CENTERS:
        p1 = pos(x - 40.625, y + 30.29)
        p2 = pos(x + 40.625, y - 30.29)
        page.draw_rect(fitz.Rect(*p1, *p2), color=teal, fill=light, width=0.6)
    page.draw_rect(
        fitz.Rect(*pos(-WIDTH / 2, DEPTH / 2), *pos(WIDTH / 2, -DEPTH / 2)), color=ink, width=1.3
    )
    line(pos(-WIDTH / 2 - 3, 0), pos(WIDTH / 2 + 3, 0), muted, 0.5, "[4 3]")
    line(pos(0, -DEPTH / 2 - 3), pos(0, DEPTH / 2 + 3), muted, 0.5, "[4 3]")
    for holes, r in [(BENCH_HOLES, 3.3), (CLINCH_HOLES, 2.705)]:
        for x, y in holes:
            page.draw_circle(pos(x, y), r * scale, color=ink, width=0.8, fill=(1, 1, 1))
            line(pos(x - 4.4, y), pos(x + 4.4, y), muted, 0.35)
            line(pos(x, y - 4.4), pos(x, y + 4.4), muted, 0.35)
    for i, (x, y) in enumerate(CENTERS, 1):
        tx, ty = pos(x, y)
        txt(tx - 13, ty + 3, "Bay " + str(i), 9, teal)
    txt(222, 112, "304 overall", 10)
    line((56, 106), (211, 106), muted, 0.6)
    line((283, 106), (496, 106), muted, 0.6)
    txt(60, 409, "Datum at plate center. Positive X right and positive Y up.", 9, muted)
    txt(530, 110, "Stock and datums", 13, font="hebo")
    for i, t in enumerate(
        [
            "304 x 180 x 2.00 flat aluminum",
            "6061 stock; verify hardness at",
            "or below HRB 70 / HB 125.",
            "",
            "Bay centers (X, Y)",
            "1   (-78, -41)     2   (-78, +41)",
            "3   (+78, -41)     4   (+78, +41)",
            "",
            "Three holes per bay, offsets",
            "(-12.7, +25.4)   (+12.7, +25.4)",
            "(0, -25.4)",
            "",
            "12x diameter 5.41, +0.08/-0",
            "4x diameter 6.60 through",
            "at ( +/-140, +/-78 ).",
        ]
    ):
        txt(530, 135 + i * 16, t, 10)
    line((32, 428), (760, 428), teal, 1)
    txt(32, 453, "Captive bracket fasteners", 13, font="hebo")
    txt(
        32,
        475,
        "Press one PEM CLS-M4-2 from below each 5.41 hole. Check seating and retention on a stock coupon.",
        10,
    )
    txt(
        32,
        494,
        "Use the three M4 x 8 socket screws at each received bracket. Its nominal flange is 2.5 thick.",
        10,
    )
    txt(
        32,
        513,
        "Provide clearance below the plate for the screw tips. The nominal tips reach 3.5 below the lower face.",
        10,
    )
    txt(
        32,
        532,
        "The shaded rectangles show measured supplier package extents. The four large holes attach to a bench fixture.",
        10,
    )
    line((32, 555), (760, 555), muted, 0.6)
    txt(
        32,
        577,
        "Source geometry and nominal fits are recorded in bench-fit.json. Inspect the received parts before pressing hardware.",
        9,
        muted,
    )
    doc.set_metadata(
        {"title": "R11 four-controller bench receiving plate", "author": "FRC proof of concept"}
    )
    doc.save(output, deflate=True, no_new_id=True)
    doc.close()


if __name__ == "__main__":
    drawing(Path(__file__).with_name("bench-plate.pdf"))
