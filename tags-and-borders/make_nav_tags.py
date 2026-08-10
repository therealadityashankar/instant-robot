"""make_nav_tags.py — Print navigation tags, packed several to a page.

These are the same kind of plain fiducial as the shelf tag (make_shelf_tag.py): an
ArUco marker the LeKiwi's onboard camera can spot to navigate toward. Use them on the
sides of cuboidal "station" blocks (with an ArUco board on top for the pick), on
shelves, etc. — one distinct ID per station. To save paper, tags are laid out in a
grid; pick a size and it fits as many as possible per A4 page.

Default is DICT_6X6_250 (what the app already detects) with IDs 200–20N, chosen to NOT
collide with the board tags (0–63 border, 100–199 objects). A larger dictionary
(e.g. DICT_5X5_1000) is available via --dict, but the app's detector would need that
dictionary added to see them.

IMPORTANT: the black square is printed at --size mm; that must match the marker size
the app uses when it solvePnPs the tag. Print at 100% / no scaling.

Output: printables/nav_tags.pdf  (+ preview PNG of page 1).

Usage:
    uv run python make_nav_tags.py                       # IDs 200-203 @ 60 mm
    uv run python make_nav_tags.py --ids 200 201 202 203 204 205 --size 45
    uv run python make_nav_tags.py --dict DICT_5X5_1000 --ids 0 1 2 3
"""

import argparse
import os
import tempfile
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np
from PIL import Image
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas as rl_canvas

DPI = 300
A4_W_MM, A4_H_MM = 210.0, 297.0
MARGIN_MM = 6.0    # page margin
GUTTER_MM = 6.0    # white space between tags (shared quiet zone, ~3 mm each side)
LABEL_MM = 4.5     # caption strip under each tag


def mm_to_px(v, dpi=DPI):
    return int(round(v / 25.4 * dpi))


def marker_img(dict_name: str, tag_id: int, size_px: int) -> np.ndarray:
    d = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
    return cv2.cvtColor(aruco.generateImageMarker(d, tag_id, size_px, borderBits=1), cv2.COLOR_GRAY2RGB)


def build_pages(dict_name: str, ids, size_mm: float, dpi=DPI):
    a4w, a4h = mm_to_px(A4_W_MM, dpi), mm_to_px(A4_H_MM, dpi)
    margin = mm_to_px(MARGIN_MM, dpi)
    gutter = mm_to_px(GUTTER_MM, dpi)
    label = mm_to_px(LABEL_MM, dpi)
    size = mm_to_px(size_mm, dpi)
    font = cv2.FONT_HERSHEY_SIMPLEX

    cell_w = size + gutter
    cell_h = size + label + gutter
    top = margin + mm_to_px(6, dpi)  # leave room for the title
    cols = max(1, (a4w - 2 * margin + gutter) // cell_w)
    rows = max(1, (a4h - top - margin + gutter) // cell_h)
    per_page = cols * rows

    pages = []
    for start in range(0, len(ids), per_page):
        chunk = ids[start : start + per_page]
        page = np.ones((a4h, a4w, 3), dtype=np.uint8) * 255
        cv2.putText(page, f"Navigation tags  -  {dict_name}  -  {size_mm:.0f} mm  -  PRINT AT 100%, NO SCALING",
                    (margin, margin + mm_to_px(4, dpi)), font, 0.5, (30, 30, 30), 1, cv2.LINE_AA)
        for k, tag_id in enumerate(chunk):
            r, c = divmod(k, cols)
            x = margin + c * cell_w
            y = top + r * cell_h
            page[y : y + size, x : x + size] = marker_img(dict_name, tag_id, size)
            cv2.rectangle(page, (x - 1, y - 1), (x + size, y + size), (210, 210, 210), 1)
            cv2.putText(page, f"ID {tag_id}", (x, y + size + mm_to_px(5, dpi)),
                        font, 0.5, (60, 60, 60), 1, cv2.LINE_AA)

        # Scale bar (bottom-left)
        bx0, by = margin, a4h - mm_to_px(6, dpi)
        bx1 = bx0 + mm_to_px(50, dpi)
        tick = mm_to_px(2, dpi)
        cv2.line(page, (bx0, by), (bx1, by), (80, 80, 80), 2)
        cv2.line(page, (bx0, by - tick), (bx0, by + tick), (80, 80, 80), 2)
        cv2.line(page, (bx1, by - tick), (bx1, by + tick), (80, 80, 80), 2)
        cv2.putText(page, "50 mm", (bx0 + mm_to_px(3, dpi), by - mm_to_px(2, dpi)),
                    font, 0.4, (80, 80, 80), 1, cv2.LINE_AA)
        pages.append(page)
    print(f"Layout: {cols}×{rows} = {per_page} tags/page")
    return pages


def save_pdf(pages, out_path, dpi=DPI):
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    c = rl_canvas.Canvas(str(out_path), pagesize=A4)
    tmpfiles = []
    for page in pages:
        tmp = tempfile.NamedTemporaryFile(suffix=".png", delete=False)
        tmp.close()
        Image.fromarray(page).save(tmp.name, format="PNG", dpi=(dpi, dpi))
        tmpfiles.append(tmp.name)
        c.drawImage(tmp.name, 0, 0, width=210 * mm, height=297 * mm)
        c.showPage()
    c.save()
    for t in tmpfiles:
        os.unlink(t)
    print(f"PDF saved: {out_path}  ({len(pages)} page(s))")
    png = out_path.with_suffix(".png")
    cv2.imwrite(str(png), cv2.cvtColor(pages[0], cv2.COLOR_RGB2BGR))
    print(f"PNG preview (page 1): {png}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ids", type=int, nargs="+", default=[200, 201, 202, 203],
                    help="Tag IDs (default: 200 201 202 203)")
    ap.add_argument("--size", type=float, default=60.0, help="black marker side (mm)")
    ap.add_argument("--dict", type=str, default="DICT_6X6_250",
                    help="ArUco dictionary name (default: DICT_6X6_250, what the app detects)")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--out", type=str, default="printables/nav_tags.pdf")
    args = ap.parse_args()

    if not hasattr(aruco, args.dict):
        raise SystemExit(f"Unknown dictionary '{args.dict}'.")
    pages = build_pages(args.dict, args.ids, args.size, dpi=args.dpi)
    save_pdf(pages, args.out, dpi=args.dpi)
    print(f"Tags: {args.ids}  |  {args.size:.0f} mm  |  {args.dict}")


if __name__ == "__main__":
    main()
