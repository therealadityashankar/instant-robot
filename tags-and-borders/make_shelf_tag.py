"""make_shelf_tag.py — Print the onboard-camera shelf navigation tag.

A single plain ArUco marker (DICT_6X6_250, ID 200) to stick on the bottom of the
shelf. The LeKiwi's onboard camera detects it and the app drives the base to a
standoff in front of it ("Drive to tag" in the Base-wheels panel).

ID 200 is chosen to NOT collide with the board border tags (IDs 0–63) or the
bordered object/block tags (IDs 100–199). The marker is plain (no L-border) — the
app localises it with solvePnP directly on the ArUco corners.

IMPORTANT: the black marker square is printed at --size mm (default 80). This MUST
match SHELF_TAG_MM in app/src/Simulator.svelte for the distance readout to be metric.
Print at 100% / no scaling.

Output: printables/shelf_tag.pdf  (+ shelf_tag.png preview)

Usage:
    uv run python make_shelf_tag.py
    uv run python make_shelf_tag.py --id 200 --size 80
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

# Must match the detector dictionary in app/src/lib/homography.ts (DICT_6X6_250).
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
DPI = 300
A4_W_MM, A4_H_MM = 210.0, 297.0
QUIET_MM = 12.0  # white quiet zone around the black marker (not part of the metric size)


def mm_to_px(v, dpi=DPI):
    return int(round(v / 25.4 * dpi))


def make_marker(tag_id: int, size_mm: float, dpi=DPI) -> np.ndarray:
    size_px = mm_to_px(size_mm, dpi)
    img = aruco.generateImageMarker(ARUCO_DICT, tag_id, size_px, borderBits=1)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)


def build_page(tag_id: int, size_mm: float, dpi=DPI) -> np.ndarray:
    a4w, a4h = mm_to_px(A4_W_MM, dpi), mm_to_px(A4_H_MM, dpi)
    page = np.ones((a4h, a4w, 3), dtype=np.uint8) * 255
    font = cv2.FONT_HERSHEY_SIMPLEX

    marker = make_marker(tag_id, size_mm, dpi)
    mpx = marker.shape[0]
    quiet = mm_to_px(QUIET_MM, dpi)
    x = (a4w - mpx) // 2
    y = mm_to_px(45, dpi)
    page[y : y + mpx, x : x + mpx] = marker

    # Quiet-zone guide box (light grey) so you keep white space around the marker.
    cv2.rectangle(page, (x - quiet, y - quiet), (x + mpx + quiet, y + mpx + quiet),
                  (200, 200, 200), 1)

    cv2.putText(page, f"Shelf navigation tag  -  DICT_6X6_250  ID {tag_id}",
                (mm_to_px(15, dpi), mm_to_px(20, dpi)), font, 0.6, (20, 20, 20), 1, cv2.LINE_AA)
    cv2.putText(page, f"Black square = {size_mm:.0f} mm  (must match SHELF_TAG_MM).  "
                      f"Stick on the shelf bottom, keep the white border clear.",
                (mm_to_px(15, dpi), mm_to_px(28, dpi)), font, 0.4, (80, 80, 80), 1, cv2.LINE_AA)

    # Scale bar
    bx0 = mm_to_px(15, dpi)
    bx1 = bx0 + mm_to_px(50, dpi)
    by = a4h - mm_to_px(12, dpi)
    tick = mm_to_px(2, dpi)
    cv2.line(page, (bx0, by), (bx1, by), (80, 80, 80), 2)
    cv2.line(page, (bx0, by - tick), (bx0, by + tick), (80, 80, 80), 2)
    cv2.line(page, (bx1, by - tick), (bx1, by + tick), (80, 80, 80), 2)
    cv2.putText(page, "50 mm  |  PRINT AT 100%  NO SCALING",
                (bx0 + mm_to_px(3, dpi), by - mm_to_px(2, dpi)), font, 0.4, (80, 80, 80), 1, cv2.LINE_AA)
    return page


def save_pdf(page, out_path, dpi=DPI):
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    pil = Image.fromarray(page)
    tmp = tempfile.NamedTemporaryFile(suffix=".png", delete=False)
    tmp.close()
    pil.save(tmp.name, format="PNG", dpi=(dpi, dpi))
    c = rl_canvas.Canvas(str(out_path), pagesize=A4)
    c.drawImage(tmp.name, 0, 0, width=210 * mm, height=297 * mm)
    c.save()
    os.unlink(tmp.name)
    print(f"PDF saved: {out_path}")
    png = out_path.with_suffix(".png")
    cv2.imwrite(str(png), cv2.cvtColor(page, cv2.COLOR_RGB2BGR))
    print(f"PNG saved: {png}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=200)
    ap.add_argument("--size", type=float, default=80.0, help="black marker side (mm)")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--out", type=str, default="printables/shelf_tag.pdf")
    args = ap.parse_args()
    page = build_page(args.id, args.size, dpi=args.dpi)
    save_pdf(page, args.out, dpi=args.dpi)


if __name__ == "__main__":
    main()
