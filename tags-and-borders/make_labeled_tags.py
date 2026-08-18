"""make_labeled_tags.py — Generate labeled navigation tags for Instant Robot.

Each tag card has ArUco markers 200 and 201 on the LEFT and RIGHT, with a text
region in between. The top half of the text region is the LABEL (station
identity), the bottom half is a DESCRIPTION (extra context for Gemini AI).

    ┌──────────┬────────────────────┬──────────┐
    │          │                    │          │
    │  ArUco   │      LABEL        │  ArUco   │
    │   200    │                   │   201    │
    │          │────────────────────│          │
    │          │   description     │          │
    │          │     text          │          │
    └──────────┴────────────────────┴──────────┘

The app detects both markers, uses their known positions to compute a
homography for the text region between them, warps it flat, and reads it
with Tesseract.js (or sends the crop to Gemini).

IMPORTANT: print at 100% / no scaling, so the marker sizes match solvePnP.

Output: printables/nav_tags.pdf + printables/nav_tags.png (page-1 preview).

Usage:
    # Default set of labels
    uv run python make_labeled_tags.py

    # Custom labels  (label:description pairs)
    uv run python make_labeled_tags.py \\
        --tags "Apple:Red fruit on table" "Banana:Yellow curved fruit" \\
               "Basket:Drop zone for items" "Orange:Round citrus fruit"

    # Bigger marker
    uv run python make_labeled_tags.py --size 50
"""

import argparse
import os
import tempfile
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas as rl_canvas

DPI = 300
A4_W_MM, A4_H_MM = 210.0, 297.0
PAGE_MARGIN_MM = 10.0
GUTTER_MM = 10.0  # space between cards on the page

# The two marker IDs — always the same pair on every card.
LEFT_ID = 200
RIGHT_ID = 201

# Card geometry relative to marker size
QUIET_MM = 3.0        # quiet-zone padding around each marker
TEXT_GAP_MM = 4.0      # gap between marker edge and text region
BORDER_MM = 0.6        # card outline thickness
CARD_PAD_MM = 3.0      # padding inside card border


def mm_to_px(v, dpi=DPI):
    return int(round(v / 25.4 * dpi))


def marker_img(tag_id: int, size_px: int, dict_name: str = "DICT_6X6_250") -> np.ndarray:
    d = aruco.getPredefinedDictionary(getattr(aruco, dict_name))
    return cv2.cvtColor(
        aruco.generateImageMarker(d, tag_id, size_px, borderBits=1),
        cv2.COLOR_GRAY2RGB,
    )


def _try_load_font(size: int, bold: bool = False):
    """Try to load a clean sans-serif font; fall back to PIL default."""
    if bold:
        candidates = [
            "/System/Library/Fonts/Helvetica.ttc",
            "/Library/Fonts/Arial Bold.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
            "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
            "C:/Windows/Fonts/arialbd.ttf",
        ]
    else:
        candidates = [
            "/System/Library/Fonts/Helvetica.ttc",
            "/Library/Fonts/Arial.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
            "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
            "C:/Windows/Fonts/arial.ttf",
        ]
    for path in candidates:
        if os.path.isfile(path):
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                continue
    return ImageFont.load_default()


def wrap_text_to_lines(draw: ImageDraw.Draw, text: str, font, max_width: int) -> list[str]:
    """Wrap text into lines that fit within max_width."""
    words = text.split()
    if not words:
        return [""]
    lines = []
    curr = []
    for w in words:
        test_line = " ".join(curr + [w])
        bbox = draw.textbbox((0, 0), test_line, font=font)
        if bbox[2] - bbox[0] <= max_width or not curr:
            curr.append(w)
        else:
            lines.append(" ".join(curr))
            curr = [w]
    if curr:
        lines.append(" ".join(curr))
    return lines


def render_text_block(text: str, width_px: int, height_px: int, font_size: int,
                      bold: bool = False, bg: tuple = (255, 255, 255),
                      fg: tuple = (0, 0, 0), max_lines: int = 3) -> np.ndarray:
    """Render multi-line text centred in a region of the given pixel size."""
    img = Image.new("RGB", (width_px, height_px), bg)
    draw = ImageDraw.Draw(img)
    pad = 8

    # Find the largest font size where the wrapped lines fit both width and height
    while font_size >= 9:
        font = _try_load_font(font_size, bold)
        lines = wrap_text_to_lines(draw, text, font, width_px - 2 * pad)
        if len(lines) <= max_lines:
            total_h = 0
            for line in lines:
                bbox = draw.textbbox((0, 0), line, font=font)
                lh = max(bbox[3] - bbox[1], int(font_size * 1.15))
                total_h += lh
            if total_h <= height_px - 2 * pad:
                break
        font_size -= 1

    font = _try_load_font(font_size, bold)
    lines = wrap_text_to_lines(draw, text, font, width_px - 2 * pad)

    line_bboxes = [draw.textbbox((0, 0), line, font=font) for line in lines]
    line_hs = [max(b[3] - b[1], int(font_size * 1.15)) for b in line_bboxes]
    total_h = sum(line_hs)
    y_curr = max(pad, (height_px - total_h) // 2)

    for line, lh in zip(lines, line_hs):
        bbox = draw.textbbox((0, 0), line, font=font)
        lw = bbox[2] - bbox[0]
        x = (width_px - lw) // 2
        draw.text((x, y_curr), line, fill=fg, font=font)
        y_curr += lh

    return np.array(img)


def build_card(label: str, description: str, marker_mm: float,
               dict_name: str = "DICT_6X6_250", dpi: int = DPI) -> np.ndarray:
    """Build a single labeled nav-tag card (RGB, at `dpi`).

    Layout:
        [pad][quiet][marker200][quiet][gap][TEXT REGION][gap][quiet][marker201][quiet][pad]
    """
    marker_px = mm_to_px(marker_mm, dpi)
    quiet_px = mm_to_px(QUIET_MM, dpi)
    gap_px = mm_to_px(TEXT_GAP_MM, dpi)
    pad_px = mm_to_px(CARD_PAD_MM, dpi)
    border_px = max(1, mm_to_px(BORDER_MM, dpi))

    # Text region width: at least as wide as a marker, looks good at ~1.2x marker
    text_w = max(marker_px, int(marker_px * 1.3))

    # Card dimensions
    card_w = (pad_px + quiet_px + marker_px + quiet_px + gap_px) * 2 + text_w
    card_h = pad_px + quiet_px + marker_px + quiet_px + pad_px

    card = np.ones((card_h, card_w, 3), dtype=np.uint8) * 255

    # ── Left marker (ID 200) ──
    mk_left = marker_img(LEFT_ID, marker_px, dict_name)
    lx = pad_px + quiet_px
    my = pad_px + quiet_px
    card[my:my + marker_px, lx:lx + marker_px] = mk_left

    # ── Right marker (ID 201) ──
    mk_right = marker_img(RIGHT_ID, marker_px, dict_name)
    rx = card_w - pad_px - quiet_px - marker_px
    card[my:my + marker_px, rx:rx + marker_px] = mk_right

    # ── Text region (between the two markers) ──
    text_x = lx + marker_px + quiet_px + gap_px
    text_total_w = rx - quiet_px - gap_px - text_x
    if text_total_w < 10:
        text_total_w = text_w  # fallback
    text_h = marker_px  # same height as the markers

    # Split into label (top half) and description (bottom half)
    label_h = text_h // 2
    desc_h = text_h - label_h

    # Render label (bold, large, uppercase)
    label_font_size = max(14, int(marker_px * 0.30))
    label_img = render_text_block(
        label.upper(), text_total_w, label_h, label_font_size,
        bold=True, fg=(0, 0, 0),
    )
    card[my:my + label_h, text_x:text_x + text_total_w] = label_img

    # Separator line between label and description
    sep_y = my + label_h
    cv2.line(card, (text_x, sep_y), (text_x + text_total_w, sep_y),
             (180, 180, 180), max(1, dpi // 300))

    # Render description (smaller, grey-on-light-grey)
    desc_font_size = max(10, int(marker_px * 0.14))
    desc_img = render_text_block(
        description, text_total_w, desc_h, desc_font_size,
        bold=False, bg=(248, 248, 248), fg=(60, 60, 60),
    )
    card[sep_y + 1:my + text_h, text_x:text_x + text_total_w] = desc_img[:text_h - label_h - 1]

    # ── Card border ──
    cv2.rectangle(card, (0, 0), (card_w - 1, card_h - 1), (120, 120, 120), border_px)

    # Light vertical separator lines between markers and text
    cv2.line(card, (text_x - gap_px // 2, my), (text_x - gap_px // 2, my + marker_px),
             (210, 210, 210), 1)
    cv2.line(card, (text_x + text_total_w + gap_px // 2, my),
             (text_x + text_total_w + gap_px // 2, my + marker_px),
             (210, 210, 210), 1)

    return card


def build_pages(tags: list[tuple[str, str]], marker_mm: float,
                dict_name: str, dpi: int = DPI) -> list[np.ndarray]:
    """Lay out cards on A4 pages."""
    a4w, a4h = mm_to_px(A4_W_MM, dpi), mm_to_px(A4_H_MM, dpi)
    margin = mm_to_px(PAGE_MARGIN_MM, dpi)
    gutter = mm_to_px(GUTTER_MM, dpi)
    title_h = mm_to_px(8, dpi)

    # Build a sample card to measure
    sample = build_card("X", "x", marker_mm, dict_name, dpi)
    cw, ch = sample.shape[1], sample.shape[0]

    cols = max(1, (a4w - 2 * margin + gutter) // (cw + gutter))
    rows = max(1, (a4h - margin - title_h - margin + gutter) // (ch + gutter))
    per_page = cols * rows

    pages = []
    font = cv2.FONT_HERSHEY_SIMPLEX
    for start in range(0, len(tags), per_page):
        chunk = tags[start:start + per_page]
        page = np.ones((a4h, a4w, 3), dtype=np.uint8) * 255

        # Title
        cv2.putText(
            page,
            f"Labeled Nav Tags  -  {dict_name}  -  marker {marker_mm:.0f} mm  -  "
            f"IDs {LEFT_ID} & {RIGHT_ID}  -  PRINT AT 100%, NO SCALING",
            (margin, margin + mm_to_px(4, dpi)),
            font, 0.40, (30, 30, 30), 1, cv2.LINE_AA,
        )

        y_start = margin + title_h
        for k, (label, desc) in enumerate(chunk):
            card = build_card(label, desc, marker_mm, dict_name, dpi)
            r, c = divmod(k, cols)
            x = margin + c * (cw + gutter)
            y = y_start + r * (ch + gutter)
            if y + ch > a4h - margin:
                break
            h_fit = min(ch, a4h - y)
            w_fit = min(cw, a4w - x)
            card_crop = card[:h_fit, :w_fit]
            page[y:y + card_crop.shape[0], x:x + card_crop.shape[1]] = card_crop

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

    print(f"Layout: {cols}x{rows} = {per_page} cards/page, {len(pages)} page(s)")
    return pages


def save_pdf(pages: list[np.ndarray], out_path: str, dpi: int = DPI):
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


# ── Default tags ───────────────────────────────────────────────────────────────
DEFAULT_TAGS = [
    ("Apple",   "Red fruit, pick it up"),
    ("Banana",  "Yellow curved fruit"),
    ("Orange",  "Round citrus fruit"),
    ("Basket",  "Drop zone for items"),
    ("Block",   "Wooden jenga block"),
    ("Bottle",  "Water bottle, side tag"),
    ("Plant",   "Potted plant, water me"),
]


def parse_tag_spec(spec: str) -> tuple[str, str]:
    """Parse 'Label:Description' or just 'Label'."""
    if ":" in spec:
        label, desc = spec.split(":", 1)
        return label.strip(), desc.strip()
    return spec.strip(), ""


def main():
    ap = argparse.ArgumentParser(
        description="Generate labeled navigation tags for Instant Robot.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Each card has ArUco markers 200 (left) and 201 (right) flanking a text region.
The top half is the LABEL (station identity), the bottom half is a DESCRIPTION
(extra context for Gemini AI). The app reads the text via homography + OCR.

Examples:
  uv run python make_labeled_tags.py
  uv run python make_labeled_tags.py --tags "Apple:Red fruit" "Basket:Drop zone"
  uv run python make_labeled_tags.py --size 50
        """,
    )
    ap.add_argument(
        "--tags", type=str, nargs="+", default=None,
        help='Tag specs as "Label:Description" pairs. Default: built-in set of 7.',
    )
    ap.add_argument("--size", type=float, default=40.0,
                    help="ArUco marker side (mm). Default: 40")
    ap.add_argument("--dict", type=str, default="DICT_6X6_250",
                    help="ArUco dictionary")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--out", type=str, default="printables/nav_tags.pdf")
    args = ap.parse_args()

    if args.tags:
        tags = [parse_tag_spec(s) for s in args.tags]
    else:
        tags = DEFAULT_TAGS

    if not hasattr(aruco, args.dict):
        raise SystemExit(f"Unknown dictionary '{args.dict}'.")

    print(f"Generating {len(tags)} labeled nav tags "
          f"(markers {LEFT_ID} & {RIGHT_ID}, {args.size:.0f} mm):")
    for label, desc in tags:
        print(f"  • {label}" + (f"  —  {desc}" if desc else ""))

    pages = build_pages(tags, args.size, args.dict, dpi=args.dpi)
    save_pdf(pages, args.out, dpi=args.dpi)


if __name__ == "__main__":
    main()
